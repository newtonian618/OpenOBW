#include "openobw_app.h"
#include "params.h"

#define REQUEST_HEAD 0xAAU
#define RESPONSE_HEAD 0x55U
#define INTERBYTE_MS 20U

typedef struct {
  uint32_t tick;
  uint8_t byte;
  bool invalid;
} ReceivedByte;

static osMessageQueueId_t receive_queue;
static osMutexId_t transmit_mutex;
static volatile bool transmitting;

bool Protocol_Init(void)
{
  receive_queue = osMessageQueueNew(64, sizeof(ReceivedByte), NULL);
  transmit_mutex = osMutexNew(NULL);
  return receive_queue != NULL && transmit_mutex != NULL;
}

void Protocol_RxInterrupt(void)
{
  /* Reading SR then DR clears RXNE and line errors on STM32F1. */
  uint32_t status = USART3->SR;
  uint8_t byte = (uint8_t)USART3->DR;
  if (!transmitting) {
    ReceivedByte item = {
      .tick = osKernelGetTickCount(),
      .byte = byte,
      .invalid = (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0
    };
    osMessageQueuePut(receive_queue, &item, 0, 0);
  }
}

static void send_frame(uint8_t command, const uint8_t *payload, uint8_t length)
{
  uint8_t frame[14] = {RESPONSE_HEAD, g_params.dev_addr, command, length};
  uint8_t checksum = frame[1] ^ command ^ length;
  for (uint8_t i = 0; i < length; ++i) {
    frame[4 + i] = payload[i];
    checksum ^= payload[i];
  }
  frame[4 + length] = checksum;
  transmitting = true;
  HAL_UART_Transmit(&huart3, frame, (uint16_t)(5 + length), 30);
  transmitting = false;
}

void Protocol_SendEvent(uint8_t code, int16_t value)
{
  uint8_t payload[] = {code, (uint8_t)((uint16_t)value >> 8), (uint8_t)value};
  osMutexAcquire(transmit_mutex, osWaitForever);
  send_frame(0x7F, payload, sizeof(payload));
  osMutexRelease(transmit_mutex);
}

static void put_word(uint8_t *output, uint16_t value)
{
  output[0] = (uint8_t)(value >> 8);
  output[1] = (uint8_t)value;
}

static bool save_parameters(uint8_t address, FilterType_t filter)
{
  if (!Params_Save(address, filter)) return false;
  g_params.dev_addr = address;
  osMutexAcquire(gsMutexHandle, osWaitForever);
  if (Filters_GetType() != filter) Filters_SetType(filter);
  g_params.filter_type = filter;
  osMutexRelease(gsMutexHandle);
  return true;
}

static void process_request(const uint8_t *frame)
{
  uint8_t address = frame[1], command = frame[2], length = frame[3];
  uint8_t checksum = address ^ command ^ length;
  for (unsigned i = 0; i < length; ++i) checksum ^= frame[4 + i];
  if (checksum != frame[4 + length] ||
      (address != g_params.dev_addr && address != 0xFF)) return;

  uint8_t payload[9] = {0};
  uint8_t response_length = 1;
  uint8_t expected_length = (command == 0x05 || command == 0x07) ? 1 : 0;
  Measurement sample;

  osMutexAcquire(transmit_mutex, osWaitForever);
  if (address == g_params.dev_addr) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  if (command < 1 || command > 0x0A) {
    payload[0] = 1;
  } else if (length != expected_length) {
    payload[0] = 2;
  } else {
    switch (command) {
      case 0x01:
      case 0x02:
        App_GetMeasurement(&sample);
        if (!sample.motion_valid) {
          payload[0] = 2;
          break;
        }
        for (unsigned i = 0; i < 3; ++i) {
          put_word(&payload[1 + 2 * i], (uint16_t)sample.motion[3 * (command - 1) + i]);
        }
        response_length = 7;
        break;
      case 0x03:
        App_GetMeasurement(&sample);
        if (!sample.analog_valid) {
          payload[0] = 2;
          break;
        }
        put_word(&payload[1], (uint16_t)(int16_t)(sample.temperature * 100.0f +
                     (sample.temperature < 0.0f ? -0.5f : 0.5f)));
        put_word(&payload[3], (uint16_t)(sample.voltage * 100.0f + 0.5f));
        response_length = 5;
        break;
      case 0x04:
        payload[1] = g_params.dev_addr;
        response_length = 2;
        break;
      case 0x05:
        if (frame[4] == 0 || frame[4] == 255 ||
            !save_parameters(frame[4], g_params.filter_type)) payload[0] = 2;
        break;
      case 0x06:
        payload[1] = (uint8_t)g_params.filter_type;
        response_length = 2;
        break;
      case 0x07:
        if (frame[4] > FILTER_KALMAN ||
            !save_parameters(g_params.dev_addr, (FilterType_t)frame[4])) payload[0] = 2;
        break;
      case 0x08:
        if (!save_parameters(g_params.dev_addr, g_params.filter_type)) payload[0] = 2;
        break;
      case 0x09:
        payload[1] = 1;
        response_length = 4;
        break;
      case 0x0A:
        for (unsigned i = 0; i < 8; ++i) payload[1 + i] = g_params.uid[i];
        response_length = 9;
        break;
    }
  }
  send_frame(command, payload, response_length);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  osMutexRelease(transmit_mutex);
}

void Protocol_Run(void)
{
  uint8_t frame[260];
  unsigned used = 0;
  uint32_t last_tick = 0;
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  for (;;) {
    ReceivedByte item;
    if (osMessageQueueGet(receive_queue, &item, NULL, osWaitForever) != osOK) continue;
    if (item.invalid || item.tick - last_tick >= INTERBYTE_MS) used = 0;
    last_tick = item.tick;
    if (item.invalid || (used == 0 && item.byte != REQUEST_HEAD)) continue;
    frame[used++] = item.byte;
    if (used >= 5 && used == 5U + frame[3]) {
      process_request(frame);
      used = 0;
    }
  }
}
