#include "openobw_app.h"
#include "params.h"
#include <math.h>

#define ACC_READY 1U
#define GYRO_READY 2U
#define WINDOW_MS 100U

extern osThreadId_t defaultTaskHandle;
static Measurement latest;
static volatile bool adc_complete;
static uint16_t adc_samples[3] __attribute__((aligned(4)));

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == INTA1_Pin) {
    osThreadFlagsSet(defaultTaskHandle, ACC_READY);
  } else if (pin == INTG1_Pin) {
    osThreadFlagsSet(defaultTaskHandle, GYRO_READY);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
  if (adc == &hadc1) adc_complete = true;
}

static bool read_analog(Measurement *measurement)
{
  adc_complete = false;
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_samples, 3) != HAL_OK) return false;
  for (unsigned attempt = 0; !adc_complete && attempt < 3; ++attempt) osDelay(1);
  /* A normal DMA transfer is already complete here. Stop the ADC between scans. */
  if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK || !adc_complete) return false;

  uint16_t ntc = adc_samples[1];
  uint16_t reference = adc_samples[2];
  if (reference == 0 || ntc == 0 || ntc >= 4095) return false;

  /* Upper-leg NTC and equal 10 kOhm nominal lower-leg resistance. */
  float ratio = (4095.0f - ntc) / ntc;
  measurement->temperature = 1.0f / (1.0f / 298.15f + logf(ratio) / 3450.0f) - 273.15f;
  /* STM32F103 has no factory VREFINT calibration word. Use nominal 1.20 V. */
  measurement->voltage = 12.0f * 1.20f * adc_samples[0] / reference;
  return true;
}

static bool collect(bool accel, int32_t sums[3], uint32_t *count, bool keep)
{
  int16_t axes[3];
  if (!Bmi088_Read(accel, axes)) return false;
  if (keep) {
    for (unsigned i = 0; i < 3; ++i) sums[i] += axes[i];
    ++*count;
  }
  return true;
}

static int16_t round_axis(float value)
{
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)(value + (value < 0.0f ? -0.5f : 0.5f));
}

void App_Acquire(void)
{
  bool keep_gyro = false;
  uint32_t deadline = osKernelGetTickCount() + WINDOW_MS;
  osThreadFlagsClear(ACC_READY | GYRO_READY);
  __HAL_GPIO_EXTI_CLEAR_IT(INTA1_Pin | INTG1_Pin);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  for (;;) {
    int32_t sums[2][3] = {{0}};
    uint32_t count[2] = {0};
    bool spi_ok = true;

    for (;;) {
      int32_t remaining = (int32_t)(deadline - osKernelGetTickCount());
      if (remaining <= 0) break;
      uint32_t flags = osThreadFlagsWait(ACC_READY | GYRO_READY, osFlagsWaitAny,
                                         (uint32_t)remaining);
      if (flags & osFlagsError) break;
      if ((flags & ACC_READY) && !collect(true, sums[0], &count[0], true)) spi_ok = false;
      if (flags & GYRO_READY) {
        /* Read every DRDY to acknowledge the interrupt. Keep every second sample. */
        keep_gyro = !keep_gyro;
        if (!collect(false, sums[1], &count[1], keep_gyro)) spi_ok = false;
      }
    }

    Measurement next = {0};
    next.motion_valid = spi_ok && count[0] >= 4 && count[1] >= 4 &&
                        Bmi088_IsHealthy();
    next.analog_valid = read_analog(&next);

    osMutexAcquire(gsMutexHandle, osWaitForever);
    if (next.motion_valid) {
      float average[6], filtered[6];
      for (unsigned axis = 0; axis < 3; ++axis) {
        /* Convert BMI088 counts to the BTA-6 wire units. */
        average[axis] = ((float)sums[0][axis] / count[0]) * (3000.0f / 32768.0f / 0.061f);
        average[axis + 3] = ((float)sums[1][axis] / count[1]) * (250000.0f / 32768.0f / 8.75f);
      }
      const float *output = average;
      if (Filters_GetType() != FILTER_AVG) {
        Filters_Process(average, filtered);
        output = filtered;
      }
      for (unsigned i = 0; i < 6; ++i) next.motion[i] = round_axis(output[i]);
    }
    latest = next;
    osMutexRelease(gsMutexHandle);
    HAL_IWDG_Refresh(&hiwdg);
    deadline += WINDOW_MS;
    if ((int32_t)(deadline - osKernelGetTickCount()) <= 0) {
      deadline = osKernelGetTickCount() + WINDOW_MS;
    }
  }
}

void App_GetMeasurement(Measurement *measurement)
{
  osMutexAcquire(gsMutexHandle, osWaitForever);
  *measurement = latest;
  osMutexRelease(gsMutexHandle);
}

void App_Monitor(void)
{
  uint8_t previous_voltage = 0;
  uint8_t previous_temperature = 0;
  osDelay(WINDOW_MS + 20);
  for (;;) {
    Measurement sample;
    App_GetMeasurement(&sample);

    uint8_t voltage = sample.voltage > 36.0f ? 0x01 : sample.voltage < 10.0f ? 0x02 : 0;
    uint8_t temperature = sample.temperature > 75.0f ? 0x03 : sample.temperature < -20.0f ? 0x04 : 0;
    if (!sample.analog_valid || !sample.motion_valid || voltage || temperature) {
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    } else {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }

    if (sample.analog_valid && voltage != previous_voltage) {
      Protocol_SendEvent(voltage ? voltage : 0x11, (int16_t)lroundf(sample.voltage * 100.0f));
      previous_voltage = voltage;
    }
    if (sample.analog_valid && temperature != previous_temperature) {
      Protocol_SendEvent(temperature ? temperature : 0x12,
                         (int16_t)lroundf(sample.temperature * 100.0f));
      previous_temperature = temperature;
    }
    osDelay(250);
  }
}
