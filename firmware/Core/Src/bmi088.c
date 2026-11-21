#include "openobw_app.h"

extern SPI_HandleTypeDef hspi1;

static bool transfer(bool accel, uint8_t *tx, uint8_t *rx, uint16_t size)
{
  GPIO_TypeDef *port = accel ? nCSBA_GPIO_Port : nCSBG_GPIO_Port;
  uint16_t pin = accel ? nCSBA_Pin : nCSBG_Pin;
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, size, 3);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  return status == HAL_OK;
}

static bool read_register(bool accel, uint8_t reg, uint8_t *value)
{
  uint8_t tx[3] = {reg | 0x80U}, rx[3];
  uint16_t size = accel ? 3 : 2;
  if (!transfer(accel, tx, rx, size)) return false;
  *value = rx[size - 1];
  return true;
}

static bool configure(bool accel, uint8_t reg, uint8_t value, uint8_t mask,
                      uint32_t settling_ms)
{
  uint8_t tx[2] = {reg, value}, rx[2], actual;
  if (!transfer(accel, tx, rx, sizeof(tx))) return false;
  HAL_Delay(settling_ms);
  return read_register(accel, reg, &actual) && (actual & mask) == value;
}

static bool reset(bool accel, uint8_t reg)
{
  uint8_t tx[2] = {reg, 0xB6}, rx[2];
  if (!transfer(accel, tx, rx, sizeof(tx))) return false;
  HAL_Delay(50);
  return true;
}

bool Bmi088_Init(void)
{
  uint8_t id;
  HAL_Delay(50);
  /* CSB1 rising edge after the dummy read selects accelerometer SPI mode. */
  if (!read_register(true, 0, &id) || !reset(true, 0x7E) ||
      !reset(false, 0x14) || !read_register(true, 0, &id) ||
      !read_register(true, 0, &id) || id != 0x1E ||
      !read_register(false, 0, &id) || id != 0x0F) return false;

  /* Accelerometer: active, +/-3 g, normal bandwidth at 50 Hz, INT1 DRDY. */
  if (!configure(true, 0x7C, 0x00, 0x03, 5) ||
      !configure(true, 0x7D, 0x04, 0x04, 50) ||
      !configure(true, 0x41, 0x00, 0x03, 2) ||
      !configure(true, 0x40, 0xA7, 0xFF, 2) ||
      !configure(true, 0x53, 0x0A, 0x1F, 2) ||
      !configure(true, 0x58, 0x04, 0x44, 2)) return false;

  /* Gyroscope: +/-250 dps, 100 Hz ODR, 12 Hz bandwidth, INT3 DRDY. */
  return configure(false, 0x0F, 0x03, 0x07, 2) &&
         configure(false, 0x10, 0x05, 0x07, 2) &&
         configure(false, 0x16, 0x01, 0x0F, 2) &&
         configure(false, 0x18, 0x01, 0x81, 2) &&
         configure(false, 0x15, 0x80, 0xC0, 2) && Bmi088_IsHealthy();
}

bool Bmi088_Read(bool accel, int16_t axes[3])
{
  uint8_t tx[8] = {accel ? 0x92 : 0x82}, rx[8];
  uint16_t offset = accel ? 2 : 1;
  if (!transfer(accel, tx, rx, offset + 6)) return false;
  for (unsigned i = 0; i < 3; ++i) {
    unsigned index = offset + 2 * i;
    axes[i] = (int16_t)(rx[index] | ((uint16_t)rx[index + 1] << 8));
  }
  return true;
}

bool Bmi088_IsHealthy(void)
{
  uint8_t id, error;
  return read_register(true, 0, &id) && id == 0x1E &&
         read_register(true, 0x02, &error) && (error & 0x1D) == 0 &&
         read_register(false, 0, &id) && id == 0x0F;
}
