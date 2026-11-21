#include "openobw_app.h"
#include "params.h"

#define PARAM_FLASH_ADDRESS 0x0800FC00U
#define PARAM_MAGIC 0x4F42U

DeviceParams_t g_params;

void Params_Load(void)
{
  const uint8_t *uid = (const uint8_t *)UID_BASE;
  uint64_t hash = 0xcbf29ce484222325ULL;
  for (unsigned i = 0; i < 12; ++i) {
    hash ^= uid[i];
    hash *= 0x100000001b3ULL;
  }
  for (unsigned i = 0; i < 8; ++i) g_params.uid[i] = (uint8_t)(hash >> (56 - 8 * i));

  const volatile uint16_t *record = (const volatile uint16_t *)PARAM_FLASH_ADDRESS;
  uint16_t packed = record[1];
  uint8_t address = (uint8_t)packed;
  unsigned filter = packed >> 8;
  g_params.dev_addr = 1;
  g_params.filter_type = FILTER_AVG;
  if (record[0] == PARAM_MAGIC && address >= 1 && address <= 254 && filter <= FILTER_KALMAN) {
    g_params.dev_addr = address;
    g_params.filter_type = (FilterType_t)filter;
  }
}

bool Params_Save(uint8_t address, FilterType_t filter)
{
  const volatile uint16_t *record = (const volatile uint16_t *)PARAM_FLASH_ADDRESS;
  uint16_t packed = (uint16_t)(address | ((uint16_t)filter << 8));
  if (record[0] == PARAM_MAGIC && record[1] == packed) return true;
  if (HAL_FLASH_Unlock() != HAL_OK) return false;

  FLASH_EraseInitTypeDef erase = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .PageAddress = PARAM_FLASH_ADDRESS,
    .NbPages = 1
  };
  uint32_t failed_page;
  bool ok = HAL_FLASHEx_Erase(&erase, &failed_page) == HAL_OK &&
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PARAM_FLASH_ADDRESS + 2, packed) == HAL_OK &&
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PARAM_FLASH_ADDRESS, PARAM_MAGIC) == HAL_OK;
  HAL_FLASH_Lock();
  return ok && record[0] == PARAM_MAGIC && record[1] == packed;
}
