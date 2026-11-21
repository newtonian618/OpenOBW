/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    params.h
  * @brief   Non-volatile device parameters (stored in internal Flash)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __PARAMS_H
#define __PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "filters.h"

typedef struct {
  uint8_t      dev_addr;     /* device address 1..254 */
  FilterType_t filter_type;  /* active filter */
  uint8_t      uid[8];       /* unique device ID: FNV-1a64 of the 96-bit chip UID */
} DeviceParams_t;

extern DeviceParams_t g_params;

void Params_Load(void);
bool Params_Save(uint8_t address, FilterType_t filter);

#ifdef __cplusplus
}
#endif

#endif /* __PARAMS_H */

