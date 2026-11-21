#ifndef OPENOBW_APP_H
#define OPENOBW_APP_H

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>

typedef struct {
  int16_t motion[6];
  float temperature;
  float voltage;
  bool motion_valid;
  bool analog_valid;
} Measurement;

extern ADC_HandleTypeDef hadc1;
extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart3;
extern osMutexId_t gsMutexHandle;

bool Bmi088_Init(void);
bool Bmi088_IsHealthy(void);
bool Bmi088_Read(bool accelerometer, int16_t axes[3]);
void App_Acquire(void);
void App_GetMeasurement(Measurement *measurement);
void App_Monitor(void);
bool Protocol_Init(void);
void Protocol_Run(void);
void Protocol_RxInterrupt(void);
void Protocol_SendEvent(uint8_t code, int16_t value);

#endif

