/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    filters.h
  * @brief   Multi-average / FIR / Kalman filters for 3-axis channels
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __FILTERS_H
#define __FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  FILTER_AVG = 0,   /* multi-sample moving average */
  FILTER_FIR = 1,   /* 15-tap low-pass FIR, fc=0.1fs */
  FILTER_KALMAN = 2 /* 1-D Kalman per channel      */
} FilterType_t;

#define FILTER_CHANNELS 6   /* accel xyz + gyro xyz */

void     Filters_Init(void);
void     Filters_SetType(FilterType_t t);
FilterType_t Filters_GetType(void);
/* Push one sample set (6 floats), get filtered result back */
void     Filters_Process(const float in[FILTER_CHANNELS], float out[FILTER_CHANNELS]);

#ifdef __cplusplus
}
#endif

#endif /* __FILTERS_H */
