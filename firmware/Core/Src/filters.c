/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    filters.c
  * @brief   Moving-average / FIR / Kalman filter implementations
  ******************************************************************************
  */
/* USER CODE END Header */
#include "filters.h"
#include <string.h>

#define AVG_LEN   8
#define FIR_LEN   15

/* 15-tap symmetric low-pass FIR (14th order), fc = 0.1 fs,
   Blackman-windowed sinc, normalized so DC gain = 1.0 (sum = 1.0) */
static const float fir_taps[FIR_LEN] = {
  0.0000f, -0.0007f, 0.0000f, 0.0127f, 0.0530f, 0.1236f, 0.1970f, 0.2288f,
  0.1970f,  0.1236f, 0.0530f, 0.0127f, 0.0000f, -0.0007f, 0.0000f
};

/* Kalman tuning: Q = process noise, R = measurement noise */
#define KALMAN_Q 0.01f
#define KALMAN_R 4.0f

typedef struct {
  float    buf[FIR_LEN];   /* ring buffer shared by AVG and FIR */
  uint8_t  idx;
  uint8_t  filled;
  float    kf_x;           /* Kalman estimate */
  float    kf_p;           /* Kalman error covariance */
  uint8_t  kf_init;
} ChanFilter_t;

static ChanFilter_t ch[FILTER_CHANNELS];
static FilterType_t cur_type = FILTER_AVG;

void Filters_Init(void)
{
  memset(ch, 0, sizeof(ch));
  cur_type = FILTER_AVG;
}

void Filters_SetType(FilterType_t t)
{
  if (t <= FILTER_KALMAN) {
    cur_type = t;
    for (int i = 0; i < FILTER_CHANNELS; i++) {
      ch[i].idx = 0;
      ch[i].filled = 0;
      ch[i].kf_init = 0;
      ch[i].kf_p = 1.0f;
    }
  }
}

FilterType_t Filters_GetType(void)
{
  return cur_type;
}

void Filters_Process(const float in[FILTER_CHANNELS], float out[FILTER_CHANNELS])
{
  for (int i = 0; i < FILTER_CHANNELS; i++) {
    ChanFilter_t *c = &ch[i];
    float x = in[i];

    /* feed ring buffer (used by AVG and FIR) */
    c->buf[c->idx] = x;
    c->idx = (uint8_t)((c->idx + 1) % FIR_LEN);
    if (c->filled < FIR_LEN) c->filled++;

    switch (cur_type) {
    case FILTER_FIR: {
      float acc = 0.0f;
      uint8_t n = c->filled;
      for (uint8_t k = 0; k < n; k++) {
        /* newest sample gets tap[0] */
        uint8_t pos = (uint8_t)((c->idx + FIR_LEN - 1 - k) % FIR_LEN);
        acc += fir_taps[k] * c->buf[pos];
      }
      out[i] = acc;
      break;
    }
    case FILTER_KALMAN: {
      if (!c->kf_init) {
        c->kf_x = x;
        c->kf_p = 1.0f;
        c->kf_init = 1;
      }
      c->kf_p += KALMAN_Q;
      float k = c->kf_p / (c->kf_p + KALMAN_R);
      c->kf_x += k * (x - c->kf_x);
      c->kf_p *= (1.0f - k);
      out[i] = c->kf_x;
      break;
    }
    case FILTER_AVG:
    default: {
      float acc = 0.0f;
      uint8_t n = (c->filled < AVG_LEN) ? c->filled : AVG_LEN;
      for (uint8_t k = 0; k < n; k++) {
        acc += c->buf[k];
      }
      out[i] = (n > 0) ? acc / (float)n : x;
      break;
    }
    }
  }
}
