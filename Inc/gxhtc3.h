//
// Created by hanghang on 18/06/2024.
//

#ifndef H723_SSR_GXHTC3_H
#define H723_SSR_GXHTC3_H

#include "main.h"
#include "cmsis_os2.h"

#define GXHTC3_ADD     0x70
#define GXHTC3_SEND_H  0x78
#define GXHTC3_SEND_L  0x66

#define SIG_GXHTC3_TXCB  0x00
#define SIG_GXHTC3_RXCB  0x01
#define SIG_GXHTC3_ERRCB 0x02

#define GXHTC3_FLAG_SAT  0X00
#define GXHTC3_FLAG_END  0X01

typedef enum {
  GXHTC3_OK,
  GXHTC3_SEND_ERR,
  GXHTC3_RECEIVE_ERR,
  GXHTC3_COUNT_ERR,
  GXHTC3_ERR,
} Gxhtc3Err;


typedef struct {
  I2C_HandleTypeDef* i2c;
  Gxhtc3Err err_st;
  osMessageQueueId_t queue;
  uint8_t data[6];
  uint8_t flag;
} Gxhtc3;

typedef void (* SignalHandler)(void);


void gxhtc3Thread(void* arg);
void gxhtc3Init(I2C_HandleTypeDef* i2c, osMessageQueueId_t queue);

#endif //H723_SSR_GXHTC3_H
