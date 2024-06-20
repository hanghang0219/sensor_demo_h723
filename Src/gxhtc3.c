//
// Created by hanghang on 18/06/2024.
//
#include <stdio.h>
#include <string.h>
#include "gxhtc3.h"
#include "usart.h"
#include "SEGGER_RTT.h"


Gxhtc3 gxhtc3;

/* usart printf to check outcome */
HAL_StatusTypeDef HAL_Printf(uint8_t* pData) {
  return HAL_UART_Transmit(&huart1, pData, strlen((char*) pData), 1000);
}

void gxhtc3Init(I2C_HandleTypeDef* i2c, osMessageQueueId_t queue) {
  gxhtc3.i2c = i2c;
  gxhtc3.err_st = GXHTC3_OK;
  gxhtc3.flag = GXHTC3_FLAG_SAT;
  gxhtc3.queue = queue;
}

/* start ask gxhtc3 */
void masterToGxhtc3(void) {
  uint8_t cmd[2];
  cmd[0] = GXHTC3_SEND_H;
  cmd[1] = GXHTC3_SEND_L;

  HAL_StatusTypeDef st = HAL_I2C_Master_Transmit_IT(gxhtc3.i2c, GXHTC3_ADD << 1, cmd, sizeof(cmd));
  gxhtc3.err_st = st == HAL_OK ? GXHTC3_OK : GXHTC3_SEND_ERR;
}

/* wait gxhtc3 response */
void gxhtc3ToMaster(void) {
  osDelay(10);

  HAL_StatusTypeDef st = HAL_I2C_Master_Receive_IT(gxhtc3.i2c, GXHTC3_ADD << 1, gxhtc3.data, sizeof(gxhtc3.data));
  gxhtc3.err_st = st == HAL_OK ? GXHTC3_OK : GXHTC3_RECEIVE_ERR;
}

/* deal response data */
void gxhtc3Data(void) {
  uint16_t tem, hum;
  float Temperature = 0;
  float Humidity = 0;
  char data_buf[20];

  tem = ((gxhtc3.data[0] << 8) | gxhtc3.data[1]);//温度拼接
  hum = ((gxhtc3.data[3] << 8) | gxhtc3.data[4]);//湿度拼接

  Temperature = (175.0f * (float) tem / 65535.0f - 45.0f);// T = -45 + 175 * tem / (2^16-1)
  Humidity = (100.0f * (float) hum / 65535.0f);// RH = hum*100 / (2^16-1)

  sprintf(data_buf, "temp:%d.%d℃\r\n", (uint) Temperature, (uint) ((Temperature - (float) (uint) Temperature) * 100));
  HAL_Printf((uint8_t*) data_buf);
  sprintf(data_buf, "hum:%d.%d%%\r\n", (uint) Humidity, (uint) ((Humidity - (float) (uint) Humidity) * 100));
  HAL_Printf((uint8_t*) data_buf);
}

void gxhtc3TxCb(void) {
  gxhtc3.err_st = GXHTC3_OK;
  gxhtc3.flag = GXHTC3_FLAG_SAT;
  gxhtc3ToMaster();
}

void gxhtc3RxCb(void) {
  gxhtc3.err_st = GXHTC3_OK;
  gxhtc3.flag = GXHTC3_FLAG_END;
  gxhtc3Data();
}

void gxhtc3ErrCb(void) {
  gxhtc3.err_st = GXHTC3_ERR;
  SEGGER_RTT_printf(0, "ERR\r\n");
}


SignalHandler handler[3] = {gxhtc3TxCb, gxhtc3RxCb, gxhtc3ErrCb};


__NO_RETURN void gxhtc3Thread(void* arg) {
  uint8_t signal;
  masterToGxhtc3();


  while (1) {
    if (gxhtc3.flag == GXHTC3_FLAG_END || gxhtc3.err_st != GXHTC3_OK) {
      masterToGxhtc3();
    }

    if (osMessageQueueGet(gxhtc3.queue, &signal, 0, 10) == osOK)
      handler[signal]();

    osThreadYield();
  }
}

