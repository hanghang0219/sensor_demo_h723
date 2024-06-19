#include "cmsis_os2.h"
#include "stm32h723xx.h"
#include "gxhtc3.h"
#include "i2c.h"


const osThreadAttr_t main_attributes = {
        .name = "main",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

osMessageQueueId_t I2C1_queue;


void i2cSendHandler(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1) {
    uint8_t signal = SIG_GXHTC3_TXCB;
    osMessageQueuePut(I2C1_queue, &signal, 0, 0);
  }
}


void i2cReceiveHandler(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1) {
    uint8_t signal = SIG_GXHTC3_RXCB;
    osMessageQueuePut(I2C1_queue, &signal, 0, 0);
  }
}


void i2cErrHandler(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1) {
    uint8_t signal = SIG_GXHTC3_ERRCB;
    osMessageQueuePut(I2C1_queue, &signal, 0, 0);
  }
}

void ZM_I2C1_Init(I2C_HandleTypeDef* hi2c1) {
  HAL_I2C_RegisterCallback(hi2c1, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, i2cSendHandler);
  HAL_I2C_RegisterCallback(hi2c1, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, i2cReceiveHandler);
  HAL_I2C_RegisterCallback(hi2c1, HAL_I2C_ERROR_CB_ID, i2cErrHandler);
}

void MX_FREERTOS_Init(void) {
  I2C1_queue = osMessageQueueNew(8, 1, NULL);

  ZM_I2C1_Init(&hi2c1);
  gxhtc3Init(&hi2c1, I2C1_queue);


  osThreadNew(gxhtc3Thread, NULL, &main_attributes);
}




