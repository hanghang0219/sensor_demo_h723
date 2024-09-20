

#include <SEGGER_RTT.h>
#include <lsm6dsow.h>
#include <lsm6dsow_motionFX.h>
#include "cmsis_os2.h"

Lsm6dsow lsm6dsow;
osMessageQueueId_t sig_0_qid;

const osThreadAttr_t main_attributes = {
    .name = "main",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t thread_attributes = {
    .name = "main",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void I2C2_Callback_Init(I2C_HandleTypeDef* hi2c);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_15) {
    /* FIFO is full now
       this fun can check Fifo status :  lsm6dsowFifoStatus(&lsm6dsow); */
    lsm6dsow.callback->wtm_cur = LSM6DSOW_FIFO_WATERMARK;
    lsm6dsow.callback->rx_flag = DATA_OUT_TYPE_FLAG;

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read_IT(lsm6dsow.i2c, lsm6dsow.reg_cmd->read, LSM6DSOW_FIFO_DATA_OUT_TAG_REG,
                                               I2C_MEMADD_SIZE_8BIT, lsm6dsow.callback->status_buf, 1);

    if (st != HAL_OK) {
      SEGGER_RTT_printf(0, "%s lsm6dsow INT1 it error: %x \n%s", RTT_CTRL_TEXT_BRIGHT_BLUE, st, RTT_CTRL_RESET);
    }
  }
}

typedef bool (*signalHandle)(void*);

typedef struct {
  signalHandle handle;
  void* parameter;
} SignalEvent;

__NO_RETURN void mainThread(void* arg) {

  SignalEvent event[1] = {{.handle = lsm6dsowI2cReadCallback, .parameter = &lsm6dsow}};

  lsm6dsowInit(&lsm6dsow, &hi2c2);
  lsm6dsowRegInit(&lsm6dsow);

  uint8_t sig = 0;
  osStatus_t st;
  while (1) {
    st = osMessageQueueGet(sig_0_qid, &sig, 0, 3600000);

    if (st == osOK) {
      event[sig].handle(event[sig].parameter);
    }

    osDelay(1);
  }
}

__NO_RETURN void threadThread(void* arg) {
  osDelay(3000);
  while (1) {}
}

void MX_FREERTOS_Init(void) {
  SEGGER_RTT_Init();
  I2C2_Callback_Init(&hi2c2);
  lsm6ds3trMotionFxInit();

  sig_0_qid = osMessageQueueNew(4, 1, NULL);
  SEGGER_RTT_printf(0, "%s welcome to fly  \n%s", RTT_CTRL_TEXT_BRIGHT_BLUE, RTT_CTRL_RESET);

  osThreadNew(mainThread, NULL, &main_attributes);
}

void i2c2_rx_callback(I2C_HandleTypeDef* hi2c) {
  uint8_t signal = 0;
  osMessageQueuePut(sig_0_qid, &signal, 0, 0);
}

void I2C2_Callback_Init(I2C_HandleTypeDef* hi2c) {
  HAL_I2C_RegisterCallback(hi2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID, i2c2_rx_callback);
}