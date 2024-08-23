#include <lsm6ds3tr.h>
#include <lsm6ds3tr_motionFX.h>
#include "SEGGER_RTT.h"
#include "cmsis_os2.h"

osMessageQueueId_t sig_qid;
Lsm6ds3tr lsm6ds3tr;

const osThreadAttr_t main_attributes = {
    .name = "main",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t thread_attributes = {
    .name = "main",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void lsm6ds3tr_i2c_rx_cmp_cb(__attribute__((unused)) I2C_HandleTypeDef* i2c) {
  uint8_t signal = 0;
  osMessageQueuePut(sig_qid, &signal, 0, 0);
}

void CS_I2C1_CB_Init(I2C_HandleTypeDef* i2c) {
  // HAL_I2C_RegisterCallback(i2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID, lsm6ds3tr_i2c_rx_cmp_cb);
  HAL_I2C_RegisterCallback(i2c, HAL_I2C_MEM_TX_COMPLETE_CB_ID, lsm6ds3tr_i2c_rx_cmp_cb);
}

typedef void (*SigHandler)(void*);

__NO_RETURN void mainThread(void* arg) {
  lsm6ds3trInit(&lsm6ds3tr, &hi2c1);
  CS_I2C1_CB_Init(lsm6ds3tr.i2c);
  lsm6ds3trMotionFxInit();

  if (lsm6ds3trRegInit(&lsm6ds3tr)) {
    SEGGER_RTT_printf(0, "%s lsm6ds3tr_reg_init err : %d", RTT_CTRL_TEXT_BRIGHT_BLUE, lsm6ds3tr.err_st);
    return;
  }

  // lsm6ds3trAccGryOutput(&lsm6ds3tr);
  osStatus_t st;
  uint8_t sig = 0;
  SigHandler sig_handler[1] = {lsm6ds3trFifoResult};

  osDelay(100);
  lsm6ds3trFifoClear(&lsm6ds3tr);
  while (1) {

    st = osMessageQueueGet(sig_qid, &sig, 0, 3600000);
    if (st == osOK) {
      sig_handler[sig](&lsm6ds3tr);
    }
  }
}

__NO_RETURN void threadThread(void* arg) {
  uint8_t a = 10000;

  osDelay(3000);
  while (1) {}
}

void MX_FREERTOS_Init(void) {
  sig_qid = osMessageQueueNew(4, 1, NULL);

  osThreadNew(mainThread, NULL, &main_attributes);
  //  osThreadNew(threadThread, NULL, &thread_attributes);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  static uint32_t num;
  switch (GPIO_Pin) {
    case GPIO_PIN_0:
      break;
    case GPIO_PIN_1:
      uint8_t signal = 0;
      osMessageQueuePut(sig_qid, &signal, 0, 0);
      SEGGER_RTT_printf(0, "%sgpio it %d \n", RTT_CTRL_TEXT_BRIGHT_BLUE, num);
      num++;
      break;
    case GPIO_PIN_2:

      break;
    case GPIO_PIN_3:

      break;
    case GPIO_PIN_4:

      break;
    case GPIO_PIN_5:

      break;
    case GPIO_PIN_6:

      break;
    case GPIO_PIN_7:

      break;
    case GPIO_PIN_8:

      break;
    case GPIO_PIN_9:

      break;
    case GPIO_PIN_10:

      break;
    case GPIO_PIN_11:

      break;
    case GPIO_PIN_12:

      break;
    case GPIO_PIN_13:

    case GPIO_PIN_14:

      break;
    case GPIO_PIN_15:
      break;
    default:
      break;
  }
}