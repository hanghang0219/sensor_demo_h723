#include <lsm6ds3tr.h>
#include "SEGGER_RTT.h"
#include "cmsis_os2.h"

osMessageQueueId_t sig_qid;
Lsm6ds3tr lsm6ds3tr;

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

void lsm6ds3tr_i2c_rx_cmp_cb(__attribute__((unused)) I2C_HandleTypeDef* i2c) {
  uint8_t signal = 0;
  osMessageQueuePut(sig_qid, &signal, 0, 0);
}

void CS_I2C1_CB_Init(I2C_HandleTypeDef* i2c) {
  HAL_I2C_RegisterCallback(i2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID,
                           lsm6ds3tr_i2c_rx_cmp_cb);
  HAL_I2C_RegisterCallback(i2c, HAL_I2C_MEM_TX_COMPLETE_CB_ID,
                           lsm6ds3tr_i2c_rx_cmp_cb);
}

typedef void (*SigHandler)(void*);

__NO_RETURN void mainThread(void* arg) {
  lsm6ds3trInit(&lsm6ds3tr, &hi2c1);
  CS_I2C1_CB_Init(lsm6ds3tr.i2c);
  if (lsm6ds3trRegInit(&lsm6ds3tr)) {
    SEGGER_RTT_printf(0, "%s lsm6ds3tr_reg_init err : %d",
                      RTT_CTRL_TEXT_BRIGHT_BLUE, lsm6ds3tr.err_st);
    return;
  }
  lsm6ds3trAccGryOutput(&lsm6ds3tr);

  osStatus_t st;
  uint8_t sig = 0;

  SigHandler sig_handler[2] = {lsm6ds3trOutputReadCb, lsm6ds3trAccGryOutput};

  while (1) {
    st = osMessageQueueGet(sig_qid, &sig, 0, 3600000);
    if (st == osOK) {
      sig_handler[sig](&lsm6ds3tr);
      if (sig == 0) {
        uint8_t signal = 1;
        osMessageQueuePut(sig_qid, &signal, 0, 10);
      }
    }
  }
}

__NO_RETURN void threadThread(void* arg) {

  osDelay(3000);
  while (1) {}
}

void MX_FREERTOS_Init(void) {
  sig_qid = osMessageQueueNew(4, 1, NULL);

  osThreadNew(mainThread, NULL, &main_attributes);
  //  osThreadNew(threadThread, NULL, &thread_attributes);
}
