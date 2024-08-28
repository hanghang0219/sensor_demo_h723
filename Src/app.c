

#include <SEGGER_RTT.h>
#include <gpio.h>
#include <lsm6dsow.h>

#include "cmsis_os2.h"
#include "stm32h723xx.h"

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_15) {
    SEGGER_RTT_printf(0, "%s Fifo now is full \n%s", RTT_CTRL_TEXT_BRIGHT_YELLOW, RTT_CTRL_RESET);
    lsm6dsowFifoStatus(&lsm6dsow);
    uint8_t signal = 1;
    osMessageQueuePut(sig_0_qid, &signal, 0, 0);
  }
}

__NO_RETURN void mainThread(void* arg) {

  lsm6dsowInit(&lsm6dsow, &hi2c2);
  lsm6dsowRegInit(&lsm6dsow);

  uint8_t sig = 0;
  osStatus_t st;
  while (1) {
    st = osMessageQueueGet(sig_0_qid, &sig, 0, 3600000);
    if (st == osOK) {
      lsm6dsowFifoData(&lsm6dsow);
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
  sig_0_qid = osMessageQueueNew(4, 1, NULL);

  SEGGER_RTT_printf(0, "%s welcome to fly  \n%s", RTT_CTRL_TEXT_BRIGHT_BLUE, RTT_CTRL_RESET);

  osThreadNew(mainThread, NULL, &main_attributes);
}
