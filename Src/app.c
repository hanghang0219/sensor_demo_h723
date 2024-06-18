

#include "cmsis_os2.h"
#include "gxhtc3.h"
#include "delay_tim.h"


const osThreadAttr_t main_attributes = {
        .name = "main",
        .stack_size = 1024 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t thread_attributes = {
        .name = "main",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};


__NO_RETURN void mainThread(void* arg) {
  GXHTC3_INIT();
  al_get_gxth30_temp();
  while (1) {

    osDelay(1);
  }
}


__NO_RETURN void threadThread(void* arg) {

  while (1) {
  }
}


void MX_FREERTOS_Init(void) {

  osThreadNew(mainThread, NULL, &main_attributes);
//  osThreadNew(threadThread, NULL, &thread_attributes);
}

