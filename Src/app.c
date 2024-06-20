

#include "cmsis_os2.h"
#include "lps22hb.h"


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
  osDelay(100);
  masterToLps22hb();
  while (1) {

    osDelay(1);
  }
}


__NO_RETURN void threadThread(void* arg) {
  osDelay(3000);
  while (1) {
  }
}


void MX_FREERTOS_Init(void) {

  osThreadNew(mainThread, NULL, &main_attributes);
//  osThreadNew(threadThread, NULL, &thread_attributes);
}

