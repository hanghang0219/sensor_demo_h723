//
// Created by MSI-NB on 16/04/2024.
//

#include "cmsis_os2.h"
#include "lv_port_disp.h"
#include "ui.h"
#include "stm32h723xx.h"
#include "DEV_Config.h"
#include "lv_demo_widgets.h"
#include "Touch_Driver.h"
#include "lv_port_indev.h"
#include "LCD_1in69.h"
#include "GUI_Paint.h"


uint8_t mutex_lv_task, mutex_second = 0;
uint8_t one_second_flag, touch_flag = 0;
uint16_t display_count = 0;

Touch_1IN69_XY XY;

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


void vApplicationTickHook(void) {
  lv_tick_inc(1);  // let lvgl know the system time
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin & TP_INT_Pin)
    touch_flag = TOUCH_IRQ;
}


void TIM7_IRQHandler(void) {
  if (!mutex_second) {
    mutex_second = 1;
    one_second_flag = 1;
    TIM7->SR &= ~TIM_SR_UIF;
    mutex_second = 0;
  }
}

void initTimingClk() {

  RCC->APB1LENR |= RCC_APB1LENR_TIM7EN;
  TIM7->PSC = 27499;
  TIM7->ARR = 9999;   // 1S It setting
  TIM7->DIER |= TIM_DIER_UIE;
  TIM7->CR1 = TIM_CR1_CEN;
  NVIC_SetPriority(TIM7_IRQn, 7);
  NVIC_EnableIRQ(TIM7_IRQn);
}

void display_tp_init() {
  DEV_Module_Init();
  Touch_1IN69_init(2);
  LCD_1IN69_SetBackLight(1000);
  LCD_1IN69_Init(VERTICAL);
  LCD_1IN69_Clear(WHITE);
}


__NO_RETURN void mainThread(void* arg) {
  display_tp_init();
  lv_init();
  lv_port_disp_init();
  lv_port_indev_init();

  osDelay(100);

//  ui_init();
  lv_demo_widgets();
//  initTimingClk();
  while (1) {

    if (!mutex_lv_task) {
      mutex_lv_task = 1;
      lv_task_handler();
      mutex_lv_task = 0;
    }
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

