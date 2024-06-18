//
// Created by admin on 17/06/2024.
//

#include "delay_tim.h"
#include "stm32h723xx.h"
#include "tim.h"


void delay_us(uint8_t us) {
  uint16_t differ = 0xffff - us - 5;
  __HAL_TIM_SET_COUNTER(&htim7, differ);
  HAL_TIM_Base_Start(&htim7);

  while (differ < 0xffff - 5)
    differ = __HAL_TIM_GET_COUNTER(&htim7);

  HAL_TIM_Base_Stop(&htim7);
}
