//
// Created by hanghang on 20/06/2024.
//

#include <string.h>
#include <stdio.h>
#include "lps22hb.h"
#include "i2c.h"
#include "usart.h"
#include "cmsis_os2.h"


HAL_StatusTypeDef HAL_Printf(uint8_t* pData) {
  return HAL_UART_Transmit(&huart1, pData, strlen((char*) pData), 1000);
}


void masterToLps22hb(void) {
  uint8_t who_am_i = 0, ctrl_reg2 = 0;
  char display_buf[20];

  HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x0f, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);

  sprintf(display_buf, "ctrl_reg2_old:%d \r\n", ctrl_reg2);
  HAL_Printf((uint8_t*) display_buf);
  if (who_am_i == 0xB1) {
    /* 1.set the single measurement mode 2.make register add increase*/
    uint8_t one_shot = 1, if_add_inc = 1 << 4;
    ctrl_reg2 = one_shot | if_add_inc;
    HAL_I2C_Mem_Write(&hi2c1, 0xB9, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);

    HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);
    sprintf(display_buf, "ctrl_reg2_new:%d \r\n", ctrl_reg2);
    HAL_Printf((uint8_t*) display_buf);
    uint8_t air_pressure[3];
    HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x28, I2C_MEMADD_SIZE_8BIT, air_pressure, 3, 100);
    volatile uint32_t tmp = air_pressure[2] << 16 | air_pressure[1] << 8 | air_pressure[0];
    if (tmp & 0x00800000)
      tmp |= 0xFF000000;
    float pressure = tmp / 4096.00;

    sprintf(display_buf, "air pressure:%d \r\n", (int) pressure);
    HAL_Printf((uint8_t*) display_buf);
  }

}
