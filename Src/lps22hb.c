//
// Created by hanghang on 20/06/2024.
//

#include <string.h>
#include <stdio.h>
#include "lps22hb.h"
#include "usart.h"


HAL_StatusTypeDef HAL_Printf(uint8_t* pData) {
  return HAL_UART_Transmit(&huart1, pData, strlen((char*) pData), 1000);
}

/* no interrupt */
//void example(void) {
//  uint8_t who_am_i = 0, ctrl_reg2 = 0;
//  char display_buf[20];
//
//
//  HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x0f, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
//  HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);
//
//  sprintf(display_buf, "ctrl_reg2_old:%d \r\n", ctrl_reg2);
//  HAL_Printf((uint8_t*) display_buf);
//  if (who_am_i == 0xB1) {
//    /* 1.set the single measurement mode 2.make register add increase*/
//    uint8_t one_shot = 1, if_add_inc = 1 << 4;
//    ctrl_reg2 = one_shot | if_add_inc;
//    HAL_I2C_Mem_Write(&hi2c1, 0xB9, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);
//
//    HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);
//    sprintf(display_buf, "ctrl_reg2_new:%d \r\n", ctrl_reg2);
//    HAL_Printf((uint8_t*) display_buf);
//    uint8_t air_pressure[3];
//    HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x28, I2C_MEMADD_SIZE_8BIT, air_pressure, 3, 100);
//    volatile uint32_t tmp = air_pressure[2] << 16 | air_pressure[1] << 8 | air_pressure[0];
//    if (tmp & 0x00800000)
//      tmp |= 0xFF000000;
//    float pressure = tmp / 4096.00;
//
//    sprintf(display_buf, "air pressure:%d \r\n", (int) pressure);
//    HAL_Printf((uint8_t*) display_buf);
//  }
//}

void lps22hbInit(Lps22hb* ins, I2C_HandleTypeDef* i2c) {
  ins->i2c = i2c;
  ins->err_st = LPS22HB_OK;
}

// read id register to get id
void readLps22hbIdReg(Lps22hb* ins) {
  ins->flag = LPS22HB_FLAG_ID;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read_IT(
          ins->i2c, LPS22HB_SAD_READ, LPS22HB_REGISTER_ADDR_ID,
          I2C_MEMADD_SIZE_8BIT,
          &ins->data_id, 1);
  ins->err_st = st == HAL_OK ? LPS22HB_OK : LPS22HB_READ_ERR;
}

// read pressure register to get pressure
void readLps22hbPressureReg(Lps22hb* ins) {
  ins->flag = LPS22HB_FLAG_DATA;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read_IT(
          ins->i2c, LPS22HB_SAD_READ, LPS22HB_REGISTER_ADDR_PRESS_OUT_XL,
          I2C_MEMADD_SIZE_8BIT,
          (uint8_t*) &ins->data_air_pressure, 3);
  ins->err_st = st == HAL_OK ? LPS22HB_OK : LPS22HB_READ_ERR;
}

// change register to measure pressure once
void writeLps22hbOnceReg(Lps22hb* ins) {
  uint8_t cmd = LPS22HB_ONCE;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write_IT(
          ins->i2c, LPS22HB_SAD_WRITE, LPS22HB_REGISTER_ADDR_CTRL_REG2,
          I2C_MEMADD_SIZE_8BIT,
          &cmd, 1);
  ins->err_st = st == HAL_OK ? LPS22HB_OK : LPS22HB_WRITE_ERR;
}

void lps22hbParser(Lps22hb* ins) {
  volatile uint32_t tmp = ins->data_air_pressure[2] << 16 | ins->data_air_pressure[1] << 8 | ins->data_air_pressure[0];
  if (tmp & 0x00800000)
    tmp |= 0xFF000000;
  ins->air_pressure = (float) (tmp / 4096.00);
  char data[20];
  sprintf(data, "air pressure:%d \r\n", (int) ins->air_pressure);
  HAL_Printf((uint8_t*) data);
  ins->flag = LPS22HB_FLAG_END;
}

