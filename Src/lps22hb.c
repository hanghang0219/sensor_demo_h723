//
// Created by hanghang on 20/06/2024.
//

#include "lps22hb.h"
#include "i2c.h"


void masterToLps22hb(void) {
  uint8_t who_am_i = 0, ctrl_reg2 = 0;

  HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x0f, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);
  if (who_am_i == 0xB1) {
    /* 1.set the single measurement mode 2.make register add increase*/
    uint8_t one_shot = 1, if_add_inc = 1 << 4;
    uint8_t ctrl_reg2_new = one_shot | if_add_inc;
    HAL_I2C_Mem_Write(&hi2c1, 0xB9, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2_new, 1, 100);

    HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x11, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);

    uint8_t air_pressure[3];
    HAL_I2C_Mem_Read(&hi2c1, 0xB8, 0x28, I2C_MEMADD_SIZE_8BIT, air_pressure, 3, 100);
    volatile uint32_t tmp = air_pressure[2] << 16 | air_pressure[1] << 8 | air_pressure[0];
    if (tmp & 0x00800000)
      tmp |= 0xFF000000;
    float pressure = tmp / 4096.00;
  }
}
