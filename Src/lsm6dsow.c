//
// Created by hanghang on 24-8-11.
//

#include "lsm6dsow.h"
#include <cmsis_os2.h>
#include <stdbool.h>
#include <string.h>
#include "SEGGER_RTT.h"

bool lsm6dsowFifoInit(Lsm6dsow* ins);
void LSM6DSOWFifoClear(Lsm6dsow* ins);
bool lsm6dsowFifoBDR(Lsm6dsow* self, uint8_t gy_bdr, uint8_t xl_bdr);

Lsm6dsowReg LSM6DSOW_reg = {.read = LSM6DSOW_READ, .write = LSM6DSOW_WRITE};
Lsm6dsowData LSM6DSOW_data = {0};
Lsm6dsowFifo LSM6DSOW_fifo = {LSM6DSOW_FIFO_EMPTY};

void lsm6dsowInit(Lsm6dsow* ins, I2C_HandleTypeDef* i2c) {
  ins->i2c = i2c;
  ins->err_st = LSM6DSOW_OK;
  ins->reg = LSM6DSOW_reg;
  ins->data = LSM6DSOW_data;
  ins->fifo = LSM6DSOW_fifo;
  ins->update_flag = LSM6DSOW_FLAG_UNUPDATE;
}

void lsm6dsowInt1GpioItInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/* 1.identify id */
/**
  * @param dev_read_addr  LSM6DSOW_READ
  * @param mem_addr LSM6DSOW_ID_ADD
  */
bool lsm6dsowWhoIAm(Lsm6dsow* ins, uint8_t dev_read_addr, uint8_t mem_addr) {

  uint8_t who_am_i = 0;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1);
  if (st != HAL_OK || who_am_i != 0x6C) {
    SEGGER_RTT_printf(0, "%s who_i_am error :%d\n%s", RTT_CTRL_TEXT_BRIGHT_RED, st, RTT_CTRL_RESET);
    ins->err_st = LSM6DSOW_ID_ERR;
    return false;
  }
  SEGGER_RTT_printf(0, "%s i am lsm6dsow \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* 2.reset && bdu improve
 *  the accelerometer and the gyroscope in power-down mode after reset 7:1
 *  register add automatically increase  2:1
 *  bdu : output register are updated until MSB and LSB have been read  6:1
 *  128 + 4 + 64
 */
/**
  * @param dev_read_addr  LSM6DSOW_READ
  * @param dev_write_addr LSM6DSOW_WRITE
  * @param mem_addr LSM6DSOW_CTRL3_C_ADD
  * @param mem_value LSM6DSOW_CTRL3_C_VALUE
  */
bool lsm6dsowResetDevice(Lsm6dsow* ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                         uint8_t mem_value) {
  uint8_t reset_sw_value = 0;
  uint8_t reset_boot_value[1] = {mem_value};

  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_15);
  reset_sw_value = 4;
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, LSM6DSOW_FIFO_INT1_CTRL, I2C_MEMADD_SIZE_8BIT, &reset_sw_value, 1, 1);

  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, reset_boot_value, 1, 1);
  osDelay(30);  // must wait reset

  st |= HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &reset_sw_value, 1, 1);
  reset_sw_value |= 1;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &reset_sw_value, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DSOW_RESET_ERR;
    SEGGER_RTT_printf(0, "%s reset error :%d %s\n", RTT_CTRL_TEXT_BRIGHT_WHITE, st, RTT_CTRL_RESET);
    return false;
  }

  SEGGER_RTT_printf(0, "%s reset suc  \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* 3.acceleration ：833hz 4g*/
/**
  * @param dev_read_addr  LSM6DSOW_READ
  * @param dev_write_addr LSM6DSOW_WRITE
  * @param mem_addr LSM6DSOW_CTRL1_XL
  * @param mem_value1 LSM6DSOW_ACC_RATE_833HZ
  * @param mem_value2 LSM6DSOW_ACC_FSXL_4G
  */
bool lsm6dsowAccParam(Lsm6dsow* ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                      uint8_t mem_value1, uint8_t mem_value2) {
  uint8_t acceleration = 0;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &acceleration, 1, 1);

  acceleration |= mem_value1;
  acceleration |= mem_value2;

  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &acceleration, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DSOW_ACC_PARAM_ERR;
    SEGGER_RTT_printf(0, "%s acc param error :%d\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, st, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s acc param suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* 4.gyroscope 833hz 2000*/
/**
  * @param dev_read_addr  LSM6DSOW_READ
  * @param dev_write_addr LSM6DSOW_WRITE
  * @param mem_addr LSM6DSOW_CTRL2_G
  * @param mem_value1 LSM6DSOW_GYR_RATE_833HZ
  * @param mem_value2 LSM6DSOW_GYR_FSG_2000
  */
bool lsm6dsowGyrParam(Lsm6dsow* ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                      uint8_t mem_value1, uint8_t mem_value2) {
  uint8_t gyroscope = 0;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &gyroscope, 1, 1);
  gyroscope |= mem_value1;
  gyroscope |= mem_value2;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &gyroscope, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DSOW_GYR_PARAM_ERR;
    SEGGER_RTT_printf(0, "%s gyr param error :%d\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, st, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s gyr param suc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

bool lsm6dsowRegInit(Lsm6dsow* ins) {

  int result = lsm6dsowWhoIAm(ins, LSM6DSOW_READ, LSM6DSOW_ID_ADD);
  CHECK_RESULT(result);

  result = lsm6dsowResetDevice(ins, LSM6DSOW_READ, LSM6DSOW_WRITE, LSM6DSOW_CTRL3_C_ADD, LSM6DSOW_CTRL3_C_VALUE);
  CHECK_RESULT(result);

  lsm6dsowInt1GpioItInit();

  result = lsm6dsowAccParam(ins, LSM6DSOW_READ, LSM6DSOW_WRITE, LSM6DSOW_CTRL1_XL, LSM6DSOW_ACC_RATE_104HZ,
                            LSM6DSOW_ACC_FSXL_4G);
  CHECK_RESULT(result);

  result = lsm6dsowGyrParam(ins, LSM6DSOW_READ, LSM6DSOW_WRITE, LSM6DSOW_CTRL2_G, LSM6DSOW_GYR_RATE_104HZ,
                            LSM6DSOW_GYR_FSG_2000);
  CHECK_RESULT(result);

  result = lsm6dsowFifoInit(ins);
  CHECK_RESULT(result);

  return true;
}

// FIFO init
/**
  * @param dev_read_addr  LSM6DSOW_READ
  * @param dev_write_addr LSM6DSOW_WRITE
  * @param mem_addr1 LSM6DSOW_FIFO_CRTL1
  * @param mem_addr2 LSM6DSOW_FIFO_CRTL2
  * @param water_mark
  */
bool lsm6dsowFifoWatermarkSet(Lsm6dsow* ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr1,
                              uint8_t mem_addr2, uint16_t water_mark) {

  const uint8_t low_reg_value = (water_mark & 0x00ff);
  const uint8_t high_reg_value = ((water_mark & (0x0001 << 8)) >> 8);

  uint8_t buf[1] = {low_reg_value};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr1, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  st |= HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr2, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  // FIFO_CTRL2 : bit 7 STOP_ON_WTM ->  1
  // buf[0] |= high_reg_value | 1 << 7;
  buf[0] |= high_reg_value;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr2, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DSOW_FIFO_SET_WATERMARK_ERR;
    SEGGER_RTT_printf(0, "%s Fifo wtm unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo wtm suc : %d\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, water_mark, RTT_CTRL_RESET);
  return true;
}

#define LSM6DSOW_FIFO_CRTL3 0x09

#define BDR_GY_RATE_12HZ5 0x01
#define BDR_GY_RATE_26HZ 0x02
#define BDR_GY_RATE_52HZ 0x03
#define BDR_GY_RATE_104HZ 0x04
#define BDR_GY_RATE_208HZ 0x05
#define BDR_GY_RATE_417HZ 0x06
#define BDR_GY_RATE_833HZ 0x07
#define BDR_GY_RATE_1667HZ 0x08
#define BDR_GY_RATE_3333HZ 0x09
#define BDR_GY_RATE_6667HZ 0x0A
#define BDR_GY_RATE_6HZ5 0x0B

#define BDR_XL_RATE_12HZ5 0x01
#define BDR_XL_RATE_26HZ 0x02
#define BDR_XL_RATE_52HZ 0x03
#define BDR_XL_RATE_104HZ 0x04
#define BDR_XL_RATE_208HZ 0x05
#define BDR_XL_RATE_417HZ 0x06
#define BDR_XL_RATE_833HZ 0x07
#define BDR_XL_RATE_1667HZ 0x08
#define BDR_XL_RATE_3333HZ 0x09
#define BDR_XL_RATE_6667HZ 0x0A
#define BDR_XL_RATE_6HZ5 0x0B

bool lsm6dsowFifoBDR(Lsm6dsow* self, uint8_t gy_bdr, uint8_t xl_bdr) {
  uint8_t buf[1] = {0};
  buf[0] = gy_bdr << 4 | xl_bdr;

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(self->i2c, self->reg.write, LSM6DSOW_FIFO_CRTL3, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_BDR_ERR;
    SEGGER_RTT_printf(0, "%s Fifo BDR unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo BDR suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* Set fifo mode and timestamp start */
#define LSM6DSOW_FIFO_CRTL4 0x0A

bool lsm6dsowFifoModeSet(Lsm6dsow* self, uint8_t mode) {
  uint8_t buf[1] = {0};

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(self->i2c, self->reg.read, LSM6DSOW_FIFO_CRTL4, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  // timestamp decimation [7:6] : 01
  buf[0] |= mode | 1 << 6;
  st |= HAL_I2C_Mem_Write(self->i2c, self->reg.write, LSM6DSOW_FIFO_CRTL4, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_MODE_ERR;
    SEGGER_RTT_printf(0, "%s Fifo mode set unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo mode set suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* Set IT touch by low power */
#define LSM6DSOW_FIFO_COUNTER_BDR_REG1 0x0B

bool lsm6dsowFifoINT1PulseSet(Lsm6dsow* self) {
  uint8_t buf[1] = {0};

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(self->i2c, self->reg.read, LSM6DSOW_FIFO_COUNTER_BDR_REG1, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  buf[0] |= 0x80;
  st |= HAL_I2C_Mem_Write(self->i2c, self->reg.write, LSM6DSOW_FIFO_COUNTER_BDR_REG1, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_INT1_PULSED_ERR;
    SEGGER_RTT_printf(0, "%s Fifo INT1 pulse set unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo INT1 pulse set suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* Set INT1 by wtm full */
#define LSM6DSOW_FIFO_INT1_CTRL 0x0D

#define INT1_FIFO_TH 0x08

bool lsm6dsowFifoINT1Set(Lsm6dsow* self, uint8_t mode) {
  uint8_t buf[1] = {0};

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(self->i2c, self->reg.read, LSM6DSOW_FIFO_INT1_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  buf[0] |= mode;
  st |= HAL_I2C_Mem_Write(self->i2c, self->reg.write, LSM6DSOW_FIFO_INT1_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_INT1_ERR;
    SEGGER_RTT_printf(0, "%s Fifo INT1 set unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo INT1 set suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* Query wtm flag is full or not */
#define LSM6DSOW_FIFO_STATUS2 0x3B

bool lsm6dsowFifoStatus(Lsm6dsow* self) {
  uint8_t buf[1] = {0};

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(self->i2c, self->reg.read, LSM6DSOW_FIFO_STATUS2, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_ACCESS_REG_FIFO_STATUS2_ERR;
    SEGGER_RTT_printf(0, "%s Fifo status2  unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo status2 suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  SEGGER_RTT_printf(0, "%s Fifo status2 :%d \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, buf[0], RTT_CTRL_RESET);
  return true;
}

#define LSM6DSOW_FIFO_TIMESTAMP 0X19

bool lsm6dsowFifoTimestamp(Lsm6dsow* self) {
  uint8_t buf[1] = {0};
  buf[0] = 1 << 5;

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(self->i2c, self->reg.read, LSM6DSOW_FIFO_TIMESTAMP, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_TIMER_ERR;
    SEGGER_RTT_printf(0, "%s Fifo Timestamp  unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo Timestamp suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

#define LSM6DSOW_FIFO_DATA_OUT_TAG 0x78
#define LSM6DSOW_FIFO_DATA_OUT_X_L 0x79
#define LSM6DSOW_FIFO_DATA_OUT_X_H 0x7A
#define LSM6DSOW_FIFO_DATA_OUT_Y_L 0x7B
#define LSM6DSOW_FIFO_DATA_OUT_Y_H 0x7C
#define LSM6DSOW_FIFO_DATA_OUT_Z_L 0x7D
#define LSM6DSOW_FIFO_DATA_OUT_Z_H 0x7E
#define LSM6DSOW_FIFO_TIMESTAMP0 0x40
#define LSM6DSOW_FIFO_TIMESTAMP1 0x41
#define LSM6DSOW_FIFO_TIMESTAMP2 0x42
#define LSM6DSOW_FIFO_TIMESTAMP3 0x43

#define LSM6DSOW_GY_NC 0x01
#define LSM6DSOW_XL_NC 0x02
#define LSM6DSOW_TIMESTAMP_NC 0x04

bool lsm6dsowFifoData(Lsm6dsow* self) {
  uint8_t status_buf[1] = {0};
  uint8_t data_buf[6] = {0};
  uint16_t wtm = LSM6DSOW_FIFO_WATERMARK;

  uint8_t gy_ready_flag = 0;
  uint8_t ac_ready_flag = 0;
  uint8_t timestamp_ready_flag = 0;
  uint8_t data_ready_total_num = 0;
  HAL_StatusTypeDef st = HAL_OK;
  while (wtm--) {
    st |=
        HAL_I2C_Mem_Read(self->i2c, self->reg.read, LSM6DSOW_FIFO_DATA_OUT_TAG, I2C_MEMADD_SIZE_8BIT, status_buf, 1, 1);
    st |=
        HAL_I2C_Mem_Read(self->i2c, self->reg.read, LSM6DSOW_FIFO_DATA_OUT_X_L, I2C_MEMADD_SIZE_8BIT, data_buf, 6, 10);
    switch (status_buf[0] >> 3) {
      case LSM6DSOW_GY_NC:
        gy_ready_flag = 1;
        self->data.gyr_x = (int16_t)(data_buf[1] << 8 | data_buf[0]) * 70.00f;
        self->data.gyr_y = (int16_t)(data_buf[3] << 8 | data_buf[2]) * 70.00f;
        self->data.gyr_z = (int16_t)(data_buf[5] << 8 | data_buf[4]) * 70.00f;

        break;
      case LSM6DSOW_XL_NC:
        ac_ready_flag = 1;
        self->data.acc_x = (int16_t)(data_buf[1] << 8 | data_buf[0]) * 0.122f;
        self->data.acc_y = (int16_t)(data_buf[3] << 8 | data_buf[2]) * 0.122f;
        self->data.acc_z = (int16_t)(data_buf[5] << 8 | data_buf[4]) * 0.122f;

        break;
      case LSM6DSOW_TIMESTAMP_NC:
        timestamp_ready_flag = 1;
        self->data.timestamp = (uint32_t)(data_buf[0] | data_buf[1] << 8 | data_buf[2] << 16 | data_buf[3] << 24);

        break;
      default:
        SEGGER_RTT_printf(0, "%s Fifo data out :%d\n%s", RTT_CTRL_TEXT_BRIGHT_YELLOW, status_buf[0] >> 3,
                          RTT_CTRL_RESET);
        break;
    }
    st |=
        HAL_I2C_Mem_Read(self->i2c, self->reg.read, LSM6DSOW_FIFO_DATA_OUT_TAG, I2C_MEMADD_SIZE_8BIT, status_buf, 1, 1);
    if (st != HAL_OK) {
      self->err_st = LSM6DSOW_ACCESS_REG_FIFO_DATA_OUT_ERR;
      SEGGER_RTT_printf(0, "%s Fifo data out unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
      return false;
    }
    if (timestamp_ready_flag == 1 && gy_ready_flag == 1 && ac_ready_flag == 1) {

      data_ready_total_num++;
      SEGGER_RTT_printf(0, "%s Fifo data out Gy:%d,%d,%d\n%s", RTT_CTRL_TEXT_BRIGHT_MAGENTA, (int)self->data.gyr_x,
                        (int)self->data.gyr_y, (int)self->data.gyr_z, RTT_CTRL_RESET);
      SEGGER_RTT_printf(0, "%s Fifo data out Ac:%d,%d,%d\n%s", RTT_CTRL_TEXT_BRIGHT_GREEN, (int)self->data.acc_x,
                        (int)self->data.acc_y, (int)self->data.acc_z, RTT_CTRL_RESET);
      SEGGER_RTT_printf(0, "%s Fifo timestamp :%d\n%s", RTT_CTRL_TEXT_BRIGHT_YELLOW, self->data.timestamp,
                        RTT_CTRL_RESET);
      SEGGER_RTT_printf(0, "%s Fifo data out num :%d\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, data_ready_total_num,
                        RTT_CTRL_RESET);

      gy_ready_flag = 0;
      ac_ready_flag = 0;
      timestamp_ready_flag = 0;
    }
    memset(data_buf, 0, sizeof(data_buf));
  }
  return true;
}

/*------------------------------------------------- fifo init ----------------------------------------------------------------*/
bool lsm6dsowFifoInit(Lsm6dsow* ins) {

  int result = lsm6dsowFifoWatermarkSet(ins, LSM6DSOW_READ, LSM6DSOW_WRITE, LSM6DSOW_FIFO_CRTL1, LSM6DSOW_FIFO_CRTL2,
                                        LSM6DSOW_FIFO_WATERMARK);
  CHECK_RESULT(result);

  result = lsm6dsowFifoBDR(ins, BDR_GY_RATE_52HZ, BDR_XL_RATE_52HZ);
  CHECK_RESULT(result);

  result = lsm6dsowFifoModeSet(ins, LSM6DSOW_STREAM_MODE);
  CHECK_RESULT(result);

  result = lsm6dsowFifoINT1PulseSet(ins);
  CHECK_RESULT(result);

  result = lsm6dsowFifoINT1Set(ins, INT1_FIFO_TH);
  CHECK_RESULT(result);

  result = lsm6dsowFifoStatus(ins);
  CHECK_RESULT(result);

  result = lsm6dsowFifoTimestamp(ins);
  CHECK_RESULT(result);

  return true;
}

