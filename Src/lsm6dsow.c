//
// Created by hanghang on 24-8-11.
//

#include "lsm6dsow.h"
#include <cmsis_os2.h>
#include <lsm6dsow_motionFX.h>
#include <stdbool.h>
#include <string.h>
#include "SEGGER_RTT.h"

bool lsm6dsowFifoInit(Lsm6dsow* ins);

Lsm6dsowRegCmd lsm6dsow_reg = {.read = LSM6DSOW_READ, .write = LSM6DSOW_WRITE};
Lsm6dsowRegData lsm6dsow_data = {0};
Lsm6dsowOutput lsm6dsow_output = {0};
Lsm6dsowCallback lsm6dsow_callback = {LSM6DSOW_FIFO_WATERMARK, 0, 0, 0};

void lsm6dsowInit(Lsm6dsow* ins, I2C_HandleTypeDef* i2c) {
  ins->i2c = i2c;
  ins->err_st = LSM6DSOW_OK;
  ins->reg_cmd = &lsm6dsow_reg;
  ins->reg_data = &lsm6dsow_data;
  ins->output = &lsm6dsow_output;
  ins->callback = &lsm6dsow_callback;
}

void lsm6dsowInt1GPIOInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/* REGISTER ID */
#define LSM6DSOW_ID_ADD_REG 0x0F

/* REGISTER VALUE*/
#define LSM6DSOW_ID_REG_VALUE_READ 0x6C

bool lsm6dsowWhoIAm(Lsm6dsow* ins) {
  uint8_t who_am_i = 0;

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, ins->reg_cmd->read, LSM6DSOW_ID_ADD_REG, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1);
  if (st != HAL_OK || who_am_i != LSM6DSOW_ID_REG_VALUE_READ) {
    SEGGER_RTT_printf(0, "%s who_i_am error :%d\n%s", RTT_CTRL_TEXT_BRIGHT_RED, st, RTT_CTRL_RESET);
    ins->err_st = LSM6DSOW_ID_ERR;
    return false;
  }
  SEGGER_RTT_printf(0, "%s i am lsm6dsow \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/*
 *  Reset && bdu improve
 *  the accelerometer and the gyroscope in power-down mode after reset 7:1
 *  register add automatically increase  2:1
 *  bdu : output register are updated until MSB and LSB have been read  6:1
 *  128 + 4 + 64
 */

/* REGISTER ID */
#define LSM6DSOW_CTRL3_REG 0x12

/* REGISTER VALUE*/
#define LSM6DSOW_CTRL3_REG_VALUE 0XC4

bool lsm6dsowResetDevice(Lsm6dsow* ins) {
  uint8_t reset_sw_value = 0;
  uint8_t reset_boot_value[1] = {LSM6DSOW_CTRL3_REG_VALUE};

  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_15);

  // INT1 reg flag bit
  reset_sw_value = 1 << 2;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(ins->i2c, ins->reg_cmd->write, LSM6DSOW_FIFO_INT1_CTRL_REG,
                                           I2C_MEMADD_SIZE_8BIT, &reset_sw_value, 1, 1);

  st |= HAL_I2C_Mem_Write(ins->i2c, ins->reg_cmd->write, LSM6DSOW_CTRL3_REG, I2C_MEMADD_SIZE_8BIT, reset_boot_value, 1,
                          1);
  osDelay(30);  // must wait reset

  st |= HAL_I2C_Mem_Read(ins->i2c, ins->reg_cmd->read, LSM6DSOW_CTRL3_REG, I2C_MEMADD_SIZE_8BIT, &reset_sw_value, 1, 1);

  // software reset flag bit
  reset_sw_value |= 1 << 0;
  st |=
      HAL_I2C_Mem_Write(ins->i2c, ins->reg_cmd->write, LSM6DSOW_CTRL3_REG, I2C_MEMADD_SIZE_8BIT, &reset_sw_value, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DSOW_RESET_ERR;
    SEGGER_RTT_printf(0, "%s reset error :%d %s\n", RTT_CTRL_TEXT_BRIGHT_WHITE, st, RTT_CTRL_RESET);
    return false;
  }

  SEGGER_RTT_printf(0, "%s reset suc  \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
#define LSM6DSOW_CTRL1_XL_REG 0x10

bool lsm6dsowAccParam(Lsm6dsow* ins, uint8_t mem_value1, uint8_t mem_value2) {
  uint8_t acceleration = 0;
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, ins->reg_cmd->read, LSM6DSOW_CTRL1_XL_REG, I2C_MEMADD_SIZE_8BIT, &acceleration, 1, 1);

  acceleration |= mem_value1;
  acceleration |= mem_value2;

  st |= HAL_I2C_Mem_Write(ins->i2c, ins->reg_cmd->write, LSM6DSOW_CTRL1_XL_REG, I2C_MEMADD_SIZE_8BIT, &acceleration, 1,
                          1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DSOW_ACC_PARAM_ERR;
    SEGGER_RTT_printf(0, "%s acc param error :%d\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, st, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s acc param suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
#define LSM6DSOW_CTRL2_G_REG 0x11

bool lsm6dsowGyrParam(Lsm6dsow* ins, uint8_t mem_value1, uint8_t mem_value2) {
  uint8_t gyroscope = 0;
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, ins->reg_cmd->read, LSM6DSOW_CTRL2_G_REG, I2C_MEMADD_SIZE_8BIT, &gyroscope, 1, 1);
  gyroscope |= mem_value1;
  gyroscope |= mem_value2;
  st |= HAL_I2C_Mem_Write(ins->i2c, ins->reg_cmd->write, LSM6DSOW_CTRL2_G_REG, I2C_MEMADD_SIZE_8BIT, &gyroscope, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DSOW_GYR_PARAM_ERR;
    SEGGER_RTT_printf(0, "%s gyr param error :%d\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, st, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s gyr param suc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

bool lsm6dsowRegInit(Lsm6dsow* ins) {

  int result = lsm6dsowWhoIAm(ins);
  CHECK_RESULT(result);

  result = lsm6dsowResetDevice(ins);
  CHECK_RESULT(result);

  lsm6dsowInt1GPIOInit();

  result = lsm6dsowAccParam(ins, LSM6DSOW_ACC_RATE_416HZ, LSM6DSOW_ACC_FSXL_4G);
  CHECK_RESULT(result);

  result = lsm6dsowGyrParam(ins, LSM6DSOW_GYR_RATE_416HZ, LSM6DSOW_GYR_FSG_2000);
  CHECK_RESULT(result);

  result = lsm6dsowFifoInit(ins);
  CHECK_RESULT(result);

  return true;
}

/* REGISTER ID */
#define LSM6DSOW_FIFO_CRTL1_REG 0x07
#define LSM6DSOW_FIFO_CRTL2_REG 0x08

bool lsm6dsowFifoWatermarkSet(Lsm6dsow* ins, uint16_t water_mark) {

  const uint8_t low_reg_value = water_mark & 0x00ff;
  const uint8_t high_reg_value = (water_mark & 0x0001 << 8) >> 8;

  uint8_t buf[1] = {low_reg_value};
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(ins->i2c, ins->reg_cmd->write, LSM6DSOW_FIFO_CRTL1_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  st |= HAL_I2C_Mem_Read(ins->i2c, ins->reg_cmd->read, LSM6DSOW_FIFO_CRTL2_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  // FIFO_CTRL2 : bit 7 STOP_ON_WTM ->  1
  // buf[0] |= high_reg_value | 1 << 7;
  buf[0] |= high_reg_value;
  st |= HAL_I2C_Mem_Write(ins->i2c, ins->reg_cmd->write, LSM6DSOW_FIFO_CRTL2_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DSOW_FIFO_SET_WATERMARK_ERR;
    SEGGER_RTT_printf(0, "%s Fifo wtm unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo wtm suc : %d\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, water_mark, RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
#define LSM6DSOW_FIFO_CRTL3_REG 0x09

/* REGISTER VALUE*/
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
      HAL_I2C_Mem_Write(self->i2c, self->reg_cmd->write, LSM6DSOW_FIFO_CRTL3_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_BDR_ERR;
    SEGGER_RTT_printf(0, "%s Fifo BDR unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo BDR suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
/* Set fifo mode and timestamp start */
#define LSM6DSOW_FIFO_CRTL4_RGE 0x0A

bool lsm6dsowFifoModeSet(Lsm6dsow* self, uint8_t mode) {
  uint8_t buf[1] = {0};

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_CRTL4_RGE, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  // timestamp decimation [7:6] : 01
  buf[0] |= mode | 1 << 6;
  st |= HAL_I2C_Mem_Write(self->i2c, self->reg_cmd->write, LSM6DSOW_FIFO_CRTL4_RGE, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_MODE_ERR;
    SEGGER_RTT_printf(0, "%s Fifo mode set unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo mode set suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
#define LSM6DSOW_FIFO_COUNTER_BDR_REG1 0x0B

/* REGISTER VALUE*/
#define LSM6DSOW_FIFO_COUNTER_BDR_REG1_VALUE 0x80

/* Set IT touch by low power */
bool lsm6dsowFifoINT1PulseSet(Lsm6dsow* self) {
  uint8_t buf[1] = {0};

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_COUNTER_BDR_REG1, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  buf[0] |= LSM6DSOW_FIFO_COUNTER_BDR_REG1_VALUE;
  st |= HAL_I2C_Mem_Write(self->i2c, self->reg_cmd->write, LSM6DSOW_FIFO_COUNTER_BDR_REG1, I2C_MEMADD_SIZE_8BIT, buf, 1,
                          1);

  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_INT1_PULSED_ERR;
    SEGGER_RTT_printf(0, "%s Fifo INT1 pulse set unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo INT1 pulse set suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
#define LSM6DSOW_FIFO_INT1_CTRL_REG 0x0D

/* REGISTER VALUE*/
#define INT1_FIFO_TH_WRITE 0x08

/* Set INT1 by wtm full */
bool lsm6dsowFifoINT1Set(Lsm6dsow* self, uint8_t mode) {
  uint8_t buf[1] = {0};

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_INT1_CTRL_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  buf[0] |= mode;
  st |=
      HAL_I2C_Mem_Write(self->i2c, self->reg_cmd->write, LSM6DSOW_FIFO_INT1_CTRL_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_INT1_ERR;
    SEGGER_RTT_printf(0, "%s Fifo INT1 set unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo INT1 set suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
/* Query wtm flag is full or not */
#define LSM6DSOW_FIFO_STATUS2_REG 0x3B

bool lsm6dsowFifoStatus(Lsm6dsow* self) {
  uint8_t buf[1] = {0};

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_STATUS2_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_ACCESS_REG_FIFO_STATUS2_ERR;
    SEGGER_RTT_printf(0, "%s Fifo status2  unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo status2 suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  SEGGER_RTT_printf(0, "%s Fifo status2 :%d \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, buf[0], RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
#define LSM6DSOW_FIFO_TIMESTAMP_REG 0X19

/* REGISTER VALUE*/
#define LSM6DSOW_FIFO_TIMESTAMP_REG_VALUE_WRITE 1 << 5

bool lsm6dsowFifoTimestamp(Lsm6dsow* self) {
  uint8_t buf[1] = {0};
  buf[0] = LSM6DSOW_FIFO_TIMESTAMP_REG_VALUE_WRITE;

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_TIMESTAMP_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_FIFO_SET_TIMER_REG_ERR;
    SEGGER_RTT_printf(0, "%s Fifo Timestamp unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  SEGGER_RTT_printf(0, "%s Fifo Timestamp suc \n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
  return true;
}

/* REGISTER ID */
#define LSM6DSOW_FIFO_DATA_OUT_TAG_REG 0x78
#define LSM6DSOW_FIFO_DATA_OUT_X_L_REG 0x79
#define LSM6DSOW_FIFO_DATA_OUT_X_H_REG 0x7A
#define LSM6DSOW_FIFO_DATA_OUT_Y_L_REG 0x7B
#define LSM6DSOW_FIFO_DATA_OUT_Y_H_REG 0x7C
#define LSM6DSOW_FIFO_DATA_OUT_Z_L_REG 0x7D
#define LSM6DSOW_FIFO_DATA_OUT_Z_H_REG 0x7E
#define LSM6DSOW_FIFO_TIMESTAMP0_REG 0x40
#define LSM6DSOW_FIFO_TIMESTAMP1_REG 0x41
#define LSM6DSOW_FIFO_TIMESTAMP2_REG 0x42
#define LSM6DSOW_FIFO_TIMESTAMP3_REG 0x43

/* REGISTER VALUE*/
#define LSM6DSOW_GY_NC_REG_VALUE_READ 0x01
#define LSM6DSOW_XL_NC_REG_VALUE_READ 0x02
#define LSM6DSOW_TIMESTAMP_NC_REG_VALUE_READ 0x04

bool lsm6dsowFifoData(Lsm6dsow* self) {
  uint8_t status_buf[1] = {0};
  char data_buf[6] = {0};
  uint16_t wtm = LSM6DSOW_FIFO_WATERMARK;

  uint8_t gy_ready_flag = 0;
  uint8_t ac_ready_flag = 0;
  uint8_t timestamp_ready_flag = 0;

  HAL_StatusTypeDef st = HAL_OK;
  while (wtm > LSM6DSOW_FIFO_WATERMARK / 2) {
    st |= HAL_I2C_Mem_Read(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_DATA_OUT_TAG_REG, I2C_MEMADD_SIZE_8BIT,
                           status_buf, 1, 1);
    st |= HAL_I2C_Mem_Read(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_DATA_OUT_X_L_REG, I2C_MEMADD_SIZE_8BIT,
                           data_buf, 6, 10);

    switch (status_buf[0] >> 3) {
      case LSM6DSOW_GY_NC_REG_VALUE_READ:
        gy_ready_flag = 1;
        self->reg_data->gyr_x = (float)(int16_t)(data_buf[1] << 8 | data_buf[0]) * 70.00f;
        self->reg_data->gyr_y = (float)(int16_t)(data_buf[3] << 8 | data_buf[2]) * 70.00f;
        self->reg_data->gyr_z = (float)(int16_t)(data_buf[5] << 8 | data_buf[4]) * 70.00f;

        break;
      case LSM6DSOW_XL_NC_REG_VALUE_READ:
        ac_ready_flag = 1;
        self->reg_data->acc_x = (float)(int16_t)(data_buf[1] << 8 | data_buf[0]) * 0.122f;
        self->reg_data->acc_y = (float)(int16_t)(data_buf[3] << 8 | data_buf[2]) * 0.122f;
        self->reg_data->acc_z = (float)(int16_t)(data_buf[5] << 8 | data_buf[4]) * 0.122f;

        break;
      case LSM6DSOW_TIMESTAMP_NC_REG_VALUE_READ:

        timestamp_ready_flag = 1;
        self->reg_data->timestamp_2 =
            (uint32_t)(data_buf[0] | data_buf[1] << 8 | data_buf[2] << 16 | data_buf[3] << 24);

        break;
      default:
        SEGGER_RTT_printf(0, "%s Fifo data out :%d\n%s", RTT_CTRL_TEXT_BRIGHT_YELLOW, status_buf[0] >> 3,
                          RTT_CTRL_RESET);
        break;
    }

    st |= HAL_I2C_Mem_Read(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_DATA_OUT_TAG_REG, I2C_MEMADD_SIZE_8BIT,
                           status_buf, 1, 1);
    if (st != HAL_OK) {
      self->err_st = LSM6DSOW_ACCESS_REG_FIFO_DATA_OUT_ERR;
      SEGGER_RTT_printf(0, "%s Fifo data out unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
      return false;
    }
    if (timestamp_ready_flag == 1 && gy_ready_flag == 1 && ac_ready_flag == 1) {

      // SEGGER_RTT_printf(0, "%s Fifo data out Gy:%d,%d,%d\n%s", RTT_CTRL_TEXT_BRIGHT_MAGENTA, (int)self->data.gyr_x,
      //                   (int)self->data.gyr_y, (int)self->data.gyr_z, RTT_CTRL_RESET);

      // SEGGER_RTT_printf(0, "%s Fifo data out Ac:%d,%d,%d\n%s", RTT_CTRL_TEXT_BRIGHT_GREEN, (int)self->data.acc_x,
      //                   (int)self->data.acc_y, (int)self->data.acc_z, RTT_CTRL_RESET);

      // SEGGER_RTT_printf(0, "%s Fifo timestamp1 :%d  timestamp2:%d \n%s", RTT_CTRL_TEXT_BRIGHT_YELLOW,
      //                  self->reg_data->timestamp_1,self->reg_data.timestamp_2, RTT_CTRL_RESET);
      lsm6ds3trMotionFxDetermin(self);
      self->reg_data->timestamp_1 = self->reg_data->timestamp_2;

      gy_ready_flag = 0;
      ac_ready_flag = 0;
      timestamp_ready_flag = 0;
    }
    memset(data_buf, 0, sizeof(data_buf));
    wtm--;
  }
  return true;
}

#define DATA_OUT_TYPE_FLAG 0x00
#define GYR_OUT_TYPE_FLAG 0x01
#define ACC_OUT_TYPE_FLAG 0x02
#define TIMESTAMP_OUT_TYPE_FLAG 0x03

bool lsm6dsowI2cReadCallback(Lsm6dsow* self) {
  static uint8_t gy_ready_flag = 0;
  static uint8_t ac_ready_flag = 0;
  static uint8_t timestamp_ready_flag = 0;

  HAL_StatusTypeDef st = HAL_OK;
  switch (self->callback->rx_flag) {
    case DATA_OUT_TYPE_FLAG:
      st = HAL_I2C_Mem_Read_IT(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_DATA_OUT_X_L_REG, I2C_MEMADD_SIZE_8BIT,
                               self->callback->fifo_data_buf, 6);

      switch (self->callback->status_buf[0] >> 3) {
        case LSM6DSOW_GY_NC_REG_VALUE_READ:
          self->callback->rx_flag = GYR_OUT_TYPE_FLAG;
          break;
        case LSM6DSOW_XL_NC_REG_VALUE_READ:
          self->callback->rx_flag = ACC_OUT_TYPE_FLAG;
          break;
        case LSM6DSOW_TIMESTAMP_NC_REG_VALUE_READ:
          self->callback->rx_flag = TIMESTAMP_OUT_TYPE_FLAG;
          break;
        default:
          break;
      }

      if (st != HAL_OK) {
        self->err_st = LSM6DSOW_ACCESS_REG_FIFO_DATA_OUT_ERR;
        SEGGER_RTT_printf(0, "%s Fifo data out unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
        return false;
      }
      return true;

    case GYR_OUT_TYPE_FLAG:
      gy_ready_flag = 1;

      self->reg_data->gyr_x =
          (float)(int16_t)(self->callback->fifo_data_buf[1] << 8 | self->callback->fifo_data_buf[0]) * 70.00f;
      self->reg_data->gyr_y =
          (float)(int16_t)(self->callback->fifo_data_buf[3] << 8 | self->callback->fifo_data_buf[2]) * 70.00f;
      self->reg_data->gyr_z =
          (float)(int16_t)(self->callback->fifo_data_buf[5] << 8 | self->callback->fifo_data_buf[4]) * 70.00f;

      break;

    case ACC_OUT_TYPE_FLAG:
      ac_ready_flag = 1;

      self->reg_data->acc_x =
          (float)(int16_t)(self->callback->fifo_data_buf[1] << 8 | self->callback->fifo_data_buf[0]) * 0.122f;
      self->reg_data->acc_y =
          (float)(int16_t)(self->callback->fifo_data_buf[3] << 8 | self->callback->fifo_data_buf[2]) * 0.122f;
      self->reg_data->acc_z =
          (float)(int16_t)(self->callback->fifo_data_buf[5] << 8 | self->callback->fifo_data_buf[4]) * 0.122f;

      break;

    case TIMESTAMP_OUT_TYPE_FLAG:
      timestamp_ready_flag = 1;

      self->reg_data->timestamp_2 =
          (uint32_t)(self->callback->fifo_data_buf[0] | self->callback->fifo_data_buf[1] << 8 |
                     self->callback->fifo_data_buf[2] << 16 | self->callback->fifo_data_buf[3] << 24);

      break;

    default:
      break;
  }

  self->callback->wtm_cur--;
  self->callback->rx_flag = DATA_OUT_TYPE_FLAG;
  memset(self->callback->fifo_data_buf, 0, sizeof(self->callback->fifo_data_buf));

  if (timestamp_ready_flag == 1 && gy_ready_flag == 1 && ac_ready_flag == 1) {

    lsm6ds3trMotionFxDetermin(self);
    self->reg_data->timestamp_1 = self->reg_data->timestamp_2;

    gy_ready_flag = 0;
    ac_ready_flag = 0;
    timestamp_ready_flag = 0;
  }

  if (self->callback->wtm_cur > LSM6DSOW_FIFO_WATERMARK / 2) {
    st |= HAL_I2C_Mem_Read_IT(self->i2c, self->reg_cmd->read, LSM6DSOW_FIFO_DATA_OUT_TAG_REG, I2C_MEMADD_SIZE_8BIT,
                              self->callback->status_buf, 1);
  }

  if (st != HAL_OK) {
    self->err_st = LSM6DSOW_ACCESS_REG_FIFO_DATA_OUT_ERR;
    SEGGER_RTT_printf(0, "%s Fifo data out unsuc\n%s", RTT_CTRL_TEXT_BRIGHT_WHITE, RTT_CTRL_RESET);
    return false;
  }
  return true;
}

/*------------------------------------------------- fifo init ----------------------------------------------------------------*/
bool lsm6dsowFifoInit(Lsm6dsow* ins) {

  int result = lsm6dsowFifoWatermarkSet(ins, LSM6DSOW_FIFO_WATERMARK);
  CHECK_RESULT(result);

  result = lsm6dsowFifoBDR(ins, BDR_GY_RATE_104HZ, BDR_XL_RATE_104HZ);
  CHECK_RESULT(result);

  result = lsm6dsowFifoModeSet(ins, LSM6DSOW_STREAM_MODE);
  CHECK_RESULT(result);

  result = lsm6dsowFifoINT1PulseSet(ins);
  CHECK_RESULT(result);

  result = lsm6dsowFifoINT1Set(ins, INT1_FIFO_TH_WRITE);
  CHECK_RESULT(result);

  result = lsm6dsowFifoStatus(ins);
  CHECK_RESULT(result);

  result = lsm6dsowFifoTimestamp(ins);
  CHECK_RESULT(result);

  return true;
}
