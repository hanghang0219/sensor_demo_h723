//
// Created by hanghang on 24-7-26.
//

#include "lsm6ds3tr.h"
#include <cmsis_os2.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "lsm6ds3tr_motionFX.h"

int lsm6dstrFifoInit(Lsm6ds3tr *ins);
void lsm6ds3trFifoClear(Lsm6ds3tr *ins);

Lsm6ds3trReg lsm6ds3tr_reg = {.read = LSM6DS3TR_READ, .write = LSM6DS3TR_WRITE};
Lsm6ds3trData lsm6ds3tr_data = {0};
Lsm6ds3trFifo lsm6ds3tr_fifo = {LSM6DS3TR_FIFO_EMPTY};

void lsm6ds3trInit(Lsm6ds3tr *ins, I2C_HandleTypeDef *i2c) {
  ins->i2c = i2c;
  ins->err_st = LSM6DS3TR_OK;
  ins->reg = lsm6ds3tr_reg;
  ins->data = lsm6ds3tr_data;
  ins->fifo = lsm6ds3tr_fifo;
  ins->update_flag = LSM6DS3TR_FLAG_UNUPDATE;
}

/* 1.identify id */
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param mem_addr LSM6DS3TR_ID_ADD
  */
int lsm6dstrWhoIAm(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t mem_addr) {

  uint8_t who_am_i = 0;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1);
  if (st != HAL_OK || who_am_i != 0x6A) {
    ins->err_st = LSM6DS3TR_ID_ERR;
    return -1;
  }
  return 0;
}

/* 2.reset && bdu improve
 *  the accelerometer and the gyroscope in power-down mode after reset 7:1
 *  register add automatically increase  2:1
 *  bdu update automatically   6:1
 *  128 + 4 + 64
 */
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TR_CTRL3_C_ADD
  * @param mem_value LSM6DS3TR_CTRL3_C_VALUE
  */
int lsm6dstrResetDevice(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                        uint8_t mem_value) {
  uint8_t reset_sw_value = 0;
  uint8_t reset_boot_value[1] = {mem_value};
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, reset_boot_value, 1, 1);

  osDelay(3);  // must wait reset
  st |= HAL_I2C_Mem_Read(&hi2c1, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &reset_sw_value, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_RESET_ERR;
    return -1;
  }
  return 0;
}

/* 3.acceleration ：833hz 4g*/
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TR_CTRL1_XL
  * @param mem_value1 LSM6DS3TRC_ACC_RATE_833HZ
  * @param mem_value2 LSM6DS3TRC_ACC_FSXL_4G
  */
int lsm6ds3trAccParam(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                      uint8_t mem_value1, uint8_t mem_value2) {
  uint8_t acceleration = 0;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &acceleration, 1, 1);
  acceleration |= mem_value1;
  acceleration |= mem_value2;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &acceleration, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACC_PARAM_ERR;
    return -1;
  }
  return 0;
}

/* 4.gyroscope 833hz 2000*/
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TR_CTRL2_G
  * @param mem_value1 LSM6DS3TRC_GYR_RATE_833HZ
  * @param mem_value2 LSM6DS3TRC_GYR_FSG_2000
  */
int lsm6ds3trGyrParam(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                      uint8_t mem_value1, uint8_t mem_value2) {
  uint8_t gyroscope = 0;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &gyroscope, 1, 1);
  gyroscope |= mem_value1;
  gyroscope |= mem_value2;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &gyroscope, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_GYR_PARAM_ERR;
    return -1;
  }
  return 0;
}

/* 6.acceleration bandwidth */
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr1 LSM6DS3TR_CTRL1_XL
  * @param mem_addr2 LSM6DS3TR_CTRL8_XL
  * @param mem_value1 LSM6DS3TRC_ACC_BW0XL_400HZ
  * @param mem_value2 LSM6DS3TRC_ACC_LOW_PASS_ODR_100
  */
int lsm6ds3trAccBandwidth(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr1,
                          uint8_t mem_addr2, uint8_t mem_value1, uint8_t mem_value2) {
  uint8_t acc_bandwidth = 0;
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr1, I2C_MEMADD_SIZE_8BIT, &acc_bandwidth, 1, 1);
  acc_bandwidth |= mem_value1;
  st |= HAL_I2C_Mem_Write(&hi2c1, dev_write_addr, mem_addr1, I2C_MEMADD_SIZE_8BIT, &acc_bandwidth, 1, 1);

  st |= HAL_I2C_Mem_Read(&hi2c1, dev_read_addr, mem_addr2, I2C_MEMADD_SIZE_8BIT, &acc_bandwidth, 1, 1);
  acc_bandwidth |= mem_value2;
  st |= HAL_I2C_Mem_Write(&hi2c1, dev_write_addr, mem_addr2, I2C_MEMADD_SIZE_8BIT, &acc_bandwidth, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACC_BANDWIDTH_ERR;
    return -1;
  }
  return 0;
}

/* 7.reg 7 */
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TR_CTRL7_G
  * @param mem_value1 LSM6DS3TRC_CTRL7_G_HP_EN_ENABLE
  * @param mem_value2 LSM6DS3TRC_CTRL7_G_HPM_260MHZ
  */
int lsm6ds3trReg7Param(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                       uint8_t mem_value1, uint8_t mem_value2) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= mem_value1 | mem_value2;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_REG7_ERR;
    return -1;
  }
  return 0;
}

/* 8.reg 6 */
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TRC_CTRL6_C
  * @param mem_value1 LSM6DS3TRC_CTRL6_C_FTYPE_1
  */
int lsm6ds3trReg6Param(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                       uint8_t mem_value1) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= mem_value1;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_REG6_ERR;
    return -1;
  }
  return 0;
}

/* 8.reg 4 */
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TRC_CTRL4_C
  * @param mem_value1 LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE
  */
int lsm6ds3trReg4Param(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                       uint8_t mem_value1) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= mem_value1;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_REG4_ERR;
    return -1;
  }
  return 0;
}

int lsm6ds3trRegInit(Lsm6ds3tr *ins) {

  int result = lsm6dstrWhoIAm(ins, LSM6DS3TR_READ, LSM6DS3TR_ID_ADD);
  CHECK_RESULT(result);

  result = lsm6dstrResetDevice(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_CTRL3_C_ADD, LSM6DS3TR_CTRL3_C_VALUE);
  CHECK_RESULT(result);

  result = lsm6ds3trAccParam(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_CTRL1_XL, LSM6DS3TRC_ACC_RATE_416HZ,
                             LSM6DS3TRC_ACC_FSXL_4G);
  CHECK_RESULT(result);

  result = lsm6ds3trGyrParam(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_CTRL2_G, LSM6DS3TRC_GYR_RATE_416HZ,
                             LSM6DS3TRC_GYR_FSG_2000);
  CHECK_RESULT(result);

  // result = lsm6ds3trAccBandwidth(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_CTRL1_XL, LSM6DS3TR_CTRL8_XL,
  //                                LSM6DS3TRC_ACC_BW0XL_400HZ, LSM6DS3TRC_ACC_LOW_PASS_ODR_100);
  // CHECK_RESULT(result);
  //
  // result = lsm6ds3trReg7Param(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_CTRL7_G, LSM6DS3TRC_CTRL7_G_HP_EN_ENABLE,
  //                             LSM6DS3TRC_CTRL7_G_HPM_260MHZ);
  // CHECK_RESULT(result);
  //
  // result = lsm6ds3trReg6Param(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TRC_CTRL6_C, LSM6DS3TRC_CTRL6_C_FTYPE_1);
  // CHECK_RESULT(result);
  //
  // result =
  //     lsm6ds3trReg4Param(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TRC_CTRL4_C, LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE);
  // CHECK_RESULT(result);

  result = lsm6dstrFifoInit(ins);
  CHECK_RESULT(result);

  return 0;
}

// FIFO init
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr1 LSM6DS3TRC_FIFO_CRTL1
  * @param mem_addr2 LSM6DS3TRC_FIFO_CRTL2
  * @param water_mark multiples of 24
  */
int lsm6ds3trFifoWatermarkSet(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr1,
                              uint8_t mem_addr2, uint16_t water_mark) {
  // watermark -> reg value  (0~2048) * 2
  uint8_t low_reg_value = (uint8_t)(water_mark & 0x00ff);
  uint8_t high_reg_value = (uint8_t)((water_mark & (0x0007 << 8)) >> 8);

  uint8_t buf[1] = {low_reg_value};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr1, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  st |= HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr2, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  buf[0] &= 0xf8;
  buf[0] |= high_reg_value;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr2, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_FIFO_SET_WATERMARK_ERR;
    return -1;
  }
  return 0;
}

/**
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TRC_FIFO_CRTL5
  * @param mode_set LSM6DS3TR_STREAM_MODE
  * @param fifo_odr LSM6DS3TR_FIFO_833Hz
  */
int lsm6ds3trFifoModeSet(Lsm6ds3tr *ins, uint8_t dev_write_addr, uint8_t mem_addr, uint8_t mode_set, uint8_t fifo_odr) {
  uint8_t buf[1] = {mode_set | (fifo_odr << 3)};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_FIFO_SET_MODE_ERR;
    return -1;
  }
  return 0;
}

/* enable timer */
/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TR_TIMER_CRTL10
  * @param mode_set LSM6DS3TR_TIMER_EN
  */
int lsm6ds3trTimerEnSet(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                        uint8_t mode_set) {
  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= mode_set;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_TIMER_SET_MODE_ERR;
    return -1;
  }
  return 0;
}

/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TRC_FIFO_CRTL2
  * @param mode_set LSM6DS3TR_FIFO_TIMER_EN
  */
int lsm6ds3trFifoSetTimer(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                          uint8_t mode_set) {
  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= mode_set;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_FIFO_SET_TIMER_ERR;
    return -1;
  }
  return 0;
}

/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TR_TIMER_WAKE_UP_DUR
  * @param timer_hr LSM6DS3TR_TIMER_HR_25US
  */
int lsm6ds3trWakeUPDur(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                       uint8_t timer_hr) {
  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= timer_hr;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_WAKE_UP_DUR_ERR;
    return -1;
  }
  return 0;
}

/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TRC_FIFO_CRTL4
  * @param third_dec LSM6DS3TR_DEC_FACTOR_NO
  * @param fourth_dec LSM6DS3TR_DEC_FACTOR_NO
  */
int lsm6ds3trFifoDecThirdFourth(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                                uint8_t third_dec, uint8_t fourth_dec) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= third_dec | (fourth_dec << 3);
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_DEC_THIRD_FOURTH_FIFO_ERR;
    return -1;
  }

  return 0;
}

/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TRC_FIFO_CRTL3
  * @param xl_dec LSM6DS3TR_DEC_FACTOR_NO
  * @param gyr_dec LSM6DS3TR_DEC_FACTOR_NO
  */
int lsm6ds3trFifoDecXlGyr(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                          uint8_t xl_dec, uint8_t gyr_dec) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= xl_dec | (gyr_dec << 3);
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_DEC_XL_GYR_FIFO_ERR;
    return -1;
  }

  return 0;
}

/**
  * @param dev_read_addr  LSM6DS3TR_READ
  * @param dev_write_addr LSM6DS3TR_WRITE
  * @param mem_addr LSM6DS3TR_INT1_CTRL
  * @param int1_full_flag LSM6DS3TR_INT1_FULL_FLAG
  */
int lsm6ds3trFifoInt1Full(Lsm6ds3tr *ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                          uint8_t int1_full_flag) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, dev_read_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] = int1_full_flag;
  st |= HAL_I2C_Mem_Write(ins->i2c, dev_write_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_INT1_SET_FIFO_FLAG_ERR;
    return -1;
  }

  return 0;
}

/*------------------------------------------------- get fifo status and value  ----------------------------------------------------------------*/

// FIFO result
// The reg can check if the queue is full
int lsm6ds3trFifoStatus2(Lsm6ds3tr *ins) {
  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, ins->reg.read, LSM6DS3TRC_FIFO_STATUS2, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACCESS_REG_FIFO_STATUS2_ERR;
    return -1;
  }
  if ((buf[0] & 128) == 128)
    ins->fifo.st = LSM6DS3TR_FIFO_FULL;
  else if ((buf[0] & 0x10) == 0x10)
    ins->fifo.st = LSM6DS3TR_FIFO_EMPTY;
  return 0;
}

// The reg can check the status1 value : fifo information exist number
int lsm6ds3trFifoStatus1(Lsm6ds3tr *ins) {
  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, ins->reg.read, LSM6DS3TRC_FIFO_STATUS2, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACCESS_REG_FIFO_STATUS1_ERR;
    return -1;
  }

  /* TODO
   * buf[0] have fifo number
   */

  return 0;
}

// every fifo have 6 bytes info , there are 4 fifo -> 4 * 6
// fifo 1 : gyr
// fifo 2 : acc
// fifo 3 : unused sensor
// fifo 4 : time

uint8_t data[LSM6DS3TR_FIFO_WATERMARK] = {0};

int lsm6ds3trGetFifoInfo(Lsm6ds3tr *ins) {
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, ins->reg.read, FIFO_DATA_OUT_L, I2C_MEMADD_SIZE_8BIT, data,
                                          LSM6DS3TR_FIFO_WATERMARK, 1000);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACCESS_REG_FIFO_DATA_OUT_ERR;
    return -1;
  }

  return 0;
}

// reset fifo
void lsm6ds3trFifoClear(Lsm6ds3tr *ins) {
  lsm6ds3trFifoModeSet(ins, LSM6DS3TR_WRITE, LSM6DS3TRC_FIFO_CRTL5, LSM6DS3TR_BYPASS_MODE, LSM6DS3TR_FIFO_416Hz);
  lsm6ds3trFifoModeSet(ins, LSM6DS3TR_WRITE, LSM6DS3TRC_FIFO_CRTL5, LSM6DS3TR_STREAM_MODE, LSM6DS3TR_FIFO_416Hz);
}

// if (ins->fifo.st != LSM6DS3TR_FIFO_EMPTY) {
//   uint8_t num = 0;
//   while (ins->fifo.st != LSM6DS3TR_FIFO_EMPTY) {
//     num++;
//     HAL_I2C_Mem_Read(ins->i2c, ins->reg.read, FIFO_DATA_OUT_L, I2C_MEMADD_SIZE_8BIT, data, 24, 1000);
//     lsm6ds3trFifoStatus2(ins);
//   }
//   SEGGER_RTT_printf(0, "%sQUE already clear :%d\n %s", RTT_CTRL_TEXT_BRIGHT_GREEN, num, RTT_CTRL_RESET);
//   memset(&ins->data, 0, sizeof(Lsm6ds3trData));
// } else
//   SEGGER_RTT_printf(0, "%sQUE dont clear \n %s", RTT_CTRL_TEXT_BRIGHT_GREEN, RTT_CTRL_RESET);
// }

int lsm6ds3trFifoResult(Lsm6ds3tr *ins) {

  /* check watermark full */
  int result = lsm6ds3trFifoStatus2(ins);
  CHECK_RESULT(result);

  if (ins->fifo.st == LSM6DS3TR_FIFO_FULL) {
    uint8_t num = 0;
    SEGGER_RTT_printf(0, "%sQUE IS FULL\n%s", RTT_CTRL_TEXT_BRIGHT_GREEN, RTT_CTRL_RESET);

    result = lsm6ds3trGetFifoInfo(ins);
    CHECK_RESULT(result);

    for (int i = 0; i < LSM6DS3TR_FIFO_WATERMARK / 24; i++) {
      num++;
      int index = i * 24;
      ins->data.gyr_x = (float)((int16_t)(data[index + 1] << 8 | data[index + 0])) * 70.00f;
      ins->data.gyr_y = (float)((int16_t)(data[index + 3] << 8 | data[index + 2])) * 70.00f;
      ins->data.gyr_z = (float)((int16_t)(data[index + 5] << 8 | data[index + 4])) * 70.00f;
      ins->data.acc_x = (float)((int16_t)(data[index + 7] << 8 | data[index + 6])) * 0.122f;
      ins->data.acc_y = (float)((int16_t)(data[index + 9] << 8 | data[index + 8])) * 0.122f;
      ins->data.acc_z = (float)((int16_t)(data[index + 11] << 8 | data[index + 10])) * 0.122f;
      ins->data.timestamp = data[index + 18] << 8 | data[index + 19] << 16 | data[index + 21];

      // SEGGER_RTT_printf(0,
      //                   "%sAcc:x:%d y:%d z:%d%s \n"
      //                   "%sGxy:x:%d y:%d z:%d%s \n"
      //                   "%stimestamp: %d \n%s",
      //                   RTT_CTRL_TEXT_BRIGHT_RED, (int)ins->data.acc_x, (int)ins->data.acc_y, (int)ins->data.acc_z,
      //                   RTT_CTRL_RESET, RTT_CTRL_TEXT_GREEN, (int)ins->data.gyr_x, (int)ins->data.gyr_y,
      //                   (int)ins->data.gyr_z, RTT_CTRL_RESET, RTT_CTRL_TEXT_YELLOW, ins->data.timestamp,
      //                   RTT_CTRL_RESET);

      // motionFx

      if (i == 0)
        ins->data.timestamp_1 = ins->data.timestamp_2;
      else {
        ins->data.timestamp_2 = ins->data.timestamp;
        lsm6ds3trMotionFxDetermin(ins);
        ins->data.timestamp_1 = ins->data.timestamp_2;
      }
    }
    SEGGER_RTT_printf(0, "%snum = %d\n%s", RTT_CTRL_TEXT_BRIGHT_GREEN, num, RTT_CTRL_RESET);
    // SEGGER_RTT_printf(0, "QUE IS Emp && num = %d time = %dms \n", num,
    //                   (int)((ins->data.timestamp - old_record) * 25 / 1000));
  } else
    SEGGER_RTT_printf(0, "%sQUE IS NOFULL\n%s", RTT_CTRL_TEXT_BRIGHT_GREEN, RTT_CTRL_RESET);
  return 0;
}

// int lsm6ds3trFifoResult(Lsm6ds3tr *ins) {
//
//   /* check watermark full */
//   int result = lsm6ds3trFifoStatus2(ins);
//
//   CHECK_RESULT(result);
//   if (ins->fifo.st == LSM6DS3TR_FIFO_FULL) {
//     uint8_t num = 0;
//     uint32_t old_record = 0;
//
//     SEGGER_RTT_printf(0, "%sQUE IS FULL\n%s", RTT_CTRL_TEXT_BRIGHT_GREEN, RTT_CTRL_RESET);
//     while (ins->fifo.st != LSM6DS3TR_FIFO_EMPTY) {
//       result = lsm6ds3trGetFifoInfo(ins);
//       result |= lsm6ds3trFifoStatus2(ins);
//       CHECK_RESULT(result);
//
//       // if (num == 0)
//       //   old_record = ins->data.timestamp;
//       if (num == 0) {
//         old_record = ins->data.timestamp;
//         ins->data.timestamp_1 = ins->data.timestamp;
//         ins->data.timestamp_2 = ins->data.timestamp;
//       } else
//         ins->data.timestamp_2 = ins->data.timestamp;
//
//       SEGGER_RTT_printf(0,
//                         "%sAcc:x:%d y:%d z:%d%s \n"
//                         "%sGxy:x:%d y:%d z:%d%s \n"
//                         "%stimestamp: %d \n%s",
//                         RTT_CTRL_TEXT_BRIGHT_RED, (int)ins->data.acc_x, (int)ins->data.acc_y, (int)ins->data.acc_z,
//                         RTT_CTRL_RESET, RTT_CTRL_TEXT_GREEN, (int)ins->data.gyr_x, (int)ins->data.gyr_y,
//                         (int)ins->data.gyr_z, RTT_CTRL_RESET, RTT_CTRL_TEXT_YELLOW, ins->data.timestamp,
//                         RTT_CTRL_RESET);
//
//       // motionFx
//       // ins->data.timestamp_2 = ins->data.timestamp;
//       lsm6ds3trMotionFxDetermin(ins);
//       ins->data.timestamp_1 = ins->data.timestamp_2;
//       num++;
//     }
//
//     SEGGER_RTT_printf(0, "QUE IS Emp && num = %d time = %dms \n", num,
//                       (int)((ins->data.timestamp - old_record) * 25 / 1000));
//   } else
//     osDelay(1);
//
//   return 0;
// }

/*------------------------------------------------- get fifo status and value by it ----------------------------------------------------------------*/
int lsm6ds3trFifoStatus2It(Lsm6ds3tr *ins) {
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read_IT(ins->i2c, ins->reg.read, LSM6DS3TRC_FIFO_STATUS2, I2C_MEMADD_SIZE_8BIT, ins->reg_read_buf, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACCESS_REG_FIFO_STATUS2_ERR;
    return -1;
  }
  ins->read_sig_flag = LSM6DS3TR_READ_FLAG_FIFO_STATUS2;
  return 0;
}

int lsm6ds3trGetFifoInfoIt(Lsm6ds3tr *ins) {
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read_IT(ins->i2c, ins->reg.read, FIFO_DATA_OUT_L, I2C_MEMADD_SIZE_8BIT, ins->fifo_data_get_buf, 24);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACCESS_REG_FIFO_DATA_OUT_ERR;
    return -1;
  }
  ins->read_sig_flag = LSM6DS3TR_READ_FLAG_GET_FIFO_DATA;
  return 0;
}

void lsm6ds3trFifoDataAnalysis(Lsm6ds3tr *ins) {
  ins->data.gyr_x = (float)((int16_t)(ins->fifo_data_get_buf[1] << 8 | ins->fifo_data_get_buf[0])) * 70.00f;
  ins->data.gyr_y = (float)((int16_t)(ins->fifo_data_get_buf[3] << 8 | ins->fifo_data_get_buf[2])) * 70.00f;
  ins->data.gyr_z = (float)((int16_t)(ins->fifo_data_get_buf[5] << 8 | ins->fifo_data_get_buf[4])) * 70.00f;
  ins->data.acc_x = (float)((int16_t)(ins->fifo_data_get_buf[7] << 8 | ins->fifo_data_get_buf[6])) * 0.122f;
  ins->data.acc_y = (float)((int16_t)(ins->fifo_data_get_buf[9] << 8 | ins->fifo_data_get_buf[8])) * 0.122f;
  ins->data.acc_z = (float)((int16_t)(ins->fifo_data_get_buf[11] << 8 | ins->fifo_data_get_buf[10])) * 0.122f;
  ins->data.timestamp = ins->fifo_data_get_buf[18] << 8 | ins->fifo_data_get_buf[19] << 16 | ins->fifo_data_get_buf[21];

  SEGGER_RTT_printf(0,
                    "%sAcc:x:%d y:%d z:%d%s \n"
                    "%sGxy:x:%d y:%d z:%d%s \n"
                    "%stimestamp: %d \n%s",
                    RTT_CTRL_TEXT_BRIGHT_RED, (int)ins->data.acc_x, (int)ins->data.acc_y, (int)ins->data.acc_z,
                    RTT_CTRL_RESET, RTT_CTRL_TEXT_GREEN, (int)ins->data.gyr_x, (int)ins->data.gyr_y,
                    (int)ins->data.gyr_z, RTT_CTRL_RESET, RTT_CTRL_TEXT_YELLOW, ins->data.timestamp, RTT_CTRL_RESET);
}

void lsm6ds3trFifoMotionFxAnalysis(Lsm6ds3tr *ins) {
  static uint8_t first_analysis = 0;
  if (first_analysis) {
    ins->data.timestamp_2 = ins->data.timestamp;
  } else {
    ins->data.timestamp_1 = ins->data.timestamp_2;
    first_analysis++;
  }
  lsm6ds3trMotionFxDetermin(ins);
  ins->data.timestamp_1 = ins->data.timestamp_2;
}

int lsm6dstrFifoFlow(Lsm6ds3tr *ins) {
  static uint16_t data_num = 0;
  int result;

  switch (ins->read_sig_flag) {
    case LSM6DS3TR_READ_FLAG_FIFO_STATUS2:
      result = lsm6ds3trGetFifoInfoIt(ins);
      break;
    case LSM6DS3TR_READ_FLAG_GET_FIFO_DATA:
      lsm6ds3trFifoDataAnalysis(ins);
      // lsm6ds3trFifoMotionFxAnalysis(ins);
      if (data_num < LSM6DS3TR_FIFO_WATERMARK / 2) {
        result = lsm6ds3trGetFifoInfoIt(ins);
        data_num++;
      } else
        data_num = 0;
      break;
    default:
      break;
  }
  CHECK_RESULT(result);
  return 0;
}

/*------------------------------------------------- fifo init ----------------------------------------------------------------*/
int lsm6dstrFifoInit(Lsm6ds3tr *ins) {

  int result = lsm6ds3trFifoWatermarkSet(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TRC_FIFO_CRTL1,
                                         LSM6DS3TRC_FIFO_CRTL2, LSM6DS3TR_FIFO_WATERMARK);
  CHECK_RESULT(result);

  result =
      lsm6ds3trFifoModeSet(ins, LSM6DS3TR_WRITE, LSM6DS3TRC_FIFO_CRTL5, LSM6DS3TR_STREAM_MODE, LSM6DS3TR_FIFO_416Hz);
  CHECK_RESULT(result);

  result = lsm6ds3trTimerEnSet(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_TIMER_CRTL10, LSM6DS3TR_TIMER_EN);
  CHECK_RESULT(result);

  result = lsm6ds3trFifoSetTimer(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TRC_FIFO_CRTL2, LSM6DS3TR_FIFO_TIMER_EN);
  CHECK_RESULT(result);

  result =
      lsm6ds3trWakeUPDur(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_TIMER_WAKE_UP_DUR, LSM6DS3TR_TIMER_HR_25US);
  CHECK_RESULT(result);

  result = lsm6ds3trFifoDecThirdFourth(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TRC_FIFO_CRTL4,
                                       LSM6DS3TR_DEC_FACTOR_NO, LSM6DS3TR_DEC_FACTOR_NO);
  CHECK_RESULT(result);

  result = lsm6ds3trFifoDecXlGyr(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TRC_FIFO_CRTL3, LSM6DS3TR_DEC_FACTOR_NO,
                                 LSM6DS3TR_DEC_FACTOR_NO);
  CHECK_RESULT(result);

  result = lsm6ds3trFifoInt1Full(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_INT1_CTRL, LSM6DS3TR_INT1_FULL_FLAG);
  if (result != 0)
    CHECK_RESULT(result);
  return 0;
}

/*------------------------------------------------- demo get measured data ----------------------------------------------------------------*/
void lsm6ds3trAccGryOutput(Lsm6ds3tr *ins) {
  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, LSM6DS3TR_READ, LSM6DS3TRC_STATUS_REG, I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_STATUS_UPDATE_ERR;
    return;
  }
  /* 0x07: 01:acc 02:gyr 04:temp */
  if (buf[0] == 0x07) {
    /* can get value */
    st = HAL_I2C_Mem_Read_IT(ins->i2c, LSM6DS3TR_READ, LSM6DS3TRC_OUT_TEMP_L, I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)&ins->data_buf[0], 14);
    if (st != HAL_OK)
      ins->err_st = LSM6DS3TR_STATUS_UPDATE_ERR;
  } else {
    SEGGER_RTT_printf(0, "no ready \r\n", RTT_CTRL_TEXT_BRIGHT_BLUE);
    ins->update_flag = LSM6DS3TR_FLAG_UNUPDATE;

    /* only test */
    osDelay(20);
    lsm6ds3trAccGryOutput(ins);
    //
  }
}

void lsm6ds3trOutputReadCb(Lsm6ds3tr *ins) {
  ins->temp = (float)((int16_t)(ins->data_buf[1] << 8 | ins->data_buf[0])) / 256.0 + 25.0;
  ins->data.gyr_x = (float)((int16_t)(ins->data_buf[3] << 8 | ins->data_buf[2])) * 70.00f;
  ins->data.gyr_y = (float)((int16_t)(ins->data_buf[5] << 8 | ins->data_buf[4])) * 70.00f;
  ins->data.gyr_z = (float)((int16_t)(ins->data_buf[7] << 8 | ins->data_buf[6])) * 70.00f;
  ins->data.acc_x = (float)((int16_t)(ins->data_buf[9] << 8 | ins->data_buf[8])) * 0.122f;
  ins->data.acc_y = (float)((int16_t)(ins->data_buf[11] << 8 | ins->data_buf[10])) * 0.122f;
  ins->data.acc_z = (float)((int16_t)(ins->data_buf[13] << 8 | ins->data_buf[12])) * 0.122f;

  SEGGER_RTT_printf(0,
                    "%s acc: x:%d y:%d z:%d \n"
                    " gxy :x:%d y:%d z:%d temp:%d \n",
                    RTT_CTRL_TEXT_BRIGHT_RED, (int)ins->data.acc_x, (int)ins->data.acc_y, (int)ins->data.acc_z,
                    (int)ins->data.gyr_x, (int)ins->data.gyr_y, (int)ins->data.gyr_z, (int)ins->temp);

  ins->update_flag = LSM6DS3TR_FLAG_UPDATED;
}
