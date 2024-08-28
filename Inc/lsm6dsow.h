//
// Created by hanghang on 24-8-11.
//

#ifndef LSM6DSOW_H
#define LSM6DSOW_H

#include <stdbool.h>
#include "i2c.h"

#define CHECK_RESULT(result) \
  do {                       \
    if (result == false)     \
      return result;         \
  } while (0)

#define LSM6DSOW_READ 0xD5
#define LSM6DSOW_WRITE 0xD4

/* reg addr */
#define LSM6DSOW_ID_ADD 0x0F
#define LSM6DSOW_CTRL1_XL 0x10     // acceleration reg
#define LSM6DSOW_CTRL2_G 0x11      // gyroscope reg
#define LSM6DSOW_CTRL3_C_ADD 0x12  // reset & addr auto increase & bdu update auto
#define LSM6DSOW_CTRL8_XL 0x17
#define LSM6DSOW_CTRL7_G 0x16
#define LSM6DSOW_CTRL6_C 0x15
#define LSM6DSOW_CTRL4_C 0x13
#define LSM6DSOW_STATUS_REG 0x1E
#define LSM6DSOW_OUTX_L_XL 0x28
#define LSM6DSOW_OUTX_L_G 0x22
#define LSM6DSOW_OUT_TEMP_L 0x20
/* common value */
#define LSM6DSOW_CTRL3_C_VALUE 0XC4
#define LSM6DSOW_STATUS_TEMPERATURE 0x04
#define LSM6DSOW_STATUS_GYROSCOPE 0x02
#define LSM6DSOW_STATUS_ACCELEROMETER 0x01

// Linear acceleration out data rate
#define LSM6DSOW_ACC_RATE_0 0x00
#define LSM6DSOW_ACC_RATE_1HZ6 0xB0
#define LSM6DSOW_ACC_RATE_12HZ5 0x10
#define LSM6DSOW_ACC_RATE_26HZ 0x20
#define LSM6DSOW_ACC_RATE_52HZ 0x30
#define LSM6DSOW_ACC_RATE_104HZ 0x40
#define LSM6DSOW_ACC_RATE_208HZ 0x50
#define LSM6DSOW_ACC_RATE_416HZ 0x60
#define LSM6DSOW_ACC_RATE_833HZ 0x70
#define LSM6DSOW_ACC_RATE_1660HZ 0x80
#define LSM6DSOW_ACC_RATE_3330HZ 0x90
#define LSM6DSOW_ACC_RATE_6660HZ 0xA0
// Accelerometer full-scale.
#define LSM6DSOW_ACC_FSXL_2G 0x00
#define LSM6DSOW_ACC_FSXL_16G 0x04
#define LSM6DSOW_ACC_FSXL_4G 0x08
#define LSM6DSOW_ACC_FSXL_8G 0x0C
// Accelerometer analog chain bandwidth
#define LSM6DSOW_ACC_BW0XL_1500HZ 0x00
#define LSM6DSOW_ACC_BW0XL_400HZ 0x01
// Accelerometer bandwidth selection
#define LSM6DSOW_ACC_LOW_PASS_ODR_50 0x88
#define LSM6DSOW_ACC_LOW_PASS_ODR_100 0xA8
#define LSM6DSOW_ACC_LOW_PASS_ODR_9 0xC8
#define LSM6DSOW_ACC_LOW_PASS_ODR_400 0xE8

// Linear gyroscope out data rate
#define LSM6DSOW_GYR_RATE_0 0x00
#define LSM6DSOW_GYR_RATE_1HZ6 0xB0
#define LSM6DSOW_GYR_RATE_12HZ5 0x10
#define LSM6DSOW_GYR_RATE_26HZ 0x20
#define LSM6DSOW_GYR_RATE_52HZ 0x30
#define LSM6DSOW_GYR_RATE_104HZ 0x40
#define LSM6DSOW_GYR_RATE_208HZ 0x50
#define LSM6DSOW_GYR_RATE_416HZ 0x60
#define LSM6DSOW_GYR_RATE_833HZ 0x70
#define LSM6DSOW_GYR_RATE_1660HZ 0x80
#define LSM6DSOW_GYR_RATE_3330HZ 0x90
#define LSM6DSOW_GYR_RATE_6660HZ 0xA0
// Gyroscope full-scale.
#define LSM6DSOW_GYR_FSG_245 0x00
#define LSM6DSOW_GYR_FSG_500 0x04
#define LSM6DSOW_GYR_FSG_1000 0x08
#define LSM6DSOW_GYR_FSG_2000 0x0C
#define LSM6DSOW_CTRL1_XL 0x10
#define LSM6DSOW_CTRL8_XL 0x17

// CTRL7_G register value
#define LSM6DSOW_CTRL7_G_HM_MODE_ENABLE 0x00
#define LSM6DSOW_CTRL7_G_HM_MODE_DISABLE 0x80
#define LSM6DSOW_CTRL7_G_HP_EN_DISABLE 0x00
#define LSM6DSOW_CTRL7_G_HP_EN_ENABLE 0x40
#define LSM6DSOW_CTRL7_G_HPM_16MHZ 0x00
#define LSM6DSOW_CTRL7_G_HPM_65MHZ 0x10
#define LSM6DSOW_CTRL7_G_HPM_260MHZ 0x20
#define LSM6DSOW_CTRL7_G_HPM_1HZ04 0x30
#define LSM6DSOW_CTRL7_G_ROUNDING_STATUS_DISABLE 0x04
#define LSM6DSOW_CTRL7_G_ROUNDING_STATUS_ENABLE 0x00

// CTRL6_C register value
#define LSM6DSOW_CTRL6_C_EDGE_TRIGGER 0x80
#define LSM6DSOW_CTRL6_C_LEVEL_TRIGGER 0x40
#define LSM6DSOW_CTRL6_C_LEVEL_LATCHED 0x60
#define LSM6DSOW_CTRL6_C_LEVEL_FIFO 0xC0
#define LSM6DSOW_CTRL6_C_XL_HM_MODE_ENABLE 0x00
#define LSM6DSOW_CTRL6_C_XL_HM_MODE_DISABLE 0x10
#define LSM6DSOW_CTRL6_C_FTYPE_1 0x00
#define LSM6DSOW_CTRL6_C_FTYPE_2 0x01
#define LSM6DSOW_CTRL6_C_FTYPE_3 0x02
#define LSM6DSOW_CTRL6_C_FTYPE_4 0x03

// CTRL4_C register value
#define LSM6DSOW_CTRL4_DEN_XL_EN_DISABLE 0x00
#define LSM6DSOW_CTRL4_DEN_XL_EN_ENABLE 0x80
#define LSM6DSOW_CTRL4_SLEEP_ENABLE 0x40
#define LSM6DSOW_CTRL4_SLEEP_DISABLE 0x00
#define LSM6DSOW_CTRL4_DEN_DRDY_INT1_DISBALE 0x00
#define LSM6DSOW_CTRL4_DEN_DRDY_INT1_ENABLE 0x20
#define LSM6DSOW_CTRL4_DRDY_MASK_DISABLE 0x00
#define LSM6DSOW_CTRL4_DRDY_MASK_ENABLE 0x08
#define LSM6DSOW_CTRL4_I2C_DISABLE 0x04
#define LSM6DSOW_CTRL4_I2C_ENABLE 0x00
#define LSM6DSOW_CTRL4_LPF1_SELG_ENABLE 0x02
#define LSM6DSOW_CTRL4_LPF1_SELG_DISABLE 0x00

// FIFO register
#define LSM6DSOW_FIFO_CRTL1 0x07  // only watermark value
#define LSM6DSOW_FIFO_CRTL2 0x08
#define LSM6DSOW_FIFO_CRTL3 0x09
#define LSM6DSOW_FIFO_CRTL4 0x0A
#define LSM6DSOW_FIFO_COUNTER_BDR_REG1 0x0B
#define LSM6DSOW_FIFO_STATUS1 0x3A
#define LSM6DSOW_FIFO_STATUS2 0x3B
#define FIFO_DATA_OUT_L 0x3E
#define FIFO_DATA_OUT_H 0x3F

// FIFO MODE
#define LSM6DSOW_BYPASS_MODE 0x00
#define LSM6DSOW_FIFO_MODE 0x01
#define LSM6DSOW_STREAM_TO_FIFO_MODE 0x03
#define LSM6DSOW_BYPASS_TO_STREAM_MODE 0x04
#define LSM6DSOW_STREAM_MODE 0x06
#define LSM6DSOW_FIFO_MODE_ND 0x08

// FIFO ODR
#define LSM6DSOW_FIFO_DISABLE 0x00
#define LSM6DSOW_FIFO_12Hz5 0x01
#define LSM6DSOW_FIFO_26Hz 0x02
#define LSM6DSOW_FIFO_52Hz 0x03
#define LSM6DSOW_FIFO_104Hz 0x04
#define LSM6DSOW_FIFO_208Hz 0x05
#define LSM6DSOW_FIFO_416Hz 0x06
#define LSM6DSOW_FIFO_833Hz 0x07
#define LSM6DSOW_FIFO_1k66Hz 0x08
#define LSM6DSOW_FIFO_3k33Hz 0x09
#define LSM6DSOW_FIFO_6k66Hz 0x0A

// Fifo decimation
#define LSM6DSOW_NO_SET_FIFO 0X00
#define LSM6DSOW_DEC_FACTOR_NO 0X01
#define LSM6DSOW_DEC_FACTOR_2 0X02
#define LSM6DSOW_DEC_FACTOR_3 0X03
#define LSM6DSOW_DEC_FACTOR_4 0X04
#define LSM6DSOW_DEC_FACTOR_8 0X05
#define LSM6DSOW_DEC_FACTOR_16 0X06
#define LSM6DSOW_DEC_FACTOR_32 0X07

// Timer register
#define LSM6DSOW_TIMER_CRTL10 0x19
#define LSM6DSOW_TIMER_WAKE_UP_DUR 0x5C

// Timer en
#define LSM6DSOW_TIMER_EN 0x20
#define LSM6DSOW_FIFO_TIMER_EN 0x80
#define LSM6DSOW_TIMER_HR_25US 0X10

// Int register
#define LSM6DSOW_FIFO_INT1_CTRL 0x0D

// INT1 value
#define LSM6DSOW_INT1_FULL_FLAG 0x20

typedef enum {
  LSM6DSOW_OK,
  LSM6DSOW_ID_ERR,
  LSM6DSOW_RESET_ERR,
  LSM6DSOW_ACC_PARAM_ERR,
  LSM6DSOW_GYR_PARAM_ERR,
  LSM6DSOW_ACC_BANDWIDTH_ERR,
  LSM6DSOW_REG7_ERR,
  LSM6DSOW_REG6_ERR,
  LSM6DSOW_REG4_ERR,
  LSM6DSOW_STATUS_UPDATE_ERR,
  LSM6DSOW_FIFO_SET_WATERMARK_ERR,
  LSM6DSOW_FIFO_SET_BDR_ERR,
  LSM6DSOW_FIFO_SET_MODE_ERR,
  LSM6DSOW_FIFO_SET_INT1_ERR,
  LSM6DSOW_FIFO_SET_INT1_PULSED_ERR,
  LSM6DSOW_TIMER_SET_MODE_ERR,
  LSM6DSOW_FIFO_SET_TIMER_ERR,
  LSM6DSOW_DEC_THIRD_FOURTH_FIFO_ERR,
  LSM6DSOW_DEC_XL_GYR_FIFO_ERR,
  LSM6DSOW_ACCESS_REG_FIFO_STATUS2_ERR,
  LSM6DSOW_ACCESS_REG_FIFO_STATUS1_ERR,
  LSM6DSOW_ACCESS_REG_FIFO_DATA_OUT_ERR,
  LSM6DSOW_INT1_SET_FIFO_FLAG_ERR,
  LSM6DSOW_TX_ERR,
  LSM6DSOW_RX_ERR,
  LSM6DSOW_ERR
} Lsm6dsowErr;

typedef struct {
  uint8_t read;
  uint8_t write;
} Lsm6dsowReg;

typedef struct {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  uint32_t timestamp;
  uint32_t timestamp_1;
  uint32_t timestamp_2;
} Lsm6dsowData;

// Fifo status
#define LSM6DSOW_FIFO_EMPTY 0x00
#define LSM6DSOW_FIFO_FULL 0x01
#define LSM6DSOW_FIFO_WATERMARK 104

typedef struct {
  uint8_t st;
} Lsm6dsowFifo;

// Check lsm update
#define LSM6DSOW_FLAG_UPDATED 01
#define LSM6DSOW_FLAG_UNUPDATE 02

// Read cb sig
#define LSM6DSOW_READ_FLAG_FIFO_STATUS2 0x01
#define LSM6DSOW_READ_FLAG_GET_FIFO_DATA 0x02

typedef struct {
  I2C_HandleTypeDef* i2c;
  Lsm6dsowErr err_st;
  Lsm6dsowReg reg;
  Lsm6dsowData data;
  Lsm6dsowFifo fifo;
  int16_t temp;
  uint8_t read_sig_flag;  // various tx cb
  uint8_t write_sig_flag;
  uint8_t reg_read_buf[1];
  uint8_t fifo_data_get_buf[24];
  char data_buf[14];  // direct access to registers without fifo
  uint8_t update_flag;
} Lsm6dsow;

void lsm6dsowInit(Lsm6dsow* ins, I2C_HandleTypeDef* i2c);
bool lsm6dsowRegInit(Lsm6dsow* ins);

void lsm6dsowOutputReadCb(Lsm6dsow* ins);
void lsm6dsowAccGryOutput(Lsm6dsow* ins);
bool lsm6dsowFifoResult(Lsm6dsow* ins);

void lsm6dsowFifoClear(Lsm6dsow* ins);
bool lsm6dsowFifoStatus2It(Lsm6dsow* ins);
bool lsm6dstrFifoFlow(Lsm6dsow* ins);
bool lsm6dsowFifoInt1Full(Lsm6dsow* ins, uint8_t dev_read_addr, uint8_t dev_write_addr, uint8_t mem_addr,
                          uint8_t int1_full_flag);
bool lsm6dsowFifoStatus(Lsm6dsow* self);
bool lsm6dsowFifoData(Lsm6dsow* self);

#endif  //LSM6DSOW_H
