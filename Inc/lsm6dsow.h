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

// Fifo WTM value
#define LSM6DSOW_FIFO_WATERMARK 10

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

// FIFO MODE
#define LSM6DSOW_BYPASS_MODE 0x00
#define LSM6DSOW_FIFO_MODE 0x01
#define LSM6DSOW_STREAM_TO_FIFO_MODE 0x03
#define LSM6DSOW_BYPASS_TO_STREAM_MODE 0x04
#define LSM6DSOW_STREAM_MODE 0x06
#define LSM6DSOW_FIFO_MODE_ND 0x08

// Common Register
#define LSM6DSOW_FIFO_INT1_CTRL_REG 0x0D
#define LSM6DSOW_FIFO_DATA_OUT_TAG_REG 0x78

#define DATA_OUT_TYPE_FLAG 0x00
#define GYR_OUT_TYPE_FLAG 0x01
#define ACC_OUT_TYPE_FLAG 0x02
#define TIMESTAMP_OUT_TYPE_FLAG 0x03

typedef enum {
  LSM6DSOW_OK,
  LSM6DSOW_ID_ERR,
  LSM6DSOW_RESET_ERR,
  LSM6DSOW_ACC_PARAM_ERR,
  LSM6DSOW_GYR_PARAM_ERR,
  LSM6DSOW_FIFO_SET_WATERMARK_ERR,
  LSM6DSOW_FIFO_SET_BDR_ERR,
  LSM6DSOW_FIFO_SET_MODE_ERR,
  LSM6DSOW_FIFO_SET_INT1_ERR,
  LSM6DSOW_FIFO_SET_INT1_PULSED_ERR,
  LSM6DSOW_FIFO_SET_TIMER_REG_ERR,
  LSM6DSOW_ACCESS_REG_FIFO_STATUS2_ERR,
  LSM6DSOW_ACCESS_REG_FIFO_DATA_OUT_ERR,
} Lsm6dsowErr;

typedef struct {
  uint8_t read;
  uint8_t write;
} Lsm6dsowRegCmd;

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
} Lsm6dsowRegData;

typedef struct {
  float roll;
  float pitch;
  float yaw;
} Lsm6dsowOutput;

typedef struct {
  uint16_t wtm_cur;
  uint8_t rx_flag;
  uint8_t status_buf[1];
  uint8_t fifo_data_buf[6];
} Lsm6dsowCallback;

typedef struct {
  I2C_HandleTypeDef* i2c;
  Lsm6dsowErr err_st;
  Lsm6dsowRegCmd* reg_cmd;
  Lsm6dsowRegData* reg_data;
  Lsm6dsowOutput* output;
  Lsm6dsowCallback* callback;
} Lsm6dsow;

void lsm6dsowInit(Lsm6dsow* ins, I2C_HandleTypeDef* i2c);
bool lsm6dsowRegInit(Lsm6dsow* ins);

bool lsm6dsowFifoStatus(Lsm6dsow* self);
bool lsm6dsowFifoData(Lsm6dsow* self);
bool lsm6dsowI2cReadCallback(Lsm6dsow* self);

#endif  //LSM6DSOW_H
