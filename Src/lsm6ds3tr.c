//
// Created by hanghang on 24-7-26.
//

#include "lsm6ds3tr.h"
#include <cmsis_os2.h>
#include "SEGGER_RTT.h"
#include "lsm6ds3tr_motionFX.h"

void lsm6ds3tr_init(Lsm6ds3tr *ins, I2C_HandleTypeDef *i2c) {
  ins->i2c = i2c;
  ins->err_st = LSM6DS3TR_OK;
  ins->acc_x = 0;
  ins->acc_y = 0;
  ins->acc_z = 0;
  ins->gyr_x = 0;
  ins->gyr_y = 0;
  ins->gyr_z = 0;
}

/* 1.identify id */
/**
  * @param devReadAddr  LSM6DS3TR_READ
  * @param memAddr LSM6DS3TR_ID_ADD
  */
int lsm6dstr_who_i_am(Lsm6ds3tr *ins, uint8_t devReadAddr, uint8_t memAddr) {

  uint8_t who_am_i = 0;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(
      ins->i2c, devReadAddr, memAddr, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1);
  if (st != HAL_OK || who_am_i != 0x6A) {
    ins->err_st = LSM6DS3TR_ID_ERR;
    return -1;
  }
  return 0;
}

/* 2.reset && bdu improve
 *  the accelerometer and the gyroscope in power-down mode after reset 7:1
 *  register add automatically increase  2:1
 *  bdu update automatically   6:0
 */
/**
  * @param devReadAddr  LSM6DS3TR_READ
  * @param devWriteAddr LSM6DS3TR_WRITE
  * @param memAddr LSM6DS3TR_CTRL3_C_ADD
  * @param memValue LSM6DS3TR_CTRL3_C_VALUE
  */
int lsm6dstr_reset_device(Lsm6ds3tr *ins, uint8_t devReadAddr,
                          uint8_t devWriteAddr, uint8_t memAddr,
                          uint8_t memValue) {
  uint8_t reset_sw_value = 0;
  uint8_t reset_boot_value[1] = {memValue};
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(ins->i2c, devWriteAddr, memAddr, I2C_MEMADD_SIZE_8BIT,
                        reset_boot_value, 1, 1);

  osDelay(3);  // must wait reset

  st |= HAL_I2C_Mem_Read(&hi2c1, devReadAddr, memAddr, I2C_MEMADD_SIZE_8BIT,
                         &reset_sw_value, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_RESET_ERR;
    return -1;
  }
  return 0;
}

/* 3.acceleration ：833hz 4g*/
/**
  * @param devReadAddr  LSM6DS3TR_READ
  * @param devWriteAddr LSM6DS3TR_WRITE
  * @param memAddr LSM6DS3TR_CTRL1_XL
  * @param memValue1 LSM6DS3TRC_ACC_RATE_833HZ
  * @param memValue2 LSM6DS3TRC_ACC_FSXL_4G
  */
int lsm6ds3tr_acc_param(Lsm6ds3tr *ins, uint8_t devReadAddr,
                        uint8_t devWriteAddr, uint8_t memAddr,
                        uint8_t memValue1, uint8_t memValue2) {
  uint8_t acceleration = 0;
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, devReadAddr, memAddr, I2C_MEMADD_SIZE_8BIT,
                       &acceleration, 1, 1);
  acceleration |= memValue1;
  acceleration |= memValue2;
  st |= HAL_I2C_Mem_Write(ins->i2c, devWriteAddr, memAddr, I2C_MEMADD_SIZE_8BIT,
                          &acceleration, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACC_PARAM_ERR;
    return -1;
  }
  return 0;
}

/* 4.gyroscope 833hz 2000*/
/**
  * @param devReadAddr  LSM6DS3TR_READ
  * @param devWriteAddr LSM6DS3TR_WRITE
  * @param memAddr LSM6DS3TR_CTRL2_G
  * @param memValue1 LSM6DS3TRC_GYR_RATE_833HZ
  * @param memValue2 LSM6DS3TRC_GYR_FSG_2000
  */
int lsm6ds3tr_gyr_param(Lsm6ds3tr *ins, uint8_t devReadAddr,
                        uint8_t devWriteAddr, uint8_t memAddr,
                        uint8_t memValue1, uint8_t memValue2) {
  uint8_t gyroscope = 0;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(
      ins->i2c, devReadAddr, memAddr, I2C_MEMADD_SIZE_8BIT, &gyroscope, 1, 1);
  gyroscope |= memValue1;
  gyroscope |= memValue2;
  st |= HAL_I2C_Mem_Write(ins->i2c, devWriteAddr, memAddr, I2C_MEMADD_SIZE_8BIT,
                          &gyroscope, 1, 1);

  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_GYR_PARAM_ERR;
    return -1;
  }
  return 0;
}

/* 6.acceleration bandwidth */
/**
  * @param devReadAddr  LSM6DS3TR_READ
  * @param devWriteAddr LSM6DS3TR_WRITE
  * @param memAddr1 LSM6DS3TR_CTRL1_XL
  * @param memAddr2 LSM6DS3TR_CTRL8_XL
  * @param memValue1 LSM6DS3TRC_ACC_BW0XL_400HZ
  * @param memValue2 LSM6DS3TRC_ACC_LOW_PASS_ODR_100
  */
int lsm6ds3tr_acc_bandwidth(Lsm6ds3tr *ins, uint8_t devReadAddr,
                            uint8_t devWriteAddr, uint8_t memAddr1,
                            uint8_t memAddr2, uint8_t memValue1,
                            uint8_t memValue2) {
  uint8_t acc_bandwidth = 0;
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, devReadAddr, memAddr1, I2C_MEMADD_SIZE_8BIT,
                       &acc_bandwidth, 1, 1);
  acc_bandwidth |= memValue1;
  st |= HAL_I2C_Mem_Write(&hi2c1, devWriteAddr, memAddr1, I2C_MEMADD_SIZE_8BIT,
                          &acc_bandwidth, 1, 1);

  st |= HAL_I2C_Mem_Read(&hi2c1, devReadAddr, memAddr2, I2C_MEMADD_SIZE_8BIT,
                         &acc_bandwidth, 1, 1);
  acc_bandwidth |= memValue2;
  st |= HAL_I2C_Mem_Write(&hi2c1, devWriteAddr, memAddr2, I2C_MEMADD_SIZE_8BIT,
                          &acc_bandwidth, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_ACC_BANDWIDTH_ERR;
    return -1;
  }
  return 0;
}

/* 7.reg 7 */
/**
  * @param devReadAddr  LSM6DS3TR_READ
  * @param devWriteAddr LSM6DS3TR_WRITE
  * @param memAddr LSM6DS3TR_CTRL7_G
  * @param memValue1 LSM6DS3TRC_CTRL7_G_HP_EN_ENABLE
  * @param memValue2 LSM6DS3TRC_CTRL7_G_HPM_260MHZ
  */
int lsm6ds3tr_reg7_param(Lsm6ds3tr *ins, uint8_t devReadAddr,
                         uint8_t devWriteAddr, uint8_t memAddr,
                         uint8_t memValue1, uint8_t memValue2) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, devReadAddr, memAddr,
                                          I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= memValue1 | memValue2;
  st |= HAL_I2C_Mem_Write(ins->i2c, devWriteAddr, memAddr, I2C_MEMADD_SIZE_8BIT,
                          buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_REG7_ERR;
    return -1;
  }
  return 0;
}

/* 8.reg 6 */
/**
  * @param devReadAddr  LSM6DS3TR_READ
  * @param devWriteAddr LSM6DS3TR_WRITE
  * @param memAddr LSM6DS3TRC_CTRL6_C
  * @param memValue1 LSM6DS3TRC_CTRL6_C_FTYPE_1
  */
int lsm6ds3tr_reg6_param(Lsm6ds3tr *ins, uint8_t devReadAddr,
                         uint8_t devWriteAddr, uint8_t memAddr,
                         uint8_t memValue1) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, devReadAddr, memAddr,
                                          I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= memValue1;
  st |= HAL_I2C_Mem_Write(ins->i2c, devWriteAddr, memAddr, I2C_MEMADD_SIZE_8BIT,
                          buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_REG6_ERR;
    return -1;
  }
  return 0;
}

/* 8.reg 4 */
/**
  * @param devReadAddr  LSM6DS3TR_READ
  * @param devWriteAddr LSM6DS3TR_WRITE
  * @param memAddr LSM6DS3TRC_CTRL4_C
  * @param memValue1 LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE
  */
int lsm6ds3tr_reg4_param(Lsm6ds3tr *ins, uint8_t devReadAddr,
                         uint8_t devWriteAddr, uint8_t memAddr,
                         uint8_t memValue1) {

  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ins->i2c, devReadAddr, memAddr,
                                          I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  buf[0] |= memValue1;
  st |= HAL_I2C_Mem_Write(ins->i2c, devWriteAddr, memAddr, I2C_MEMADD_SIZE_8BIT,
                          buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_REG4_ERR;
    return -1;
  }
  return 0;
}

int lsm6ds3tr_reg_init(Lsm6ds3tr *ins) {

  int result = lsm6dstr_who_i_am(ins, LSM6DS3TR_READ, LSM6DS3TR_ID_ADD);
  if (result != 0)
    return result;

  result =
      lsm6dstr_reset_device(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE,
                            LSM6DS3TR_CTRL3_C_ADD, LSM6DS3TR_CTRL3_C_VALUE);
  if (result != 0)
    return result;

  result = lsm6ds3tr_acc_param(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE,
                               LSM6DS3TR_CTRL1_XL, LSM6DS3TRC_ACC_RATE_833HZ,
                               LSM6DS3TRC_ACC_FSXL_4G);
  if (result != 0)
    return result;

  result = lsm6ds3tr_gyr_param(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE,
                               LSM6DS3TR_CTRL2_G, LSM6DS3TRC_GYR_RATE_833HZ,
                               LSM6DS3TRC_GYR_FSG_2000);
  if (result != 0)
    return result;

  result = lsm6ds3tr_acc_bandwidth(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE,
                                   LSM6DS3TR_CTRL1_XL, LSM6DS3TR_CTRL8_XL,
                                   LSM6DS3TRC_ACC_BW0XL_400HZ,
                                   LSM6DS3TRC_ACC_LOW_PASS_ODR_100);
  if (result != 0)
    return result;

  result = lsm6ds3tr_reg7_param(
      ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE, LSM6DS3TR_CTRL7_G,
      LSM6DS3TRC_CTRL7_G_HP_EN_ENABLE, LSM6DS3TRC_CTRL7_G_HPM_260MHZ);
  if (result != 0)
    return result;

  result = lsm6ds3tr_reg6_param(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE,
                                LSM6DS3TRC_CTRL6_C, LSM6DS3TRC_CTRL6_C_FTYPE_1);
  if (result != 0)
    return result;

  result = lsm6ds3tr_reg4_param(ins, LSM6DS3TR_READ, LSM6DS3TR_WRITE,
                                LSM6DS3TRC_CTRL4_C,
                                LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE);
  if (result != 0)
    return result;

  return 0;
}

#define Kp 500.0f    // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.005f    // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.01f  // 采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;  // 按比例缩小积分误差
float Yaw, Pitch, Roll;                 // 偏航角，俯仰角，翻滚角

// 加速度单位g，陀螺仪rad/s
void IMUUpdate(float gx, float gy, float gz, float ax, float ay, float az) {

  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  // 测量正常化,把加计的三维向量转成单位向量。
  norm = sqrt(ax * ax + ay * ay + az * az);
  ax = ax / norm;  //单位化
  ay = ay / norm;
  az = az / norm;

  // 估计方向的重力,世界坐标系重力分向量是通过方向旋转矩阵的最后一列的三个元素乘上加速度就可以算出机体坐标系中的重力向量。
  vx = 2 * (q1 * q3 - q0 * q2);  //由下向上方向的加速度在加速度计X分量
  vy = 2 * (q0 * q1 + q2 * q3);  //由下向上方向的加速度在加速度计X分量
  vz = q0 * q0 - q1 * q1 - q2 * q2 +
       q3 * q3;  //由下向上方向的加速度在加速度计Z分量

  // 这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。
  //（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。

  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  // 积分误差比例积分增益,计算陀螺仪测量的重力向量与估计方向的重力向量之间的误差。

  exInt = exInt + ex * Ki;
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // 调整后的陀螺仪测量,使用叉积误差来进行比例-积分（PI）修正陀螺仪的零偏。将修正量乘以比例增益Kp，并加上之前计算的积分误差exInt、eyInt和ezInt。
  gx = gx + Kp * ex + exInt;
  gy = gy + Kp * ey + eyInt;
  gz = gz + Kp * ez + ezInt;

  // 整合四元数率和正常化,根据陀螺仪的测量值和比例-积分修正值，对四元数进行更新。根据微分方程的离散化形式，将四元数的每个分量加上相应的微分项乘以采样周期的一半（halfT）。
  q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
  q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
  q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
  q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

  // 正常化四元数
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  Pitch = asin(2 * q2 * q3 + 2 * q0 * q1) * 57.3;  // pitch ,转换为度数
  Roll =
      atan2(-2 * q1 * q3 + 2 * q0 * q2, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) *
      57.3;  // rollv
  Yaw = atan2(2 * (q1 * q2 - q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) *
        57.3;  //偏移太大，等我找一个好用的

  SEGGER_RTT_printf(0, "%s Pitch: %d ,Roll: %d ,Yaw: %d \r\n",
                    RTT_CTRL_TEXT_BRIGHT_RED, (int)Pitch, (int)Roll, (int)Yaw);
}

void lsm6ds3tr_acc_gry_output(Lsm6ds3tr *ins) {
  uint8_t buf[1] = {0};
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(ins->i2c, LSM6DS3TR_READ, LSM6DS3TRC_STATUS_REG,
                       I2C_MEMADD_SIZE_8BIT, buf, 1, 1);
  if (st != HAL_OK) {
    ins->err_st = LSM6DS3TR_STATUS_UPDATE_ERR;
    return;
  }
  /* 0x07: 01:acc 02:gyr 04:temp */
  if (buf[0] == 0x07) {
    /* can get value */
    st = HAL_I2C_Mem_Read_IT(ins->i2c, LSM6DS3TR_READ, LSM6DS3TRC_OUT_TEMP_L,
                             I2C_MEMADD_SIZE_8BIT, &ins->data_buf[0], 14);
    if (st != HAL_OK)
      ins->err_st = LSM6DS3TR_STATUS_UPDATE_ERR;
  } else {
    SEGGER_RTT_printf(0, "no ready \r\n", RTT_CTRL_TEXT_BRIGHT_BLUE);
    ins->flag = LSM6DS3TR_FLAG_UNUPDATE;

    /* only test */
    osDelay(20);
    lsm6ds3tr_acc_gry_output(ins);
    //
  }
}

void lsm6ds3tr_output_read_cb(Lsm6ds3tr *ins) {
  ins->temp =
      (float)((int16_t)(ins->data_buf[1] << 8 | ins->data_buf[0])) / 256.0 +
      25.0;
  ins->gyr_x =
      (float)((int16_t)(ins->data_buf[3] << 8 | ins->data_buf[2])) * 70.00f;
  ins->gyr_y =
      (float)((int16_t)(ins->data_buf[5] << 8 | ins->data_buf[4])) * 70.00f;
  ins->gyr_z =
      (float)((int16_t)(ins->data_buf[7] << 8 | ins->data_buf[6])) * 70.00f;
  ins->acc_x =
      (float)((int16_t)(ins->data_buf[9] << 8 | ins->data_buf[8])) * 0.122f;
  ins->acc_y =
      (float)((int16_t)(ins->data_buf[11] << 8 | ins->data_buf[10])) * 0.122f;
  ins->acc_z =
      (float)((int16_t)(ins->data_buf[13] << 8 | ins->data_buf[12])) * 0.122f;

  SEGGER_RTT_printf(0,
                    "%s acc: x:%d y:%d z:%d \n"
                    " gxy :x:%d y:%d z:%d temp:%d \n",
                    RTT_CTRL_TEXT_BRIGHT_RED, (int)ins->acc_x, (int)ins->acc_y,
                    (int)ins->acc_z, (int)ins->gyr_x, (int)ins->gyr_y,
                    (int)ins->gyr_z, (int)ins->temp);

  ins->flag = LSM6DS3TR_FLAG_UPDATED;

  /* only test */
  lsm6ds3tr_acc_gry_output(ins);
  //
}

void test2() {
  uint8_t buf[1] = {0};
  uint8_t data_buf[6] = {0};
  int16_t acc_float[3] = {0};
  int16_t gry_float[3] = {0};

  uint8_t temp_buf[2] = {0};
  int16_t temp_int16 = 0;
  float temp = 0;

  volatile int8_t data_buf2[6] = {0};

  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3TR_READ,
                                          LSM6DS3TRC_STATUS_REG, 1, buf, 1, 1);
  if (buf[0] == 7) {

    /* can get value */
    st = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3TR_READ, LSM6DS3TRC_OUTX_L_XL,
                          I2C_MEMADD_SIZE_8BIT, data_buf, 6, 100);

    int8_t a;
    SEGGER_RTT_printf(0, "%d %d [%x %x]\n", INT8_MAX, sizeof(data_buf[1]),
                      data_buf[0], data_buf[1]);
    // int16_t t = (data_buf[1] << 8) | data_buf[0];
    // float tf = t * 0.122f;
    // SEGGER_RTT_printf(0, "%d ", (int)tf);

    acc_float[0] = ((int16_t)(data_buf[1] << 8) | data_buf[0]) * 0.122f;
    acc_float[1] = ((data_buf[3] << 8) | data_buf[2]) * 0.122f;
    acc_float[2] = ((data_buf[5] << 8) | data_buf[4]) * 0.122f;

    // SEGGER_RTT_printf(0, "%d\n", (int)acc_float[0]);

    // acc_float[0] = (data_buf[1] << 8);
    // acc_float[1] = (data_buf[3] << 8);
    // acc_float[2] = (data_buf[5] << 8);
    //
    // for (int i = 0; i <= 5; i++) {
    //   data_buf2[i] = data_buf[i];
    // }
    //
    // acc_float[0] = (data_buf2[1] << 8);
    // acc_float[1] = (data_buf2[3] << 8);
    // acc_float[2] = (data_buf2[5] << 8);

    st = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3TR_READ, LSM6DS3TRC_OUTX_L_G,
                          I2C_MEMADD_SIZE_8BIT, data_buf, 6, 100);

    gry_float[0] = ((data_buf[1] << 8 | data_buf[0])) * 70.00f;
    gry_float[1] = ((data_buf[3] << 8 | data_buf[2])) * 70.00f;
    gry_float[2] = ((data_buf[5] << 8 | data_buf[4])) * 70.00f;

    st = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3TR_READ, LSM6DS3TRC_OUT_TEMP_L,
                          I2C_MEMADD_SIZE_8BIT, temp_buf, 2, 100);
    temp_int16 = temp_buf[1] << 8 | temp_buf[0];
    temp = ((float)temp_int16) / 256.0 + 25.0;

    // IMUUpdate(gry_float[0]/1000,gry_float[1]/1000,gry_float[2]/1000,acc_float[0]/1000,acc_float[1]/1000,acc_float[2]/1000);
    SEGGER_RTT_printf(0,
                      "%s acc: x:%d y:%d z:%d \r\n"
                      " gxy :x:%d y:%d z:%d \r\n"
                      " temp:%dC \r\n",
                      RTT_CTRL_TEXT_BRIGHT_RED, (int)acc_float[0],
                      (int)acc_float[1], (int)acc_float[2], (int)gry_float[0],
                      (int)gry_float[1], (int)gry_float[2], (int)temp);
  } else {
    SEGGER_RTT_printf(0, "no ready \r\n", RTT_CTRL_TEXT_BRIGHT_BLUE, (int)Pitch,
                      (int)Roll, (int)Yaw);
  }
}



