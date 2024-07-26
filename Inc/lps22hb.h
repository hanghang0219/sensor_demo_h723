//
// Created by hanghang on 20/06/2024.
//

#ifndef H723_SSR_LPS22HB_H
#define H723_SSR_LPS22HB_H


#include "i2c.h"

// sad 0 (ground)
#define LPS22HB_SAD_READ            0xB8
#define LPS22HB_SAD_WRITE           0xB9

// register add
#define LPS22HB_REGISTER_ADDR_ID            0x0F              // device id
#define LPS22HB_REGISTER_ADDR_CTRL_REG2     0x11              // control register 2
#define LPS22HB_REGISTER_ADDR_PRESS_OUT_XL  0x28
#define LPS22HB_REGISTER_ADDR_PRESS_OUT_L   0x29
#define LPS22HB_REGISTER_ADDR_PRESS_OUT_H   0x2A

#define LPS22HB_ID                      0xB1
#define LPS22HB_ONCE                    0X11                   //ONE_SHOT:1(0)   IF_ADD_INC:1(4) addr increase

#define LPS22HB_FLAG_ID                 0x00
#define LPS22HB_FLAG_DATA               0X01
#define LPS22HB_FLAG_END                0X02


typedef enum {
  LPS22HB_OK,
  LPS22HB_READ_ERR,
  LPS22HB_WRITE_ERR,
  LPS22HB_ID_ERR,
  LPS22HB_ERR
} Lps22hbErr;


typedef struct {
  I2C_HandleTypeDef* i2c;
  Lps22hbErr err_st;
  uint8_t data_id;
  uint8_t data_air_pressure[3];
  uint8_t flag;
  float air_pressure;
} Lps22hb;

void lps22hbInit(Lps22hb* ins, I2C_HandleTypeDef* i2c);
void readLps22hbIdReg(Lps22hb* ins);
void readLps22hbPressureReg(Lps22hb* ins);
void writeLps22hbOnceReg(Lps22hb* ins);
void lps22hbParser(Lps22hb* ins);

#endif //H723_SSR_LPS22HB_H
