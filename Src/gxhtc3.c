//
// Created by hanghang on 18/06/2024.
//
#include "gxhtc3.h"
#include "cmsis_os.h"


HAL_StatusTypeDef initGxhtc3(Gxhtc3* handler) {


  return HAL_I2C_Mem_Write_IT(handler->i2c, GXHTC3_ADD,);
}
