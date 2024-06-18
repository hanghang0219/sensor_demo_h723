#ifndef _GXHTC3_H
#define _GXHTC3_H


#include "string.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"


// 如果移植程序时只要改一下三个地方就行了
/* 定时使用的IO口 */
#define GXHTC3_SCL GPIO_PIN_6   //PB6
#define GXHTC3_SDA GPIO_PIN_7   //PB7
#define GPIO_GXHTC3 GPIOB

#define GXHTC3_SCL_H HAL_GPIO_WritePin(GPIO_GXHTC3,GXHTC3_SCL,1)
#define GXHTC3_SCL_L HAL_GPIO_WritePin(GPIO_GXHTC3,GXHTC3_SCL,0)

#define GXHTC3_SDA_H HAL_GPIO_WritePin(GPIO_GXHTC3,GXHTC3_SDA,1)
#define GXHTC3_SDA_L HAL_GPIO_WritePin(GPIO_GXHTC3,GXHTC3_SDA,0)

/* 声明全局函数 */
void GXHTC3_INIT(void);
void GXHTC3_SDA_OUT(void);
void GXHTC3_SDA_IN(void);
void GXHTC3_SDA_in(void);

void GXHTC3_StarT(void);
void GXHTC3_StoP(void);
void GXHTC3_Ack(void);
void GXHTC3_NAck(void);
uint8_t GXHTC3_Wait_Ack(void);
void GXHTC3_Send_Byte(uint8_t txd);
uint8_t GXHTC3_Read_Byte(uint8_t ack);
void GXHTC3_read_result(uint8_t addr);

void al_get_gxth30_temp(void);

#endif