#include "gxhtc3.h"
#include "delay_tim.h"
#include "cmsis_os2.h"

#define write 0
#define read  1
float GXHTC3_temp, GXHTC3_humi, GXHTC3_Temperature, GXHTC3_Humidity;

/*
* @name   CRC_8
* @brief  CRC-8校验
* @param  Crc_ptr -> 校验数据首地址
		  LEN     -> 校验数据长度
* @retval CRC_Value -> 校验值
*/
static uint8_t CRC_8(uint8_t* Crc_ptr, uint8_t LEN) {
  uint8_t CRC_Value = 0xFF;
  uint8_t i = 0, j = 0;

  for (i = 0; i < LEN; i++) {
    CRC_Value ^= *(Crc_ptr + i);
    for (j = 0; j < 8; j++) {
      if (CRC_Value & 0x80)
        CRC_Value = (CRC_Value << 1) ^ 0x31;
      else
        CRC_Value = (CRC_Value << 1);
    }
  }
  return CRC_Value;
}


/****************************************************************************
* Function Name  : GXHTC3_INIT
* Description    : 初始化GPIO.
****************************************************************************/
void GXHTC3_INIT() {
//  GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//  GPIO_InitStructure.GPIO_Pin = GXHTC3_SDA;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//  GPIO_InitStructure.GPIO_Pin = GXHTC3_SCL;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GXHTC3_SCL_H;
  GXHTC3_SDA_H;
}

/*******************************************************************************
* 函 数 名         : GXHTC3_SDA_OUT
* 函数功能		   : SDA输出配置
*******************************************************************************/
void GXHTC3_SDA_OUT() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GXHTC3_SDA;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*******************************************************************************
* 函 数 名         : GXHTC3_SDA_IN
* 函数功能		   : SDA输入配置
*******************************************************************************/
void GXHTC3_SDA_IN(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GXHTC3_SDA;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void GXHTC3_SDA_in(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GXHTC3_SDA;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*******************************************************************************
* 函 数 名         : GXHTC3_StarT
* 函数功能		   : 产生起始信号
*******************************************************************************/
void GXHTC3_StarT(void) {
  GXHTC3_SDA_OUT();

  GXHTC3_SDA_H;
  GXHTC3_SCL_H;
  delay_us(5);
  GXHTC3_SDA_L;
  delay_us(6);
  GXHTC3_SCL_L;
}

/*******************************************************************************
* 函 数 名         : GXHTC3_StoP
* 函数功能		   : 产生停止信号
*******************************************************************************/
void GXHTC3_StoP(void) {
  GXHTC3_SDA_OUT();

  GXHTC3_SCL_L;
  GXHTC3_SDA_L;
  GXHTC3_SCL_H;
  delay_us(6);
  GXHTC3_SDA_H;
  delay_us(6);
}

/*******************************************************************************
* 函 数 名         : GXHTC3_Ack
* 函数功能		   : 主机产生应答信号ACK
*******************************************************************************/
void GXHTC3_Ack(void) {
  GXHTC3_SCL_L;
  GXHTC3_SDA_OUT();
  GXHTC3_SDA_L;
  delay_us(2);
  GXHTC3_SCL_H;
  delay_us(5);
  GXHTC3_SCL_L;
}

/*******************************************************************************
* 函 数 名         : GXHTC3_NAck
* 函数功能		   : 主机不产生应答信号NACK
*******************************************************************************/
void GXHTC3_NAck(void) {
  GXHTC3_SCL_L;
  GXHTC3_SDA_OUT();
  GXHTC3_SDA_H;
  delay_us(2);
  GXHTC3_SCL_H;
  delay_us(5);
  GXHTC3_SCL_L;
}

/*******************************************************************************
* 函 数 名         : GXHTC3_Wait_Ack
* 函数功能		   : 等待从机应答信号
  返回值：            1 接收应答失败
		  	         0 接收应答成功
*******************************************************************************/
uint8_t GXHTC3_Wait_Ack(void) {
  uint8_t tempTime = 0;
  GXHTC3_SDA_IN();
  GXHTC3_SDA_H;
  delay_us(1);
  GXHTC3_SCL_H;
  delay_us(1);

  while (HAL_GPIO_ReadPin(GPIO_GXHTC3, GXHTC3_SDA)) {
    tempTime++;
    delay_us(1);
    if (tempTime > 250) {
      GXHTC3_StoP();
      return 1;
    }
  }
  GXHTC3_SCL_L;
  delay_us(1);
  return 0;
}

/*******************************************************************************
* 函 数 名         : GXHTC3_Send_Byte
* 函数功能		   : GXHTC3 发送一个字节
*******************************************************************************/
void GXHTC3_Send_Byte(uint8_t txd) {
  uint8_t i = 0;
  GXHTC3_SDA_OUT();
  GXHTC3_SCL_L;//拉低时钟开始数据传输

  for (i = 0; i < 8; i++) {
    if ((txd & 0x80) > 0) //0x80  1000 0000
      GXHTC3_SDA_H;
    else
      GXHTC3_SDA_L;

    txd <<= 1;
    delay_us(1);
    GXHTC3_SCL_H;
    delay_us(2); //发送数据
    GXHTC3_SCL_L;
    delay_us(2);
  }
}

/*******************************************************************************
* 函 数 名         : GXHTC3_Read_Byte
* 函数功能		   : GXHTC3 主机读取一个字节
*******************************************************************************/
uint8_t GXHTC3_Read_Byte(uint8_t ack) {
  uint8_t i = 0, receive = 0;

  GXHTC3_SDA_in();
  for (i = 0; i < 8; i++) {
    GXHTC3_SCL_L;
    delay_us(2);
    GXHTC3_SCL_H;
    while (!HAL_GPIO_ReadPin(GPIO_GXHTC3, GXHTC3_SCL));
    receive <<= 1;
    if (HAL_GPIO_ReadPin(GPIO_GXHTC3, GXHTC3_SDA))
      receive++;
    delay_us(1);
  }

  if (ack == 0)
    GXHTC3_NAck();
  else
    GXHTC3_Ack();

  return receive;
}

/*******************************************************************************
* 函 数 名         : GXHTC3_read_result
* 函数功能		   : GXHTC3 读6个字节数据
*******************************************************************************/
void GXHTC3_read_result(uint8_t addr) {
  uint16_t tem, hum;
  unsigned char buff[6];

  float Temperature = 0;
  float Humidity = 0;

  GXHTC3_StarT();
  GXHTC3_Send_Byte(addr << 1 | write);//写7位GXHTC3设备地址加0作为写取位,1为读取位
  GXHTC3_Wait_Ack();
  GXHTC3_Send_Byte(0x78);
  GXHTC3_Wait_Ack();
  GXHTC3_Send_Byte(0x66);
  GXHTC3_Wait_Ack();
  GXHTC3_StoP();

  osDelay(15);                    //数据转换等待时间

  GXHTC3_StarT();
  GXHTC3_Send_Byte(addr << 1 | read);//写7位GXHTC3设备地址加0作为写取位,1为读取位

  if (GXHTC3_Wait_Ack() == 0) {
    GXHTC3_SDA_in();

    buff[0] = GXHTC3_Read_Byte(1);
    buff[1] = GXHTC3_Read_Byte(1);
    buff[2] = GXHTC3_Read_Byte(1);
    buff[3] = GXHTC3_Read_Byte(1);
    buff[4] = GXHTC3_Read_Byte(1);
    buff[5] = GXHTC3_Read_Byte(0);
    GXHTC3_StoP();

    if (CRC_8(buff, 2) == buff[2] && CRC_8(buff + 3, 2) == buff[5]) {
      tem = ((buff[0] << 8) | buff[1]);//温度拼接
      hum = ((buff[3] << 8) | buff[4]);//湿度拼接
      /*转换实际温度*/
      Temperature = (175.0 * (float) tem / 65535.0 - 45.0);// T = -45 + 175 * tem / (2^16-1)
      Humidity = (100.0 * (float) hum / 65535.0);// RH = hum*100 / (2^16-1)
    } else {
      GXHTC3_temp = 0;
      GXHTC3_humi = 0;
    }

  }

  if ((Temperature >= -20) && (Temperature <= 125) && (Humidity >= 0) && (Humidity <= 100))//过滤错误数据
  {
    GXHTC3_temp = Temperature;
    GXHTC3_humi = Humidity;
  }

  tem = 0;
  hum = 0;
}

/*******************************************************************************
* 函 数 名         : al_float_buffer_sort
* 函数功能		   : 多次读数值排序取中间值平均
*******************************************************************************/
void al_float_buffer_sort(float* buf, uint8_t length) {
  uint8_t i, j;
  float tmp;
  for (i = 0; i < length; i++) {
    for (j = i + 1; j < length; j++) {
      if (buf[j] < buf[i]) {
        tmp = buf[j];
        buf[j] = buf[i];
        buf[i] = tmp;
      }
    }
  }
}


void al_get_gxth30_temp(void) {
  float buff_temp[20], buff_humi[20];
  for (uint8_t i = 0; i < 10; i++) {
    GXHTC3_read_result(0x70);
    buff_temp[i] = GXHTC3_temp;
    buff_humi[i] = GXHTC3_humi;
  }

  al_float_buffer_sort(buff_temp, 10);
  al_float_buffer_sort(buff_humi, 10);

  GXHTC3_Temperature = (buff_temp[4] + buff_temp[5]) / 2;
  GXHTC3_Humidity = (buff_humi[4] + buff_humi[5]) / 2;

//  printf("温度：%0.2f, 湿度：%0.2f", GXHTC3_Temperature, GXHTC3_Humidity);
}
