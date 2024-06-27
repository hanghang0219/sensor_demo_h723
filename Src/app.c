
#include "cmsis_os2.h"
#include "lps22hb.h"


const osThreadAttr_t main_attributes = {
        .name = "main",
        .stack_size = 1024 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t thread_attributes = {
        .name = "main",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

Lps22hb lps;

__NO_RETURN void mainThread(void* arg) {
  readLps22hbIdReg(&lps);
  while (1) {
    if (lps.flag == LPS22HB_FLAG_END && lps.err_st == LPS22HB_OK) {
      readLps22hbIdReg(&lps);
    }
    osThreadYield();
  }
}


__NO_RETURN void threadThread(void* arg) {


}

void lpsRxCallback(I2C_HandleTypeDef* hi2c) {
  if (lps.flag == LPS22HB_FLAG_ID) {
    if (lps.data_id == LPS22HB_ID)
      writeLps22hbOnceReg(&lps);
    else
      lps.err_st = LPS22HB_ID_ERR;
  } else if (lps.flag == LPS22HB_FLAG_DATA) {
    lps22hbParser(&lps);
  }
}

void lpsTxCallback(I2C_HandleTypeDef* hi2c) {
  readLps22hbPressureReg(&lps);
}

void ZMI2CLps22hbInit(Lps22hb* ins) {
  HAL_I2C_RegisterCallback(ins->i2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID, lpsRxCallback);
  HAL_I2C_RegisterCallback(ins->i2c, HAL_I2C_MEM_TX_COMPLETE_CB_ID, lpsTxCallback);
}


void MX_FREERTOS_Init(void) {
  lps22hbInit(&lps, &hi2c1);
  ZMI2CLps22hbInit(&lps);


  osThreadNew(mainThread, NULL, &main_attributes);
//  osThreadNew(threadThread, NULL, &thread_attributes);
}



