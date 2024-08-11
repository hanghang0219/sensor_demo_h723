//
// Created by hanghang on 24-7-30.
//

#include "lsm6ds3tr_motionFX.h"
#include <SEGGER_RTT.h>
#include <usart.h>

#include "motion_fx.h"
static uint8_t mfxstate_6x[FX_STATE_SIZE];

void lsm6ds3trMotionFxInit(void) {
  MFX_knobs_t knobs;

  MotionFX_initialize((MFXState_t *)mfxstate_6x);
  MotionFX_getKnobs(mfxstate_6x, &knobs);

  knobs.gbias_acc_th_sc = GBIAS_ACC_TH_SC_6X;
  knobs.gbias_gyro_th_sc = GBIAS_GYRO_TH_SC_6X;
  knobs.gbias_mag_th_sc = GBIAS_MAG_TH_SC_6X;

  knobs.acc_orientation[0] = ACC_ORIENTATION_X;
  knobs.acc_orientation[1] = ACC_ORIENTATION_Y;
  knobs.acc_orientation[2] = ACC_ORIENTATION_Z;

  knobs.gyro_orientation[0] = GYR_ORIENTATION_X;
  knobs.gyro_orientation[1] = GYR_ORIENTATION_Y;
  knobs.gyro_orientation[2] = GYR_ORIENTATION_Z;

  knobs.mag_orientation[0] = MAG_ORIENTATION_X;
  knobs.mag_orientation[1] = MAG_ORIENTATION_Y;
  knobs.mag_orientation[2] = MAG_ORIENTATION_Z;

  knobs.output_type = MFX_ENGINE_OUTPUT_ENU;
  knobs.LMode = 1;
  knobs.modx = 1;

  MotionFX_setKnobs(mfxstate_6x, &knobs);
  MotionFX_enable_6X(mfxstate_6x, MFX_ENGINE_ENABLE);
  MotionFX_enable_9X(mfxstate_6x, MFX_ENGINE_DISABLE);
}

typedef struct {
  float acceleration[3];
  float angular_rate[3];
  MFX_output_t mfx_6x;
} Sensor;

void printfData(Sensor *ins) {
  uint8_t data[15];
  data[0] = 0xAB;
  data[1] = 0xDC;
  data[2] = 0xFE;
  data[3] = 0x03;
  data[4] = 0x07;
  data[5] = 0;
  data[7] = (int16_t)(ins->mfx_6x.rotation[2] * 100) >> 8;
  data[6] = (int16_t)(ins->mfx_6x.rotation[2] * 100);
  data[9] = (int16_t)(ins->mfx_6x.rotation[1] * 100) >> 8;
  data[8] = (int16_t)(ins->mfx_6x.rotation[1] * 100);
  data[11] = (int16_t)(ins->mfx_6x.rotation[0] * 100) >> 8;
  data[10] = (int16_t)(ins->mfx_6x.rotation[0] * 100);
  data[12] = 0;

  uint8_t sumcheck = 0;
  uint8_t addcheck = 0;
  for (uint16_t i = 0; i < 13; i++) {
    sumcheck += data[i];   //从帧头开始，对每一字节进行求和，直到 DATA 区结束
    addcheck += sumcheck;  //每一字节的求和操作，进行一次 sumcheck 的累加
  }
  data[13] = sumcheck;
  data[14] = addcheck;
  HAL_StatusTypeDef st = HAL_UART_Transmit(&huart1, data, 15, 10);
  if (st != HAL_OK)
    SEGGER_RTT_printf(0, "%s Uart transmit_it err %s\n", RTT_CTRL_TEXT_BRIGHT_BLUE, RTT_CTRL_RESET);
}

void lsm6ds3trMotionFxDetermin(Lsm6ds3tr *ins) {
  static uint8_t num = 0;
  Sensor sensor;

  sensor.acceleration[0] = ins->data.acc_x;
  sensor.acceleration[1] = ins->data.acc_y;
  sensor.acceleration[2] = ins->data.acc_z;

  sensor.angular_rate[0] = ins->data.gyr_x;
  sensor.angular_rate[1] = ins->data.gyr_y;
  sensor.angular_rate[2] = ins->data.gyr_z;

  MFX_input_t mfx_data_in;

  mfx_data_in.acc[0] = sensor.acceleration[0] * FROM_MG_TO_G;
  mfx_data_in.acc[1] = sensor.acceleration[1] * FROM_MG_TO_G;
  mfx_data_in.acc[2] = sensor.acceleration[2] * FROM_MG_TO_G;

  mfx_data_in.gyro[0] = sensor.angular_rate[0] * FROM_MDPS_TO_DPS;
  mfx_data_in.gyro[1] = sensor.angular_rate[1] * FROM_MDPS_TO_DPS;
  mfx_data_in.gyro[2] = sensor.angular_rate[2] * FROM_MDPS_TO_DPS;

  mfx_data_in.mag[0] = 0;
  mfx_data_in.mag[1] = 0;
  mfx_data_in.mag[2] = 0;

  float delta_time[1];
  if (ins->data.timestamp_2 > ins->data.timestamp_1) {
    delta_time[0] = (float)((double)(ins->data.timestamp_2 - ins->data.timestamp_1) * 25.0f / 1000000);

    MotionFX_propagate(mfxstate_6x, &sensor.mfx_6x, &mfx_data_in, delta_time);
    MotionFX_update(mfxstate_6x, &sensor.mfx_6x, &mfx_data_in, delta_time, NULL);
  } else if (ins->data.timestamp_1 > ins->data.timestamp_2) {
    delta_time[0] = (float)((double)(0xffffffff - ins->data.timestamp_2 + ins->data.timestamp_1) * 25.0f / 1000000);

    MotionFX_propagate(mfxstate_6x, &sensor.mfx_6x, &mfx_data_in, delta_time);
    MotionFX_update(mfxstate_6x, &sensor.mfx_6x, &mfx_data_in, delta_time, NULL);

  } else if (ins->data.timestamp_1 == ins->data.timestamp_2) {
    delta_time[0] = 0.0f;
  }
  num++;
  printfData(&sensor);
  if (num == 10) {
    SEGGER_RTT_printf(0, "%smotionFx : %d  %d  %d   %d --- %d%s \n", RTT_CTRL_TEXT_BRIGHT_BLUE,
                      (int)sensor.mfx_6x.rotation[0], (int)sensor.mfx_6x.rotation[1], (int)sensor.mfx_6x.rotation[2],
                      ins->data.timestamp_2, ins->data.timestamp_1, RTT_CTRL_RESET);

    num = 0;
  }
}
