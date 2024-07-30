// //
// // Created by hanghang on 24-7-30.
// //
//
// #include "lsm6ds3tr_motionFX.h"
//
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include "motion_fx.h"
//
// #define DELATE_TIME ((double)(0.0022))  //0.0025
//
// static int16_t data_raw_acceleration[3];
// static int16_t data_raw_angular_rate[3];
//
// static uint8_t mfxstate_6x[FX_STATE_SIZE];
// static float Quaternions_data[4];
//
// typedef struct {
//   Accelerometer acc;
//   Gyroscope gyr;
//   float acceleration[3];
//   float angular_rate[3];
//   MFX_output_t mfx_6x;
// } sensor_hub_data_t;
//
// sensor_hub_data_t sensor_hub_data;
//
// void MFX_Arithmetic_Init(void);
//
// void lsm6dso_init(void) {
//   MFX_Arithmetic_Init();
// }
//
// int ii = 0;
//
// void lsm6dso_motion_fx_determin(float acc_x, float acc_y, float acc_z,
//                                 float gyr_x, float gyr_y, float gyr_z,
//                                 uint32_t deltatime_1, uint32_t deltatime_2) {
//
//   sensor_hub_data.acceleration[0] = acc_x;
//   sensor_hub_data.acceleration[1] = acc_y;
//   sensor_hub_data.acceleration[2] = acc_z;
//
//   sensor_hub_data.angular_rate[0] = gyr_x;
//   sensor_hub_data.angular_rate[1] = gyr_y;
//   sensor_hub_data.angular_rate[2] = gyr_z;
//
//   /*----------------------------------------------------------------------------------
//
//
// 		----------------------------------------------------------------------------------*/
//   MFX_input_t mfx_data_in;
//
//   mfx_data_in.acc[0] = sensor_hub_data.acceleration[0] * FROM_MG_TO_G;
//   mfx_data_in.acc[1] = sensor_hub_data.acceleration[1] * FROM_MG_TO_G;
//   mfx_data_in.acc[2] = sensor_hub_data.acceleration[2] * FROM_MG_TO_G;
//
//   mfx_data_in.gyro[0] = sensor_hub_data.angular_rate[0] * FROM_MDPS_TO_DPS;
//   mfx_data_in.gyro[1] = sensor_hub_data.angular_rate[1] * FROM_MDPS_TO_DPS;
//   mfx_data_in.gyro[2] = sensor_hub_data.angular_rate[2] * FROM_MDPS_TO_DPS;
//
//   mfx_data_in.mag[0] = 0;
//   mfx_data_in.mag[1] = 0;
//   mfx_data_in.mag[2] = 0;
//
//   //	printf("Acceleration [mg]:\t%4.2f \t%4.2f \t%4.2f\r\n",mfx_data_in.acc[0],
//   //																		mfx_data_in.acc[1], mfx_data_in.acc[2]);
//
//   //		float delta_time = DELATE_TIME;
//   float delta_time[1];
//   if (deltatime_2 > deltatime_1) {
//     delta_time[0] =
//         (float)((double)(deltatime_2 - deltatime_1) * 25.0f / 1000000);
//     //		printf("d=%f\n",delta_time[0]);
//
//     MotionFX_propagate(mfxstate_6x, &sensor_hub_data.mfx_6x, &mfx_data_in,
//                        delta_time);
//
//     MotionFX_update(mfxstate_6x, &sensor_hub_data.mfx_6x, &mfx_data_in,
//                     delta_time, NULL);
//   } else if (deltatime_1 > deltatime_2) {
//     delta_time[0] = (float)((double)(0xffffffff - deltatime_2 + deltatime_1) *
//                             25.0f / 1000000);
//
//     MotionFX_propagate(mfxstate_6x, &sensor_hub_data.mfx_6x, &mfx_data_in,
//                        delta_time);
//
//     MotionFX_update(mfxstate_6x, &sensor_hub_data.mfx_6x, &mfx_data_in,
//                     delta_time, NULL);
//   } else if (deltatime_1 == deltatime_2) {
//     delta_time[0] = 0.0f;
//   }
//
//   //	MotionFX_propagate(mfxstate_6x, &sensor_hub_data.mfx_6x, &mfx_data_in, &delta_time);
//
//   //	MotionFX_update(mfxstate_6x, &sensor_hub_data.mfx_6x, &mfx_data_in, &delta_time, NULL);
//
//   //	Quaternions_data[0] = sensor_hub_data.mfx_6x.quaternion[0];
//   //	Quaternions_data[1] = sensor_hub_data.mfx_6x.quaternion[1];
//   //	Quaternions_data[2] = sensor_hub_data.mfx_6x.quaternion[2];
//   //	Quaternions_data[3] = sensor_hub_data.mfx_6x.quaternion[3];
//
//   //	printf("%f, %f, %f, %f \n",Quaternions_data[3],\
// 						Quaternions_data[1],Quaternions_data[2],Quaternions_data[0]);
//   if (ii == 10) {
//     ii = 0;
//     printf("\n%f, %f, %f", sensor_hub_data.mfx_6x.rotation[0],
//            sensor_hub_data.mfx_6x.rotation[1],
//            sensor_hub_data.mfx_6x.rotation[2]);
//   } else
//     ii++;
// }
//
// void MFX_Arithmetic_Init(void) {
//   MFX_knobs_t iKnobs;
//   MFX_knobs_t *ipKnobs = &iKnobs;
//
//   MotionFX_initialize((MFXState_t *)mfxstate_6x);
//
//   MotionFX_getKnobs(mfxstate_6x, ipKnobs);
//
//   ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC_6X;
//   ipKnobs->gbias_gyro_th_sc = GBIAS_ACC_TH_SC_6X;
//   ipKnobs->gbias_mag_th_sc = GBIAS_ACC_TH_SC_6X;
//
//   ipKnobs->acc_orientation[0] = ACC_ORIENTATION_X;
//   ipKnobs->acc_orientation[1] = ACC_ORIENTATION_Y;
//   ipKnobs->acc_orientation[2] = ACC_ORIENTATION_Z;
//
//   ipKnobs->gyro_orientation[0] = GYR_ORIENTATION_X;
//   ipKnobs->gyro_orientation[1] = GYR_ORIENTATION_Y;
//   ipKnobs->gyro_orientation[2] = GYR_ORIENTATION_Z;
//
//   ipKnobs->mag_orientation[0] = MAG_ORIENTATION_X;
//   ipKnobs->mag_orientation[1] = MAG_ORIENTATION_Y;
//   ipKnobs->mag_orientation[2] = MAG_ORIENTATION_Z;
//
//   ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
//   ipKnobs->LMode = 1;
//   ipKnobs->modx = 1;
//
//   MotionFX_setKnobs(mfxstate_6x, ipKnobs);
//
//   MotionFX_enable_6X(mfxstate_6x, MFX_ENGINE_ENABLE);
//
//   MotionFX_enable_9X(mfxstate_6x, MFX_ENGINE_DISABLE);
// }
