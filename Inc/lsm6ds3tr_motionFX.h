// //
// // Created by hanghang on 24-7-30.
// //
//
// #ifndef LSM6DS3TR_MOTIONFX_H
// #define LSM6DS3TR_MOTIONFX_H
//
//
// typedef struct{
//   double x;
//   double y;
//   double z;
// } Accelerometer;
//
// typedef struct{
//   double x;
//   double y;
//   double z;
// } Gyroscope;
//
//
// #define FX_STATE_SIZE (size_t)(2432)
//
// #define ACC_ORIENTATION_X   'n'
// #define ACC_ORIENTATION_Y   'w'
// #define ACC_ORIENTATION_Z   'u'
//
// #define GYR_ORIENTATION_X   'n'
// #define GYR_ORIENTATION_Y   'w'
// #define GYR_ORIENTATION_Z   'u'
//
// #define MAG_ORIENTATION_X   'n'
// #define MAG_ORIENTATION_Y   'e'
// #define MAG_ORIENTATION_Z   'u'
//
// #define GBIAS_ACC_TH_SC_6X    (2.0f*0.000765f)
// #define GBIAS_GYRO_TH_SC_6X   (2.0f*0.002f)
// #define GBIAS_MAG_TH_SC_6X    (2.0f*0.001500f)
// #define GBIAS_ACC_TH_SC_9X    (2.0f*0.000765f)
// #define GBIAS_GYRO_TH_SC_9X   (2.0f*0.002f)
// #define GBIAS_MAG_TH_SC_9X    (2.0f*0.001500f)
//
// #define FROM_MG_TO_G          0.001f
// #define FROM_MDPS_TO_DPS      0.001f
//
//
// void lsm6dso_init(void);
//
//
// void lsm6dso_motion_fx_determin(void);
//
//
//
//
//
//
// #endif //LSM6DS3TR_MOTIONFX_H
