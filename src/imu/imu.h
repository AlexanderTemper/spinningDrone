#ifndef IMU_H_
#define IMU_H_

#include "MahonyAHRS.h"
#include "gyro.h"
#include "acc.h"
#include "mag.h"
#include <math.h>


typedef struct {
  float x;
  float y;
  float z;
} imuSensor;


typedef struct {
	imuSensor g;
	imuSensor a;
	imuSensor m;
} imuData;


typedef struct {
  float roll;
  float pitch;
  float yaw;
} attitude_t;

void updateAtt(attitude_t *att,imuData* data, accData *acc, gyroData *gyro, magData *mag);

#endif /* !IMU_H_ */
