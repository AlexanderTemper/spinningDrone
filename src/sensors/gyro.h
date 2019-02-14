#ifndef GYRO_H_
#define GYRO_H_

#include "sensor.h"
#include "bmg160_support.h"

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}gyroData;


void gyroInit(void);
SENSOR_OPERATION_STATUS readGyroData(gyroData *data);


#endif /* !GYRO_H_ */
