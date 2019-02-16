#ifndef GYRO_H_
#define GYRO_H_

#include "sensor.h"
#include "bmg160_support.h"

#define SCALIN_RAD 938.7054037f
#define SCALIN_GRAD 16.3835f

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} rawGyroData_t;

//rad/s per axis x,y,z
extern SENSOR_DATA gyroData;
extern rawGyroData_t rawGyroData;

void gyroInit(void);
SENSOR_OPERATION_STATUS readGyroData(void);

#endif /* !GYRO_H_ */
