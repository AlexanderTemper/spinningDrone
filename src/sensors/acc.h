#ifndef ACC_H_
#define ACC_H_

#include "sensor.h"
#include "bma2x2_support.h"

//Normalizing Factor to 1G at +-8 with +-2^13 = 8192/8 = 1024
#define ACC_1G 1024

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} rawAccData_t;

//G per axis x,y,z
extern SENSOR_DATA accData;
extern rawAccData_t rawAccData;

void accInit(void);
SENSOR_OPERATION_STATUS readAccData(void);

#endif /* !ACC_H_ */
