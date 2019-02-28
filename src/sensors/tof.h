#ifndef _TOF_H_
#define _TOF_H_

#include "sensor.h"
#include "vl53l0x_api.h"
#include "debug.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} rawTofData_t;

//G per axis x,y,z
extern SENSOR_DATA tofData;
extern rawTofData_t rawTofData;
extern VL53L0X_Dev_t tofDev;


SENSOR_OPERATION_STATUS tofInit(void);
SENSOR_OPERATION_STATUS readTofData(void);

#endif /* !_TOF_H_ */
