#ifndef MAG_H_
#define MAG_H_

#include "sensor.h"
#include "bmm050_support.h"

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} rawMagData_t;

extern SENSOR_DATA magData;
extern rawMagData_t rawMagData;

void magInit(void);
SENSOR_OPERATION_STATUS readMagData(void);

#endif /* !MAG_H_ */
