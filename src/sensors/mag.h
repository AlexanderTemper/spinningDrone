#ifndef MAG_H_
#define MAG_H_

#include "sensor.h"
#include "bmm050_support.h"

typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
}magData;


void magInit(void);
SENSOR_OPERATION_STATUS readMagData(magData *data);



#endif /* !MAG_H_ */
