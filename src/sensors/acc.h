#ifndef ACC_H_
#define ACC_H_

#include "sensor.h"
#include "bma2x2_support.h"

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}accData;


void accInit(void);
SENSOR_OPERATION_STATUS readAccData(accData *data);

#endif /* !ACC_H_ */
