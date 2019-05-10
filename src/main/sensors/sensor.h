#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensors/sensors.h"
#include "fc/runtime_config.h"

#define SENSOR_OPERATION_STATUS int8_t
#define SENSOR_SUCCESS 0
#define SENSOR_ERROR -1
#define SENSOR_FORMAT_FLOAT 0
#define SENSOR_FORMAT_32S 1

typedef struct {
    float x;
    float y;
    float z;
} sensorDataFloat_t;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} sensorData32s_t;

#define SENSOR_DATA sensorDataFloat_t

void sensorFloatToS32(sensorData32s_t *sensors32, sensorDataFloat_t *sensorFloat, float scale);

#endif /* !SENSOR_H_ */
