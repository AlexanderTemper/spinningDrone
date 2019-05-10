#include "sensor.h"

void sensorFloatToS32(sensorData32s_t *sensors32, sensorDataFloat_t *sensorFloat, float scale) {
    sensors32->x = (int32_t) ((sensorFloat->x) * scale);
    sensors32->y = (int32_t) ((sensorFloat->y) * scale);
    sensors32->z = (int32_t) ((sensorFloat->z) * scale);
}
