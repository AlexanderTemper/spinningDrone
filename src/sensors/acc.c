#include "acc.h"

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
SENSOR_DATA accData;
rawAccData_t rawAccData;
/************************************************************************/

void rawToRawAccData(struct bma2x2_accel_data *raw);

void acclegacyInit(void) {
    /* Initialize BMA280 */
    bma_init();
    bma2x2_set_range(BMA2x2_RANGE_8G);
    bma2x2_set_bw(BMA2x2_BW_500HZ);
    bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
    sensorsSet(SENSOR_ACC);
}

SENSOR_OPERATION_STATUS readAccData(void) {
    struct bma2x2_accel_data rawData;
    if (bma2x2_read_accel_xyz(&rawData) != SENSOR_SUCCESS) {
        return SENSOR_ERROR;
    }
    rawToRawAccData(&rawData);
    return SENSOR_SUCCESS;
}

void rawToRawAccData(struct bma2x2_accel_data *raw) {
    rawAccData.x = raw->x;
    rawAccData.y = raw->y;
    rawAccData.z = raw->z;

    accData.x = (float) raw->x / ACC_1G;
    accData.y = (float) raw->y / ACC_1G;
    accData.z = (float) raw->z / ACC_1G;

}

