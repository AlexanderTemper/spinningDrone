#include "gyro.h"

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
SENSOR_DATA gyroData;
rawGyroData_t rawGyroData;
/************************************************************************/

void rawToGyroData(struct bmg160_data_t *raw);

void gyroInit(void) {
    bmg_init();
    bmg160_set_range_reg(0x00);
    bmg160_set_bw(C_BMG160_BW_116HZ_U8X);
    bmg160_set_power_mode(BMG160_MODE_NORMAL);
    sensorsSet(SENSOR_GYRO);
}

SENSOR_OPERATION_STATUS readGyroData(void) {
    struct bmg160_data_t rawData;
    if (bmg160_get_data_XYZ(&rawData) != SENSOR_SUCCESS) {
        return SENSOR_ERROR;
    }
    rawToGyroData(&rawData);
    return SENSOR_SUCCESS;
}

void rawToGyroData(struct bmg160_data_t *raw) {
    rawGyroData.x = raw->datax;
    rawGyroData.x = raw->datay;
    rawGyroData.z = raw->dataz;

    gyroData.x = ((float) raw->datax) / SCALIN_RAD;
    gyroData.y = ((float) raw->datay) / SCALIN_RAD;
    gyroData.z = ((float) raw->dataz) / SCALIN_RAD;

}

