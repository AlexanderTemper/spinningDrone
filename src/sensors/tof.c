#include "tof.h"


/************************************************************************/
/* Globals                                                              */
/************************************************************************/
SENSOR_DATA tofData;
rawTofData_t rawTofData;
/************************************************************************/

VL53L0X_Dev_t tofDev;
void tofInit(void) {
    tofDev.I2cDevAddr = 0x29;
    VL53L0X_DataInit(&tofDev);
}

SENSOR_OPERATION_STATUS readTofData(void) {

    return SENSOR_SUCCESS;
}


