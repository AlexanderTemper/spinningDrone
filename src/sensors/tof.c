#include "tof.h"

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
SENSOR_DATA tofData;
VL53L0X_Error tofDevStatus;


/************************************************************************/


void tofInit(void) {
    initVL53L0X(1);
    startContinuous(0);

}

SENSOR_OPERATION_STATUS readTofData(void) {
    statInfo_t stats;
    tofData.x = readRangeContinuousMillimeters(&stats);
    return SENSOR_SUCCESS;
}

