#include "tof.h"


#include "./../../../lib/main/SAMD20J18/drivers/vl53l0x.h"
#include "build/debug.h"
/************************************************************************/
/* Globals                                                              */
/************************************************************************/
SENSOR_DATA tofData;
VL53L0X_Error tofDevStatus;
timeUs_t tofCodeTime = 0;
timeUs_t tofLastRead = 0;
timeUs_t tofLoopTime = 0;
timeUs_t max = 0;
/************************************************************************/

void tofInit(void) {
    initVL53L0X(1);
    startContinuous(0);
}

SENSOR_OPERATION_STATUS readTofData(void) {
    tofCodeTime = micros();

    if ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
        tofData.x = readReg16Bit(RESULT_RANGE_STATUS + 10);
        writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
        tofLoopTime = micros()-tofLastRead;
        tofLastRead = micros();
    }

    tofCodeTime = micros()-tofCodeTime;

    if(tofCodeTime > max){
        max =tofCodeTime;
    }
    DEBUG_SET(DEBUG_TOF, 0,tofData.x);
    DEBUG_SET(DEBUG_TOF, 1,max);
    DEBUG_SET(DEBUG_TOF, 2,tofLoopTime/1000);

    return SENSOR_SUCCESS;
}
