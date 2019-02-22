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

    if ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
        tofData.x = readReg16Bit(RESULT_RANGE_STATUS + 10);
        writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    }

    return SENSOR_SUCCESS;
}

