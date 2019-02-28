#include "tof.h"

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
SENSOR_DATA tofData;
VL53L0X_Error tofDevStatus;
VL53L0X_Dev_t tofDev;

/************************************************************************/
VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady = 0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 1

SENSOR_OPERATION_STATUS tofInit(void) {
    tofDev.I2cDevAddr = 0x29;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    DEBUG_WAIT(MODUL_TOF, "Sensor init");
    tofDevStatus = VL53L0X_DataInit(&tofDev);

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_TOF, "Call of VL53L0X_StaticInit\n");
        tofDevStatus = VL53L0X_StaticInit(&tofDev); // Device Initialization
        // StaticInit will set interrupt by default
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_TOF, "Call of VL53L0X_PerformRefSpadManagement\n");
        tofDevStatus = VL53L0X_PerformRefSpadManagement(&tofDev, &refSpadCount, &isApertureSpads); // Device Initialization
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_TOF, "Call of VL53L0X_PerformRefCalibration\n");
        tofDevStatus = VL53L0X_PerformRefCalibration(&tofDev, &VhvSettings, &PhaseCal); // Device Initialization
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_TOF, "set Device Mode CONTINUOUS");
        tofDevStatus = VL53L0X_SetDeviceMode(&tofDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    }

    // Enable/Disable Sigma and Signal check

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_SetLimitCheckEnable(&tofDev,
        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_SetLimitCheckEnable(&tofDev,
        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_SetLimitCheckValue(&tofDev,
        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t) (0.1 * 65536));
    }
    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_SetLimitCheckValue(&tofDev,
        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (60 * 65536));
    }
    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&tofDev, 33000);
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_SetVcselPulsePeriod(&tofDev,
        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_SetVcselPulsePeriod(&tofDev,
        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_TOF, "start measurement");
        tofDevStatus = VL53L0X_StartMeasurement(&tofDev);
    }

    if (tofDevStatus != VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "TOF Error %d", tofDevStatus);
        return SENSOR_ERROR;
    }

    return SENSOR_SUCCESS;
}

SENSOR_OPERATION_STATUS readTofData(void) {
   /* VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData = &RangingMeasurementData;
    uint8_t NewDatReady = 0;*/
    uint8_t Byte = 0;
    VL53L0X_RdByte(&tofDev,VL53L0X_REG_RESULT_INTERRUPT_STATUS, &Byte);

    if((Byte & 0x07) != 0){

        uint16_t data;
        VL53L0X_RdWord(&tofDev, VL53L0X_REG_RESULT_RANGE_STATUS + 10, &data);
        tofData.x = data;
        VL53L0X_WrByte(&tofDev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    }
   /* tofDevStatus = VL53L0X_GetMeasurementDataReady(&tofDev, &NewDatReady);
    if ((NewDatReady == 0x01) && tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_GetRangingMeasurementData(&tofDev, pRangingMeasurementData);

        tofData.x = pRangingMeasurementData->RangeMilliMeter;

        // Clear the interrupt
        VL53L0X_ClearInterruptMask(&tofDev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        VL53L0X_PollingDelay(&tofDev);
        return SENSOR_SUCCESS;
    }
    //TODO SENSOR Busy*/
    return SENSOR_SUCCESS;
}

