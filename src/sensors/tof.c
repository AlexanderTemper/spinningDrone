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



void tofInit(void) {
    tofDev.I2cDevAddr = 0x29;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    VL53L0X_Version_t Version;
    VL53L0X_DeviceInfo_t DeviceInfo;

    DEBUG_WAIT(MODUL_DEFAULT, "Sensor init");
    tofDevStatus = VL53L0X_DataInit(&tofDev);

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "Device Info");
        tofDevStatus = VL53L0X_GetDeviceInfo(&tofDev, &DeviceInfo);
    }
    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "VL53L0X_GetDeviceInfo:\n");
        DEBUG_WAIT(MODUL_DEFAULT, "Device Name : %s\n", DeviceInfo.Name);
        DEBUG_WAIT(MODUL_DEFAULT, "Device Type : %s\n", DeviceInfo.Type);
        DEBUG_WAIT(MODUL_DEFAULT, "Device ID : %s\n", DeviceInfo.ProductId);
        DEBUG_WAIT(MODUL_DEFAULT, "ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
        DEBUG_WAIT(MODUL_DEFAULT, "ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
            DEBUG_WAIT(MODUL_DEFAULT, "Error expected cut 1.1 but found cut %d.%d\n", DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
            tofDevStatus = VL53L0X_ERROR_NOT_SUPPORTED;
        }
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "Call of VL53L0X_StaticInit\n");
        tofDevStatus = VL53L0X_StaticInit(&tofDev); // Device Initialization
        // StaticInit will set interrupt by default
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "Call of VL53L0X_PerformRefSpadManagement\n");
        tofDevStatus = VL53L0X_PerformRefSpadManagement(&tofDev, &refSpadCount, &isApertureSpads); // Device Initialization
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "Call of VL53L0X_PerformRefCalibration\n");
        tofDevStatus = VL53L0X_PerformRefCalibration(&tofDev, &VhvSettings, &PhaseCal); // Device Initialization
    }

    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "Setze Device Mode CONTINUOUS");
        tofDevStatus = VL53L0X_SetDeviceMode(&tofDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    }

    // Enable/Disable Sigma and Signal check
     if( tofDevStatus == VL53L0X_ERROR_NONE ) {
         tofDevStatus = VL53L0X_SetLimitCheckEnable( &tofDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
     }

     if( tofDevStatus == VL53L0X_ERROR_NONE ) {
         tofDevStatus = VL53L0X_SetLimitCheckEnable( &tofDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
     }

     if( tofDevStatus == VL53L0X_ERROR_NONE ) {
         tofDevStatus = VL53L0X_SetLimitCheckEnable( &tofDev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
     }

     if( tofDevStatus == VL53L0X_ERROR_NONE ) {
         tofDevStatus = VL53L0X_SetLimitCheckValue( &tofDev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );
     }


    if (tofDevStatus == VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "Starte Messungen");
        tofDevStatus = VL53L0X_StartMeasurement(&tofDev);
    }

    if (tofDevStatus != VL53L0X_ERROR_NONE) {
        DEBUG_WAIT(MODUL_DEFAULT, "fehler %d", tofDevStatus);
    }
}

SENSOR_OPERATION_STATUS readTofData(void) {
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData = &RangingMeasurementData;
    uint8_t NewDatReady = 0;

    tofDevStatus = VL53L0X_GetMeasurementDataReady(&tofDev, &NewDatReady);
    if ((NewDatReady == 0x01) && tofDevStatus == VL53L0X_ERROR_NONE) {
        tofDevStatus = VL53L0X_GetRangingMeasurementData(&tofDev, pRangingMeasurementData);

              tofData.x = pRangingMeasurementData->RangeMilliMeter;

              // Clear the interrupt
              VL53L0X_ClearInterruptMask(&tofDev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
              VL53L0X_PollingDelay(&tofDev);
              return SENSOR_SUCCESS;
    }
    //TODO SENSOR Busy
    return SENSOR_SUCCESS;
}

