#include "asf.h"
#include <math.h>
#include <float.h>


#include "clock_support.h"
#include "spi_support.h"
#include "i2c_support.h"
#include "tc_support.h"
#include "usart_support.h"
//#include "simbleeBridge.h"

#include "io/serial.h"
#include "msp/msp_serial.h"
#include "msp/msp_protocol.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "build/debug.h"
#include "sensors/initialisation.h"
#include "drivers/serial_uart_bmf.h"
#include "flight/imu.h"

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/
/*! Sensors data are read in accordance with TC6 callback. */
#define READ_SENSORS_FLAG				tc6_callback_flag

int main(void) {
    /********************* Initialize global variables **********************/

    uint16_t timer = 0;
    //uint8_t status = STATUS_OK;

    /************************* Initializations ******************************/

    /*Initialize SAMD20 MCU*/
    system_init();

    /*Initialize clock module of SAMD20 MCU - Internal RC clock*/
    clock_initialize();

    /*SPI master for communicating with sensors*/
    spi_initialize();

    /*Initialize timers */
    tc_initialize();

    /*Initialize UART */
    usart_initialize();

    serialInit();
    mspSerialInit();

    /*Initialize I2C for communication*/
    //i2c_initialize();


    /*Enable the system interrupts*/
    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */

    /* Initialize the sensors */
    //gyroInit();
    //acclegacyInit();
    sensorsAutodetect();
    accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    gyroInitFilters();
    gyroStartCalibration(false);
    //magInit();

    imuConfigure(800, 0);
    imuInit();
    mspInit();

    timeMs_t time = 0;
    /************************** Infinite Loop *******************************/
    while (true) {
    	//mspSerialPush(MSP_API_VERSION, NULL, 0, MSP_DIRECTION_REQUEST);

    	mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand, mspFcProcessReply);
    	//DEBUG_WAIT(MODUL_DEFAULT, "Bin Da %d",timeing.imuLoop)
        /* Print sensor data periodically regarding TC6 interrupt flag (Default Period 10 ms)*/
        if (READ_SENSORS_FLAG) {

            time = getTimeMs();
            //readAccData();
        	gyroUpdate(micros());
        	accUpdate(&accelerometerConfigMutable()->accelerometerTrims);
            //readGyroData();
            //readMagData();
            //readTofData();

            imuUpdateAttitude(micros());

            DEBUG_SET(DEBUG_STACK, 0, cmpTimeMs(getTimeMs(),time));
            // 10MS * 10 == 100ms
            if (timer > 10) {
                //timeing.imuLoop = cmpTimeMs(getTimeMs(),time);
                //timeing.total = getTimeMs();
                //sendData();
                timer = 0;
            } else {
                timer++;
            }


            READ_SENSORS_FLAG = false;
        }

    } /* !while (true) */

}


