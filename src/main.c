#include "asf.h"
#include <math.h>
#include <float.h>
#include "sensor.h"
#include "imu.h"
#include "tof.h"
#include "tof.h"

#include "clock_support.h"
#include "spi_support.h"
#include "i2c_support.h"
#include "tc_support.h"
#include "usart_support.h"
#include "simbleeBridge.h"
#include "simbleeBridge.h"
#include "io/serial.h"
#include "msp/msp_serial.h"
#include "msp/msp_protocol.h"
/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/
static void errorwait(enum status_code status){
    if(status != STATUS_OK){
        DEBUG_WAIT(MODUL_DEFAULT, "Fail code 0x%x",status);
        while(1);
    }
}
/*! Sensors data are read in accordance with TC6 callback. */
#define READ_SENSORS_FLAG				tc6_callback_flag

int main(void) {
    /********************* Initialize global variables **********************/

    uint16_t timer = 0;
    uint8_t status = STATUS_OK;

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
    status = i2c_initialize();
    errorwait(status);


    /*Enable the system interrupts*/
    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */

    /* Initialize the sensors */
    gyroInit();
    accInit();
    magInit();
    tofInit();

    timeMs_t time = 0;
    /************************** Infinite Loop *******************************/
    while (true) {
    	//mspSerialPush(MSP_API_VERSION, NULL, 0, MSP_DIRECTION_REQUEST);

    	mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand, mspFcProcessReply);

    	//DEBUG_WAIT(MODUL_DEFAULT, "Bin Da %d",timeing.imuLoop)
        /* Print sensor data periodically regarding TC6 interrupt flag (Default Period 10 ms)*/
       /* if (READ_SENSORS_FLAG) {

            time = getTimeMs();
            readAccData();
            readGyroData();
            readMagData();
            readTofData();

            updateAtt();


            // 10MS * 10 == 100ms
            if (timer > 10) {
                timeing.imuLoop = cmpTimeMs(getTimeMs(),time);
                timeing.total = getTimeMs();
                sendData();
                timer = 0;
            } else {
                timer++;
            }


            READ_SENSORS_FLAG = false;
        }*/

    } /* !while (true) */

}


