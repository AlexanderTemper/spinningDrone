#include "asf.h"
#include <math.h>
#include <float.h>
#include "sensor.h"
#include "imu.h"
#include "tof.h"

#include "clock_support.h"
#include "spi_support.h"
#include "i2c_support.h"
#include "tc_support.h"
#include "usart_support.h"
#include "simbleeBridge.h"
#include "simbleeBridge.h"

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! Sensors data are read in accordance with TC6 callback. */
#define READ_SENSORS_FLAG				tc6_callback_flag


int main(void) {
    /********************* Initialize global variables **********************/


    uint16_t timer = 0;

    /************************* Initializations ******************************/

    /*Initialize SAMD20 MCU*/
    system_init();

    /*Initialize clock module of SAMD20 MCU - Internal RC clock*/
    clock_initialize();

    /*SPI master for communicating with sensors*/
    spi_initialize();

    /*Initialize timers */
    tc_initialize();

    /*Initialize UART for communication with PC*/
    usart_initialize();

    /*Initialize I2C for communication*/
    i2c_initialize();

    /*Enable the system interrupts*/
    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */

    /* Initialize the sensors */
    gyroInit();
    accInit();
    magInit();
    tofInit();

    //initVL53L0X(false);
    //setTimeout(400);
    //startContinuous(0);
    /************************** Infinite Loop *******************************/
    while (true) {
        /* Print sensor data periodically regarding TC6 interrupt flag (Default Period 10 ms)*/
        if (READ_SENSORS_FLAG) {

            readAccData();
            readGyroData();
            readMagData();

            updateAtt();



            // 10MS * 10 == 100ms
            if (timer > 10) {
                sendData();
                timer = 0;
            } else {
                timer++;
            }

            /* Reset TC flag */
            READ_SENSORS_FLAG = false;
        }

    } /* !while (true) */

}

