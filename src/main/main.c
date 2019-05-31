#include "asf.h"
#include <math.h>
#include <float.h>


#include "clock_support.h"
#include "spi_support.h"
#include "tc_support.h"
#include "usart_support.h"
#include "telemetry/simbleeBridge.h"

#include "io/serial.h"
#include "msp/msp_serial.h"
#include "msp/msp_protocol.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "build/debug.h"
#include "sensors/initialisation.h"
#include "drivers/serial_uart_bmf.h"
#include "flight/imu.h"


int main(void) {
    /************************* Initializations ******************************/




    /*Initialize SAMD20 MCU*/
    system_init();

    /*Initialize clock module of SAMD20 MCU - Internal RC clock*/
    clock_initialize();


    // led inti
    struct port_config config_prt_pin;
    port_get_config_defaults(&config_prt_pin);
    config_prt_pin.direction = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(PIN_PA28,&config_prt_pin);
    port_pin_set_config(PIN_PB02,&config_prt_pin);
    port_pin_set_config(PIN_PA24,&config_prt_pin);

    /*SPI master for communicating with sensors*/
    spi_initialize();

    /*Initialize timers */
    tc_initialize();

    /*Initialize UART */
    usart_initialize();

    serialInit();
    mspSerialInit();




    /*Enable the system interrupts*/
    system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */

    //Reset Configs
    pgResetAll();
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
    timeMs_t imutimer = 0;
    timeMs_t timergyro = 0;

    uint8_t lauflicht = 0;
    /************************** Infinite Loop *******************************/
    while (true) {
    	//mspSerialPush(MSP_API_VERSION, NULL, 0, MSP_DIRECTION_REQUEST);

    	mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand, mspFcProcessReply);

    	if (cmpTimeUs(micros(),timergyro) >= 6250){
    	    timergyro = micros();
    	    gyroUpdate(micros());
    	    accUpdate(&accelerometerConfigMutable()->accelerometerTrims);
    	}


        if (cmpTimeUs(micros(),imutimer) >= 10000) {
            timeMs_t timeel = cmpTimeUs(micros(),imutimer);
            imutimer = micros();
            time = micros();
            imuUpdateAttitude(micros());
            DEBUG_SET(DEBUG_STACK, 0, cmpTimeUs(micros(),time));
            DEBUG_SET(DEBUG_STACK, 1, timeel);
            DEBUG_SET(DEBUG_STACK, 2, millis());
            DEBUG_SET(DEBUG_STACK, 3, rx_byte);
            //debugNonBlock();
            lauflicht ++;

            port_pin_set_output_level(PIN_PA28,0b1000000&lauflicht);
            port_pin_set_output_level(PIN_PB02,0b0100000&lauflicht);
            port_pin_set_output_level(PIN_PA24,0b0010000&lauflicht);
        }

    } /* !while (true) */

}


