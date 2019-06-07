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
#include "rx/rx.h"
#include "rx/sbus.h"
#include "fc/rc_modes.h"
#include "globals.h"

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[6] = {1000,1000,1000,1000,1000,1000};
int16_t servo[6] = {1500,1500,1500,1500,1500,1500};
uint8_t s3D            = 0; // 3D an = 1 aus ist 0
uint8_t NUMBER_MOTOR   = 0;
uint8_t MULTITYPE = 0;
uint8_t throttleTest   = 0;
int16_t Zadd = 0;
uint8_t  rcOptions[CHECKBOX_ITEM_COUNT];

struct config conf;

static void checkFirstTime(uint8_t guiReset) {
    conf.P8[ROLL]  = 25;  conf.I8[ROLL] = 15; conf.D8[ROLL]  = 18;
    conf.P8[PITCH] = 25; conf.I8[PITCH] = 15; conf.D8[PITCH] = 18;
    conf.P8[YAW]   = 65;  conf.I8[YAW]  = 30;  conf.D8[YAW]  = 0;
    conf.P8[PIDALT]   = 16; conf.I8[PIDALT]   = 15; conf.D8[PIDALT]   = 7;

    conf.P8[PIDPOS]  = 0;     conf.I8[PIDPOS]    = 0;       conf.D8[PIDPOS]    = 0;
    conf.P8[PIDPOSR] = 0; conf.I8[PIDPOSR]   = 0;  conf.D8[PIDPOSR]   = 0;
    conf.P8[PIDNAVR] = 0;          conf.I8[PIDNAVR]   = 0;           conf.D8[PIDNAVR]   = 0;

    conf.P8[PIDLEVEL] = 70; conf.I8[PIDLEVEL] = 10; conf.D8[PIDLEVEL] = 100;
    conf.P8[PIDMAG] = 40;

    conf.P8[PIDVEL] = 0;  conf.I8[PIDVEL] = 0;  conf.D8[PIDVEL] = 0;

    conf.rcRate8 = 180; conf.rcExpo8 = 0;
    conf.rollPitchRate = 0;
    conf.yawRate = 100;
    conf.dynThrPID = 0;
    conf.thrMid8 = 50; conf.thrExpo8 = 0;
    for(uint8_t i=0;i<CHECKBOX_ITEM_COUNT;i++) {conf.activate[i] = 0;}
    conf.activate[BOXARM]=3;
    conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;

    if(guiReset == 0){
        conf.F3D            = 1; // ESC's are set to 3D Mode
        conf.MIDDLEDEADBAND = 40; //nur f�r 3D

        conf.sOneShot       = 1; //0=normaler betrieb (4xxhz) 1 ist oneshot 125

        conf.copterType     = 3; //0=Bi,1=Tri,2=QUADP,3=QUADX,4=Y4,5=Y6,6=H6P,7=H6X,8=Vtail4

        conf.RxType         = 2; //0StandardRX,1sat1024-DSM2,2sat2048-DSMX,3PPMGrSp,4PPMRobHiFu,5PPMHiSanOthers

        conf.MINTHROTTLE   = 1070;
        conf.MAXTHROTTLE   = 2000;
        conf.MINCOMMAND    = 1000;
        conf.MIDRC         = 1500;
        conf.MINCHECK      = 1100;
        conf.MAXCHECK      = 1900;

        conf.BILeftMiddle  = 1500;
        conf.BIRightMiddle = 1500;
        conf.TriYawMiddle  = 1500;
        conf.YAW_DIRECTION = 1;
        conf.BiLeftDir     = 1;
        conf.BiRightDir    = 1;
        conf.DigiServo     = 0;

        conf.ArmRoll       = 0;  // arm und disarm �ber roll statt yaw

        conf.MPU6050_DLPF_CFG = 3; //0=256(aus),1=188,2=98,3=42,4=20,5=10,6=5

        conf.s3DMIDDLE    = 1500;

        conf.calibState   = 0;
    }

}


mixerSetThrottleAngleCorrection(int16_t correctionValue){

}

static void convertRCData(){
    rcData[THROTTLE] = rxCh[0];
    rcData[ROLL] = rxCh[3];
    rcData[PITCH] = rxCh[2];
    rcData[YAW] = rxCh[1];
    rcData[AUX1] = rxCh[4];
    rcData[AUX2] = rxCh[5];

}
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
    sensorsAutodetect();
    accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    gyroInitFilters();
    gyroStartCalibration(false);
    checkFirstTime(0);

    if(conf.copterType == 0){//0=Bi,1=Tri,2=QUADP,3=QUADX,4=Y4,5=Y6,6=H6P,7=H6X,8=Vtail4
            MULTITYPE      = 4;
            NUMBER_MOTOR   = 2;
        }
        if(conf.copterType == 1){
            MULTITYPE      = 1;
            NUMBER_MOTOR   = 3;
        }
        if(conf.copterType == 2){
            MULTITYPE      = 2;
            NUMBER_MOTOR   = 4;
        }
        if(conf.copterType == 3){
            MULTITYPE      = 3;
            NUMBER_MOTOR   = 4;
        }
        if(conf.copterType == 4){
            MULTITYPE      = 9;
            NUMBER_MOTOR   = 4;
        }
        if(conf.copterType == 5){
            MULTITYPE      = 6;
            NUMBER_MOTOR   = 6;
        }
        if(conf.copterType == 6){
            MULTITYPE      = 7;
            NUMBER_MOTOR   = 6;
        }
        if(conf.copterType == 7){
            MULTITYPE      = 10;
            NUMBER_MOTOR   = 6;
        }
        if(conf.copterType == 8){
            MULTITYPE      = 17;
            NUMBER_MOTOR   = 4;
        }



    imuConfigure(800, 0);
    imuInit();
    mspInit();

    timeMs_t legacytime = 0;
    timeMs_t imutimer = 0;
    timeMs_t timergyro = 0;

    uint8_t lauflicht = 0;

    /************************** Infinite Loop *******************************/
    while (true) {
    	//mspSerialPush(MSP_API_VERSION, NULL, 0, MSP_DIRECTION_REQUEST);

    	mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand, mspFcProcessReply);
    	sbusFrameStatus();
    	convertRCData();
    	if (cmpTimeUs(micros(),timergyro) >= 6250){
    	    timergyro = micros();
    	    gyroUpdate(micros());
    	    accUpdate(&accelerometerConfigMutable()->accelerometerTrims);
    	}


        if (cmpTimeUs(micros(),imutimer) >= 10000) {
            timeMs_t timeel = cmpTimeUs(micros(),imutimer);
            imutimer = micros();
            legacytime = micros();
            imuUpdateAttitude(micros());
            DEBUG_SET(DEBUG_STACK, 0, cmpTimeUs(micros(),legacytime));
            DEBUG_SET(DEBUG_STACK, 1, timeel);
            DEBUG_SET(DEBUG_STACK, 2, millis());
            DEBUG_SET(DEBUG_STACK, 3, rx_byte);
            //debugNonBlock();
            lauflicht ++;

            port_pin_set_output_level(PIN_PA28,0b1000000&lauflicht);
            port_pin_set_output_level(PIN_PB02,0b0100000&lauflicht);
        }

    } /* !while (true) */

}


