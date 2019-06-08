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
#include "output.h"

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[6] = { 1000, 1000, 1000, 1000, 1000, 1000 };
int16_t servo[6] = { 1500, 1500, 1500, 1500, 1500, 1500 };
uint8_t s3D = 0; // 3D an = 1 aus ist 0
uint8_t NUMBER_MOTOR = 0;
uint8_t MULTITYPE = 0;
uint8_t throttleTest = 0;
int16_t Zadd = 0;
uint8_t rcOptions[CHECKBOX_ITEM_COUNT];
int16_t angle[2]    = {0,0};

static uint8_t dynP8[3], dynD8[3];
struct config conf;
struct flags_struct f;

static void checkFirstTime(uint8_t guiReset)
{
    conf.P8[ROLL] = 25;
    conf.I8[ROLL] = 15;
    conf.D8[ROLL] = 18;
    conf.P8[PITCH] = 25;
    conf.I8[PITCH] = 15;
    conf.D8[PITCH] = 18;
    conf.P8[YAW] = 65;
    conf.I8[YAW] = 30;
    conf.D8[YAW] = 0;
    conf.P8[PIDALT] = 16;
    conf.I8[PIDALT] = 15;
    conf.D8[PIDALT] = 7;

    conf.P8[PIDPOS] = 0;
    conf.I8[PIDPOS] = 0;
    conf.D8[PIDPOS] = 0;
    conf.P8[PIDPOSR] = 0;
    conf.I8[PIDPOSR] = 0;
    conf.D8[PIDPOSR] = 0;
    conf.P8[PIDNAVR] = 0;
    conf.I8[PIDNAVR] = 0;
    conf.D8[PIDNAVR] = 0;

    conf.P8[PIDLEVEL] = 70;
    conf.I8[PIDLEVEL] = 10;
    conf.D8[PIDLEVEL] = 100;
    conf.P8[PIDMAG] = 40;

    conf.P8[PIDVEL] = 0;
    conf.I8[PIDVEL] = 0;
    conf.D8[PIDVEL] = 0;

    conf.rcRate8 = 180;
    conf.rcExpo8 = 0;
    conf.rollPitchRate = 0;
    conf.yawRate = 100;
    conf.dynThrPID = 0;
    conf.thrMid8 = 50;
    conf.thrExpo8 = 0;
    for (uint8_t i = 0; i < CHECKBOX_ITEM_COUNT; i++) {
        conf.activate[i] = 0;
    }
    conf.activate[BOXARM] = 3;
    conf.angleTrim[0] = 0;
    conf.angleTrim[1] = 0;

    if (guiReset == 0) {
        conf.F3D = 1; // ESC's are set to 3D Mode
        conf.MIDDLEDEADBAND = 40; //nur f�r 3D

        conf.sOneShot = 1; //0=normaler betrieb (4xxhz) 1 ist oneshot 125

        conf.copterType = 3; //0=Bi,1=Tri,2=QUADP,3=QUADX,4=Y4,5=Y6,6=H6P,7=H6X,8=Vtail4

        conf.RxType = 2; //0StandardRX,1sat1024-DSM2,2sat2048-DSMX,3PPMGrSp,4PPMRobHiFu,5PPMHiSanOthers

        conf.MINTHROTTLE = 1070;
        conf.MAXTHROTTLE = 2000;
        conf.MINCOMMAND = 1000;
        conf.MIDRC = 992;
        conf.MINCHECK = 1100;
        conf.MAXCHECK = 1900;

        conf.BILeftMiddle = 1500;
        conf.BIRightMiddle = 1500;
        conf.TriYawMiddle = 1500;
        conf.YAW_DIRECTION = 1;
        conf.BiLeftDir = 1;
        conf.BiRightDir = 1;
        conf.DigiServo = 0;

        conf.ArmRoll = 0;  // arm und disarm �ber roll statt yaw

        conf.MPU6050_DLPF_CFG = 3; //0=256(aus),1=188,2=98,3=42,4=20,5=10,6=5

        conf.s3DMIDDLE = 1500;

        conf.calibState = 0;
    }

}

void mixerSetThrottleAngleCorrection(int16_t correctionValue)
{
    Zadd = correctionValue;
}

float rc_scale = 1000/(float)(RC_MAX - RC_MIN);
inline int16_t scaleThrottleRC(int16_t x){ // 1000 <-> 2000
    x = x - RC_MIN;
    return constrain((int16_t)((float)x * rc_scale) + 1000,1000,2000);
}

inline int16_t scaleRC(int16_t x){ // -500 <-> 500
    x = x -conf.MIDRC;
    return constrain((int16_t)((float)x * rc_scale),-500,500);
}
static void convertRCData()
{
    rcData[THROTTLE] = scaleThrottleRC(rxCh[0]);
    rcData[ROLL] = scaleRC(rxCh[1]);
    rcData[PITCH] = scaleRC(rxCh[2]);
    rcData[YAW] = scaleRC(rxCh[3]);
    rcData[AUX1] = rxCh[4];
    rcData[AUX2] = rxCh[5];

}
int main(void)
{
    /************************* Initializations ******************************/
    /*Initialize SAMD20 MCU*/
    system_init();

    /*Initialize clock module of SAMD20 MCU - Internal RC clock*/
    clock_initialize();

    // led inti
    struct port_config config_prt_pin;
    port_get_config_defaults(&config_prt_pin);
    config_prt_pin.direction = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(PIN_PA28, &config_prt_pin);
    port_pin_set_config(PIN_PB02, &config_prt_pin);
    port_pin_set_config(PIN_PA24, &config_prt_pin);

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

    if (conf.copterType == 0) { //0=Bi,1=Tri,2=QUADP,3=QUADX,4=Y4,5=Y6,6=H6P,7=H6X,8=Vtail4
        MULTITYPE = 4;
        NUMBER_MOTOR = 2;
    }
    if (conf.copterType == 1) {
        MULTITYPE = 1;
        NUMBER_MOTOR = 3;
    }
    if (conf.copterType == 2) {
        MULTITYPE = 2;
        NUMBER_MOTOR = 4;
    }
    if (conf.copterType == 3) {
        MULTITYPE = 3;
        NUMBER_MOTOR = 4;
    }
    if (conf.copterType == 4) {
        MULTITYPE = 9;
        NUMBER_MOTOR = 4;
    }
    if (conf.copterType == 5) {
        MULTITYPE = 6;
        NUMBER_MOTOR = 6;
    }
    if (conf.copterType == 6) {
        MULTITYPE = 7;
        NUMBER_MOTOR = 6;
    }
    if (conf.copterType == 7) {
        MULTITYPE = 10;
        NUMBER_MOTOR = 6;
    }
    if (conf.copterType == 8) {
        MULTITYPE = 17;
        NUMBER_MOTOR = 4;
    }

    imuConfigure(800, 0);
    imuInit();
    mspInit();

    timeMs_t currentTime = 0;
    timeMs_t controlloopTime = 0;
    timeMs_t cycleTime = 0;
    timeMs_t previousTime = 0;
    timeMs_t rcTime = 0;

    f.ARMED = 1; // TODO use RCDATA
    f.ACC_MODE = 1;

    /************************** Infinite Loop *******************************/
    while (true) {
        //mspSerialPush(MSP_API_VERSION, NULL, 0, MSP_DIRECTION_REQUEST);

        int8_t axis;
        int16_t error,errorAngle;
        int16_t PTerm=0,ITerm=0,PTermACC=0,ITermACC=0,PTermGYRO=0,ITermGYRO=0,DTerm=0;
        int16_t delta,deltaSum;
        static int16_t errorAngleI[2] = {0,0};
        static int16_t errorGyroI[3] = {0,0,0};
        static int16_t lastGyro[3] = {0,0,0};
        static int16_t delta1[3],delta2[3];

        currentTime = micros();

        /* RC INPUT DATA PROCESSING */
        if (currentTime > rcTime ) { // 50Hz
            rcTime = currentTime + RCINPUT_LOOPTIME_US;
            mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand,mspFcProcessReply);
            sbusFrameStatus();
            convertRCData();
        }

        /* CONTROL LOOP AND SENSOR PROCESSING */
        if (currentTime > controlloopTime) { // 400Hz
            controlloopTime = currentTime + MAINCONTROL_LOOPTIME_US;
            gyroUpdate(micros());
            accUpdate(&accelerometerConfigMutable()->accelerometerTrims);
            imuUpdateAttitude(micros());

            int16_t prop = 0;
            angle[ROLL]= attitude.values.roll;
            angle[PITCH]= attitude.values.pitch;
            float gyroAverage[XYZ_AXIS_COUNT];
            gyroGetAccumulationAverage(gyroAverage);

            //**** PITCH & ROLL & YAW PID ****
            for (axis = 0; axis < 3; axis++) {
                if ((f.ACC_MODE || f.HORIZON_MODE) && axis < 2) { //LEVEL MODE
                // 50 degrees max inclination
                    errorAngle = constrain(2 * rcData[axis], -500, +500)- angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
                    PTermACC = (int32_t) errorAngle * conf.P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
                    PTermACC = constrain(PTermACC, -conf.D8[PIDLEVEL] * 5,conf.D8[PIDLEVEL] * 5);
                    errorAngleI[axis] = constrain( errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp     //16 bits is ok here
                    ITermACC = ((int32_t) errorAngleI[axis] * conf.I8[PIDLEVEL]) >> 12; // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
                }
                if (!f.ACC_MODE || f.HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis
                    if (abs(rcData[axis]) < 350) {
                        error = rcData[axis] * 10 * 8 / conf.P8[axis]; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
                    } else{
                        error = (int32_t) rcData[axis] * 10 * 8/ conf.P8[axis]; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
                    }
                    error -= (int16_t) gyroAverage[axis];
                    PTermGYRO = rcData[axis];
                    errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000);      // WindUp   16 bits is ok here
                    if (abs((int16_t) gyroAverage[axis]) > 640) {
                        errorGyroI[axis] = 0;
                    }
                    ITermGYRO = (errorGyroI[axis] / 125 * conf.I8[axis]) >> 6; // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
                }
                if (f.HORIZON_MODE && axis < 2) {
                    PTerm = ((int32_t) PTermACC * (500 - prop) + (int32_t) PTermGYRO * prop) / 500;
                    ITerm = ((int32_t) ITermACC * (500 - prop) + (int32_t) ITermGYRO * prop) / 500;
                } else {
                    if (f.ACC_MODE && axis < 2) {
                        PTerm = PTermACC;
                        ITerm = ITermACC;
                    } else {
                        PTerm = PTermGYRO;
                        ITerm = ITermGYRO;
                    }
                }
                if (abs((int16_t) gyroAverage[axis]) < 160) {
                    PTerm -= ((int16_t) gyroAverage[axis]) * dynP8[axis] / 10 / 8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
                } else {
                    PTerm -= (int32_t) gyroAverage[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
                }
                delta = (int16_t) gyroAverage[axis] - lastGyro[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
                lastGyro[axis] = (int16_t) gyroAverage[axis];
                deltaSum = delta1[axis] + delta2[axis] + delta;
                delta2[axis] = delta1[axis];
                delta1[axis] = delta;

                if (abs(deltaSum) < 640)
                    DTerm = (deltaSum * dynD8[axis]) >> 5; // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result
                else
                    DTerm = ((int32_t) deltaSum * dynD8[axis]) >> 5; // 32 bits is needed for calculation

                axisPID[axis] = PTerm + ITerm - DTerm;
            }

            mixTable();

            /* estimate cycle time of control loop*/
            currentTime = micros();
            cycleTime = currentTime - previousTime;
            previousTime = currentTime;

            DEBUG_SET(DEBUG_STACK, 0, cycleTime);

            if (!throttleTest) {
                writeMotors();
            }
            port_pin_set_output_level(PIN_PA28, true);
            port_pin_set_output_level(PIN_PB02, true);
        }
    } /* !while (true) */

}

