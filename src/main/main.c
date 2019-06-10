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
int16_t motor[MAX_MOTORS];
uint8_t s3D = 0; // 3D an = 1 aus ist 0
uint8_t NUMBER_MOTOR = 0;
uint8_t MULTITYPE = 0;
uint8_t throttleTest = 0;
int16_t Zadd = 0;
uint8_t dynP8[3], dynI8[3], dynD8[3];
int16_t rcCommand[4]; // interval [1000;2000] for THROTTLE and [-500;+500] for
#define THROTTLE_LOOKUP_LENGTH 12
int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];
#define PITCH_LOOKUP_LENGTH 7
int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH]; // lookup table for expo & RC rate


struct config conf;
struct flags_struct f;

static void checkFirstTime(uint8_t guiReset)
{
    conf.P8[ROLL] = 40;
    conf.I8[ROLL] = 30;
    conf.D8[ROLL] = 23;
    conf.P8[PITCH] = 40;
    conf.I8[PITCH] = 30;
    conf.D8[PITCH] = 23;
    conf.P8[YAW] = 85;
    conf.I8[YAW] = 45;
    conf.D8[YAW] = 0;
    conf.P8[PIDALT] = 50;
    conf.I8[PIDALT] = 0;
    conf.D8[PIDALT] = 0;
    conf.P8[PIDPOS] = 11; // POSHOLD_P * 100;
    conf.I8[PIDPOS] = 0; // POSHOLD_I * 100;
    conf.D8[PIDPOS] = 0;
    conf.P8[PIDPOSR] = 20; // POSHOLD_RATE_P * 10;
    conf.I8[PIDPOSR] = 8; // POSHOLD_RATE_I * 100;
    conf.D8[PIDPOSR] = 45; // POSHOLD_RATE_D * 1000;
    conf.P8[PIDNAVR] = 14; // NAV_P * 10;
    conf.I8[PIDNAVR] = 20; // NAV_I * 100;
    conf.D8[PIDNAVR] = 80; // NAV_D * 1000;
    conf.P8[PIDLEVEL] = 90;
    conf.I8[PIDLEVEL] = 10;
    conf.D8[PIDLEVEL] = 100;
    conf.P8[PIDMAG] = 40;
    conf.P8[PIDVEL] = 120;
    conf.I8[PIDVEL] = 45;
    conf.D8[PIDVEL] = 1;
    conf.rcRate8 = 90;
    conf.rcExpo8 = 65;
    conf.yawRate = 0;
    conf.dynThrPID = 0;
    conf.thrMid8 = 50;
    conf.thrExpo8 = 0;
    conf.max_angle_inclination = 500; // 50 degrees

    for (uint8_t i = 0; i < CHECKBOX_ITEM_COUNT; i++) {
        conf.activate[i] = 0;
    }
    conf.activate[BOXARM] = 3;
    conf.angleTrim[0] = 0;
    conf.angleTrim[1] = 0;

    if (guiReset == 0) {
        conf.sOneShot = 1; //0=normaler betrieb (4xxhz) 1 ist oneshot 125

        conf.copterType = 3; //0=Bi,1=Tri,2=QUADP,3=QUADX,4=Y4,5=Y6,6=H6P,7=H6X,8=Vtail4

        conf.RxType = 2; //0StandardRX,1sat1024-DSM2,2sat2048-DSMX,3PPMGrSp,4PPMRobHiFu,5PPMHiSanOthers

        conf.tpa_breakpoint = 1500;
        conf.MINTHROTTLE = 1180;
        conf.MAXTHROTTLE = 1400;
        conf.MINCOMMAND = 1000;
        conf.MIDRC = 1500;
        conf.MINCHECK = 1100;
        conf.MAXCHECK = 1900;
        conf.YAW_DIRECTION = 1;

        conf.F3D = 0; // ESC's are set to 3D Mode
        conf.MIDDLEDEADBAND = 40; //nur für 3D
        conf.deadband3d_high = conf.MIDRC + conf.MIDDLEDEADBAND;
        conf.deadband3d_low = conf.MIDRC - conf.MIDDLEDEADBAND;

        conf.ArmRoll = 0;  // arm und disarm über roll statt yaw
        conf.s3DMIDDLE = 1500;
        conf.calibState = 0;

        conf.deadband = 40;
        conf.yawdeadband = 40;
    }

}

void mixerSetThrottleAngleCorrection(int16_t correctionValue)
{
    Zadd = correctionValue;
}

float rc_scale = 1000/(float)(RC_MAX - RC_MIN);
inline int16_t scaleRC(int16_t x){ // 1000 <-> 2000
    x = x - RC_MIN;
    return constrain((int16_t)((float)x * rc_scale) + 1000,1000,2000);
}
static void convertRCData()
{
    if(rxCh[15]>500) { // RSSI
        rcData[THROTTLE] = scaleRC(rxCh[0]);
        rcData[ROLL] = scaleRC(rxCh[1]);
        rcData[PITCH] = scaleRC(rxCh[2]);
        rcData[YAW] = scaleRC(rxCh[3]);
        rcData[AUX1] = rxCh[4];
        rcData[AUX2] = rxCh[5];
    } else {
        rcData[THROTTLE] = conf.MINCHECK;
        rcData[ROLL] = conf.MIDRC;
        rcData[PITCH] = conf.MIDRC;
        rcData[YAW] = conf.MIDRC;
        rcData[AUX1] = 0;
        rcData[AUX2] = 0;
    }
}


static int32_t errorGyroI[3] = { 0, 0, 0 };
static int32_t errorAngleI[2] = { 0, 0 };

static void pidMultiWii(void)
{
    int axis, prop;
    int32_t error, errorAngle;
    int32_t PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int32_t delta1[3], delta2[3];
    int32_t deltaSum;
    int32_t delta;

    int16_t angle[2] = { 0, 0 };
    angle[ROLL]= attitude.values.roll;
    angle[PITCH]= attitude.values.pitch;
    float gyroAverage[XYZ_AXIS_COUNT];
    gyroGetAccumulationAverage(gyroAverage);
    int16_t gyroData[3];
    gyroData[0] = gyroAverage[0];
    gyroData[1] = gyroAverage[1];
    gyroData[2] = gyroAverage[2];

    // **** PITCH & ROLL & YAW PID ****
    prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]
    for (axis = 0; axis < 3; axis++) {
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2) { // MODE relying on ACC
            // 50 degrees max inclination
            errorAngle = constrain(2 * rcCommand[axis], -((int)conf.max_angle_inclination), +conf.max_angle_inclination) - angle[axis] + conf.angleTrim[axis];

            PTermACC = errorAngle * conf.P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
            PTermACC = constrain(PTermACC, -conf.D8[PIDLEVEL] * 5, +conf.D8[PIDLEVEL] * 5);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * conf.I8[PIDLEVEL]) >> 12;
        }
        if (!f.ANGLE_MODE || f.HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis
            error = (int32_t)rcCommand[axis] * 10 * 8 / conf.P8[axis];
            error -= gyroData[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if ((abs(gyroData[axis]) > 640) || ((axis == YAW) && (abs(rcCommand[axis]) > 100)))
                errorGyroI[axis] = 0;
            ITermGYRO = (errorGyroI[axis] / 125 * conf.I8[axis]) >> 6;
        }
        if (f.HORIZON_MODE && axis < 2) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } else {
            if (f.ANGLE_MODE && axis < 2) {
                PTerm = PTermACC;
                ITerm = ITermACC;
            } else {
                PTerm = PTermGYRO;
                ITerm = ITermGYRO;
            }
        }

        PTerm -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
        delta = gyroData[axis] - lastGyro[axis];
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * dynD8[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}

static void activateConfig(void)
{
    uint8_t i;
    for (i = 0; i < PITCH_LOOKUP_LENGTH; i++) {
        lookupPitchRollRC[i] = (2500 + conf.rcExpo8 * (i * i - 25)) * i * (int32_t)conf.rcRate8 / 2500;
    }

    for (i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - conf.thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - conf.thrMid8;
        if (tmp < 0)
            y = conf.thrMid8;
        lookupThrottleRC[i] = 10 * conf.thrMid8 + tmp * (100 - conf.thrExpo8 + (int32_t)conf.thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = conf.MINTHROTTLE + (int32_t)(conf.MAXTHROTTLE - conf.MINTHROTTLE) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}
static void annexCode(void)
{
    int32_t tmp, tmp2;
    int32_t axis, prop1, prop2;

    // Baseflight original dynamic PID adjustemnt
    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < conf.tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)conf.dynThrPID * (rcData[THROTTLE] - conf.tpa_breakpoint) / (2000 - conf.tpa_breakpoint);
        } else {
            prop2 = 100 - conf.dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - conf.MIDRC), 500);
        if (axis != 2) {        // ROLL & PITCH
            if (conf.deadband) {
                if (tmp > conf.deadband) {
                    tmp -= conf.deadband;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t)conf.rollPitchRate[axis] * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
        } else {                // YAW
            if (conf.yawdeadband) {
                if (tmp > conf.yawdeadband) {
                    tmp -= conf.yawdeadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp * -conf.YAW_DIRECTION;
            prop1 = 100 - (uint16_t)conf.yawRate * abs(tmp) / 500;
        }
        dynP8[axis] = (uint16_t)conf.P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)conf.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)conf.D8[axis] * prop1 / 100;
        if (rcData[axis] < conf.MIDRC)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], conf.MINCHECK, 2000);
    tmp = (uint32_t)(tmp - conf.MINCHECK) * 1000 / (2000 - conf.MINCHECK);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    /*if (f.HEADFREE_MODE) {
            float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
            float cosDiff = cosf(radDiff);
            float sinDiff = sinf(radDiff);
            int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
            rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
            rcCommand[PITCH] = rcCommand_PITCH;
    }*/
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

    // Multiwii Stuff
    checkFirstTime(0);
    if (conf.copterType == 3) {
        MULTITYPE = 3;
        NUMBER_MOTOR = 4;
    }
    activateConfig();
    mixerInit();
    initOutput();

    imuConfigure(800, 0);
    imuInit();
    mspInit();

    timeMs_t currentTime = 0;
    timeMs_t controlloopTime = 0;
    timeMs_t cycleTime = 0;
    timeMs_t previousTime = 0;
    timeMs_t rcTime = 0;
    timeMs_t ledTime = 0;

    timeMs_t rcCodeTime = 0;
    timeMs_t imuCodeTime = 0;
    timeMs_t restCodeTime = 0;


    f.ARMED = 1; // TODO use RCDATA
    f.ANGLE_MODE = 1;

    /************************** Infinite Loop *******************************/
    while (true) {
        //mspSerialPush(MSP_API_VERSION, NULL, 0, MSP_DIRECTION_REQUEST);
        currentTime = micros();

        /* RC INPUT DATA PROCESSING */
        if (currentTime > rcTime ) { // 50Hz
            rcTime = currentTime + RCINPUT_LOOPTIME_US;
            mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand,mspFcProcessReply);
            rcCodeTime = micros();
                sbusFrameStatus();
                convertRCData();
            //DEBUG_SET(DEBUG_STACK, 3, micros() - rcCodeTime);
        }

        /* CONTROL LOOP AND SENSOR PROCESSING */
        if (currentTime > controlloopTime) { // 400Hz
            controlloopTime = currentTime + MAINCONTROL_LOOPTIME_US;
            // Imu Update
            imuCodeTime = micros();
                gyroUpdate(micros());
                accUpdate(&accelerometerConfigMutable()->accelerometerTrims);
                imuUpdateAttitude(micros());
            //DEBUG_SET(DEBUG_STACK, 1, micros()- imuCodeTime);

            restCodeTime = micros();
                // Measure loop rate just afer reading the sensors
                currentTime = micros();
                cycleTime = currentTime - previousTime;
                previousTime = currentTime;

                //DEBUG_SET(DEBUG_STACK, 0, cycleTime);

                annexCode();
                pidMultiWii();

                //DEBUG_SET(DEBUG_STACK, 0, axisPID[YAW]);
                //DEBUG_SET(DEBUG_STACK, 1, axisPID[PITCH]);
                //DEBUG_SET(DEBUG_STACK, 2, axisPID[ROLL]);
                //DEBUG_SET(DEBUG_STACK, 3, rcCommand[3]);
                mixTable();
                writeMotors();
            //DEBUG_SET(DEBUG_STACK, 2, micros()- restCodeTime);

        }

        if(currentTime > ledTime){
            ledTime = currentTime + 100000; // 100ms
            static uint32_t mst = 0;
            if(mst++ == 10){
                static bool led2 = false;
                led2 = !led2;
                port_pin_set_output_level(PIN_PA24, led2);
                mst = 0;
            }

            //port_pin_set_output_level(PIN_PA28, true);
            //port_pin_set_output_level(PIN_PB02, true);
        }

    } /* !while (true) */

}
