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
#include "fc/runtime_config.h"

#include "drivers/accgyro/gyro_bmg160.h"

#define LED_ROT PIN_PA24
#define LED_GELB PIN_PB02
#define LED_GRUEN PIN_PA28

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[MAX_MOTORS];
uint8_t s3D = 0; // 3D an = 1 aus ist 0
uint8_t NUMBER_MOTOR = 0;
uint8_t MULTITYPE = 0;
uint8_t throttleTest = 0;
int16_t Zadd, heading, headFreeModeHold = 0;
uint8_t dynP8[3], dynI8[3], dynD8[3];
int16_t rcCommand[4]; // interval [1000;2000] for THROTTLE and [-500;+500] for

#define PITCH_LOOKUP_LENGTH 7
int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH]; // lookup table for expo & RC rate
timeUs_t cycleTime = 0; // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static int32_t errorGyroI[3] = {
        0,
        0,
        0 };
static int32_t errorAngleI[2] = {
        0,
        0 };

uint32_t mst = 0;
bool led2 = false;
struct config conf;
struct flags_struct f;

uint16_t msp_rc_timeout = 0;

static void checkFirstTime(uint8_t guiReset)
{
    conf.P8[ROLL] = 120;
    conf.I8[ROLL] = 60;
    conf.D8[ROLL] = 30;
    conf.P8[PITCH] = 120;
    conf.I8[PITCH] = 60;
    conf.D8[PITCH] = 30;
    conf.P8[YAW] = 100;
    conf.I8[YAW] = 50;
    conf.D8[YAW] = 25;
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
    conf.P8[PIDLEVEL] = 60;
    conf.I8[PIDLEVEL] = 45;
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
        conf.MINTHROTTLE = 1020;
        conf.MAXTHROTTLE = 2000;
        conf.MINCOMMAND = 1000;
        conf.MIDRC = 1500;
        conf.MINCHECK = 1000;
        conf.MAXCHECK = 1900;
        conf.YAW_DIRECTION = 1;

        conf.F3D = 0; // ESC's are set to 3D Mode
        conf.MIDDLEDEADBAND = 40; //nur für 3D
        conf.deadband3d_high = conf.MIDRC + conf.MIDDLEDEADBAND;
        conf.deadband3d_low = conf.MIDRC - conf.MIDDLEDEADBAND;

        conf.ArmRoll = 0;  // arm und disarm über roll statt yaw
        conf.s3DMIDDLE = 1500;
        conf.calibState = 0;

        conf.deadband = 20;
        conf.yawdeadband = 20;
    }

}

void mixerSetThrottleAngleCorrection(int16_t correctionValue)
{
    Zadd = correctionValue;
}

float rc_scale = 1000 / (float) (RC_MAX - RC_MIN);
inline int16_t scaleRC(int16_t x)
{ // 1000 <-> 2000
    x = x - RC_MIN;
    return constrain((int16_t )((float )x * rc_scale) + 1000, 1000, 2000);
}

static void convertRCData()
{
    sbustimeout++; // Count each time Called and no RC Update

    if (rxCh[15] > 500 && sbustimeout < 10) { // RSSI
        rcData[THROTTLE] = scaleRC(rxCh[0]);
        rcData[ROLL] = scaleRC(rxCh[1]);
        rcData[PITCH] = scaleRC(rxCh[2]);
        rcData[YAW] = scaleRC(rxCh[3]);
        rcData[AUX1] = rxCh[4];
        rcData[AUX2] = rxCh[5];
    } else {
        rcData[THROTTLE] = 1000;
        rcData[ROLL] = conf.MIDRC;
        rcData[PITCH] = conf.MIDRC;
        rcData[YAW] = conf.MIDRC;
        rcData[AUX1] = 0;
        rcData[AUX2] = 0;
    }
}

#define ARME_FORCE 0
static void handleFlags()
{
    if (rcData[AUX1] > 1000) {
        if (f.ARMED == 0) {
            headFreeModeHold = heading;
        }
        f.ARMED = 1;
        ENABLE_ARMING_FLAG(ARMED);
    } else {
        f.ARMED = 0;
        DISABLE_ARMING_FLAG(ARMED);
    }

    if (rcData[AUX2] < 500) {
        f.ANGLE_MODE = 1;
        enableFlightMode(ANGLE_MODE);
    } else {
        f.ANGLE_MODE = 0;
        disableFlightMode(ANGLE_MODE);
    }

    if (ARME_FORCE) {
        f.ARMED = 1;
        ENABLE_ARMING_FLAG(ARMED);
    }

    // perform actions
    if (!f.ARMED || rcData[THROTTLE] < 1100) {
        errorGyroI[ROLL] = 0;
        errorGyroI[PITCH] = 0;
        errorGyroI[YAW] = 0;
        errorAngleI[ROLL] = 0;
        errorAngleI[PITCH] = 0;
    }

    port_pin_set_output_level(LED_GRUEN, f.ARMED);
}
#define DEBUG_PID 0

static void pidRewrite(void)
{
    int32_t errorAngle = 0;
    int axis;
    int32_t delta, deltaSum;
    static int32_t delta1[3], delta2[3];
    int32_t PTerm, ITerm, DTerm;
    static int32_t lastError[3] = {
            0,
            0,
            0 };
    int32_t AngleRateTmp, RateError;

    int16_t angle[2] = {
            0,
            0 };
    angle[ROLL] = attitude.values.roll;
    angle[PITCH] = attitude.values.pitch;
    int16_t gyroData[3];
    gyroData[0] = lrintf(gyro.gyroADCf[0]);
    gyroData[1] = lrintf(gyro.gyroADCf[1]);
    gyroData[2] = lrintf(gyro.gyroADCf[2]);

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        if (axis == 2) { // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            AngleRateTmp = (((int32_t) (conf.yawRate + 27) * rcCommand[YAW]) >> 5);
        } else {
            // calculate error and limit the angle to 50 degrees max inclination
            errorAngle = (constrain(rcCommand[axis], -500, +500) - angle[axis] + conf.angleTrim[axis]) / 10.0f; // 16 bits is ok here
            if (!f.ANGLE_MODE) { //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                AngleRateTmp = ((int32_t) (conf.rollPitchRate[axis] + 27) * rcCommand[axis]) >> 4;

                if (f.HORIZON_MODE) {
                    // mix up angle error to desired AngleRateTmp to add a little auto-level feel
                    AngleRateTmp += (errorAngle * conf.I8[PIDLEVEL]) >> 8;
                }
            } else { // it's the ANGLE mode - control is angle based, so control loop is needed
                AngleRateTmp = (errorAngle * conf.P8[PIDLEVEL]) >> 4;
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRateTmp - gyroData[axis];

        // -----calculate P component
        PTerm = (RateError * conf.P8[axis]) >> 7;
        // -----calculate I component
        // there should be no division before accumulating the error to integrator, because the precision would be reduced.
        // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        // Time correction (to avoid different I scaling for different builds based on average cycle time)
        // is normalized to cycle time = 2048.
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * (int32_t) cycleTime) >> 11) * conf.I8[axis];

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], -2097152, +2097152);
        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        delta = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = (delta * ((uint16_t) 0xFFFF / (cycleTime >> 4))) >> 6;
        // add moving average here to reduce noise
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * conf.D8[axis]) >> 8;

        // -----calculate total PID output
        if (DEBUG_PID && axis == 0) {
            DEBUG_SET(DEBUG_STACK, 0, PTerm);
            DEBUG_SET(DEBUG_STACK, 1, ITerm);
            DEBUG_SET(DEBUG_STACK, 2, RateError);
            DEBUG_SET(DEBUG_STACK, 3, errorAngle);
        }

        axisPID[axis] = PTerm + ITerm + DTerm;
    }
}
static void annexCode(void)
{
    int32_t tmp;
    int32_t axis;

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - conf.MIDRC), 500);
        if (axis != 2) {        // ROLL & PITCH
            rcCommand[axis] = tmp;
        } else {                // YAW
            rcCommand[axis] = tmp * -conf.YAW_DIRECTION;
        }
        if (rcData[axis] < conf.MIDRC) {
            rcCommand[axis] = -rcCommand[axis];
        }
    }
    rcCommand[THROTTLE] = constrain(rcData[THROTTLE], conf.MINCHECK, 2000);
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
    port_pin_set_config(LED_ROT, &config_prt_pin);
    port_pin_set_config(LED_GELB, &config_prt_pin);
    port_pin_set_config(LED_GRUEN, &config_prt_pin);

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
    mixerInit();
    initOutput();

    imuConfigure(800, 0);
    imuInit();
    mspInit();

    timeUs_t currentTime = 0;
    timeUs_t controlloopTime = 0;
    timeUs_t previousTime = 0;
    timeUs_t rcTime = 0;
    timeUs_t mspTime = 0;
    timeUs_t ledTime = 0;

    timeUs_t rcCodeTime = 0;
    timeUs_t imuCodeTime = 0;
    timeUs_t restCodeTime = 0;

    /************************** Infinite Loop *******************************/
    while (true) {
        //mspSerialPush(MSP_API_VERSION, NULL, 0, MSP_DIRECTION_REQUEST);
        currentTime = micros();


        /* MSP DATA */
        if (currentTime > mspTime) { // 200Hz
            mspTime = currentTime + MSP_LOOPTIME_US;
            mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand, mspFcProcessReply);
            //rcCodeTime = micros();
            msp_rc_timeout++;
            if (msp_rc_timeout > 10) {
                rcData[THROTTLE] = 1000;
                rcData[ROLL] = conf.MIDRC;
                rcData[PITCH] = conf.MIDRC;
                rcData[YAW] = conf.MIDRC;
                rcData[AUX1] = 0;
                rcData[AUX2] = 0;
            }
            //sbusFrameStatus();
            //convertRCData();
            //DEBUG_SET(DEBUG_STACK, 3, micros() - rcCodeTime);

        }
        /* RC INPUT DATA PROCESSING */
        //if (currentTime > rcTime) { // 50Hz
            //rcCodeTime = micros();
            //sbusFrameStatus();
            //convertRCData();
            //DEBUG_SET(DEBUG_STACK, 3, micros() - rcCodeTime);
        //}

        /* CONTROL LOOP AND SENSOR PROCESSING */
        if (currentTime > controlloopTime) {
            controlloopTime = currentTime + MAINCONTROL_LOOPTIME_US;
            // Imu Update
            imuCodeTime = micros();
            gyroUpdate(micros());
            accUpdate(&accelerometerConfigMutable()->accelerometerTrims);

            handleFlags(); // IMPORTANT

            // Wait for Gyro to callibrate
            port_pin_set_output_level(LED_GELB, isGyroCalibrationComplete());
            if (isGyroCalibrationComplete()) {
                imuUpdateAttitude(micros());
                heading = (int16_t) (attitude.values.yaw / 10.0f);
                //DEBUG_SET(DEBUG_ALTITUDE, 0, heading);
                restCodeTime = micros();
                // Measure loop rate just afer reading the sensors
                currentTime = micros();
                cycleTime = currentTime - previousTime;
                previousTime = currentTime;

                annexCode();
                pidRewrite();
                //pidMultiWii();

            }

            mixTable();
            writeMotors();

            //DEBUG_SET(DEBUG_STACK, 0, axisPID[YAW]);
            //DEBUG_SET(DEBUG_STACK, 1, axisPID[PITCH]);
            //DEBUG_SET(DEBUG_STACK, 2, axisPID[ROLL]);

            //DEBUG_SET(DEBUG_STACK, 2, micros()- restCodeTime);
        }

        if (currentTime > ledTime) {
            ledTime = currentTime + 100000; // 100ms
            if (mst++ == 10) {
                led2 = !led2;
                port_pin_set_output_level(LED_ROT, led2);
                mst = 0;
            }

            //port_pin_set_output_level(PIN_PA28, true);
            //port_pin_set_output_level(PIN_PB02, true);
        }

    } /* !while (true) */

}
