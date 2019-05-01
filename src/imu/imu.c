#include "imu.h"

#include "sensors/acceleration.h"
/************************************************************************/
/* Globals                                                              */
/************************************************************************/
attitudeEulerAngles_t attitude = EULER_INITIALIZE;

/************************************************************************/

void getMahAttitude(void);
int16_t yaw_offset(int16_t yaw);
float imuCalcKpGain(void);

bool speedUp = true;

float accAverage[XYZ_AXIS_COUNT];

void updateAtt(void) {

    twoKp = imuCalcKpGain();

    accGetAccumulationAverage(accAverage);

    MahonyAHRSupdate(gyroData.x, gyroData.y, gyroData.z, accAverage[X], accAverage[Y], accAverage[Z], magData.x, magData.y, magData.z);
    getMahAttitude();

    if (attitude.values.yaw < 0) {
        attitude.values.yaw += 3600;
    }

    if (!speedUp) {
       attitude.values.yaw = yaw_offset(attitude.values.yaw);
    }

}

/**
 * Speed up Convergence with high KP for the first 100
 */
float imuCalcKpGain(void) {
    static uint8_t counter = 0;

    if (speedUp) {
        counter++;
        if (counter == 100) {
            speedUp = false;
            counter = 0;
        }
        return 100.0f;
    }

    return 5.0f;
}
#define M_PIf_h 1.57079632679489661923f
void getMahAttitude(void) {

    float q2q2 = q2 * q2;

    attitude.values.roll  = (int16_t) (radiansToDegrees(atan2_approx(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2q2)) * 10);
    attitude.values.pitch = (int16_t) (radiansToDegrees(M_PIf_h - acos_approx(2 * (q0 * q2 - q1 * q3))) * 10);
    attitude.values.yaw   = (int16_t) (radiansToDegrees(atan2_approx(q1*q2 + q0*q3, 0.5f - q2q2 - q3 * q3)) * 10);

    /*attp->roll = (180* atan2(2.0f*q2*q3 - 2.0f*q0*q1, 2.0f*q0*q0 + 2.0f*q3*q3 -1.0f))/M_PI;
     attp->pitch = (180*-asin(2.0f * (q1*q3 + q0*q2)))/M_PI;
     attp->yaw = (180*atan2(2.0f*q1*q2 - 2.0f*q0*q3, 2.0f*q2*q2 + 2.0f*q3*q3 -1.0f))/M_PI;*/
}

// Reset yaw offset after 400 mesurements
int16_t yaw_offset(int16_t yaw) {
    static uint8_t counter = 0;

    static uint16_t yaw_bias = 0;
    static uint16_t yaw_min = 3600;
    static uint16_t yaw_max = 0;

    if (counter < 400) {
        yaw_min = yaw < yaw_min ? yaw : yaw_min;
        yaw_max = yaw > yaw_max ? yaw : yaw_max;
        counter++;
        return yaw;
    } else if (counter == 400) {
        yaw_bias = (yaw_max + yaw_min) / 2;
        counter++;
    }
    yaw = yaw - yaw_bias;
    if (yaw < 0) {
        yaw += 3600;
    }
    return yaw;
}
