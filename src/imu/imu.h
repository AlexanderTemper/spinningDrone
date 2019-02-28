#ifndef IMU_H_
#define IMU_H_

#include "MahonyAHRS.h"
#include "sensor.h"
#include "gyro.h"
#include "acc.h"
#include "mag.h"
#include "time.h"
#include "maths.h"

#define radiansToDegrees(angleRadians) ((angleRadians) * 57.2957795131f)

typedef struct {
    float w,x,y,z;
} quaternion;
#define QUATERNION_INITIALIZE {.w=1, .x=0, .y=0,.z=0}

typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef union {
    int16_t raw[3];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;
#define EULER_INITIALIZE { { 0, 0, 0 } }

extern attitudeEulerAngles_t attitude;


typedef struct imuConfig_s {
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
} imuConfig_t;


typedef struct imuRuntimeConfig_s {
    float dcm_ki;
    float dcm_kp;
} imuRuntimeConfig_t;



void updateAtt(void);

#endif /* !IMU_H_ */
