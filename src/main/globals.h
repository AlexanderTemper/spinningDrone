/*
 * This file is part of the MW21 adaption for BMF055.
 *
 * BMF055 flight controller is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BMF055 flight controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BMF055 flight controller.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_
#include "fc/rc_modes.h"
#define  VERSION  210

#define COUNT_MAX_16BIT			UINT16_C(0xFFFF)
#define ONESHOT_MIN_PULSE		UINT16_C(2400)

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define RCINPUT_LOOPTIME_US	20000	// 50Hz
#define MSP_LOOPTIME_US 5000  // 200Hz
#define MAINCONTROL_LOOPTIME_US	3000 // 333Hz
#define ssin(val) (val)
#define scos(val) 1.0f

#define RC_MAX 1811
#define RC_MIN 172

enum pid {
	PIDROLL,
	PIDPITCH,
	PIDYAW,
	PIDLEVEL,
	PIDMAG,
	PIDITEMS
};

struct flags_struct {
	uint8_t OK_TO_ARM :1 ;
	uint8_t ARMED :1 ;
	uint8_t ACC_MODE :1 ;
	uint8_t HORIZON_MODE :1 ;
	uint8_t ANGLE_MODE :1 ;
	uint8_t SMALL_ANGLES_25 :1 ;
	uint8_t FSBEEP :1 ;
};

extern struct flags_struct f;

// ************************
// EEPROM Layout definition
// ************************
struct config{
	uint8_t checkNewConf;
	uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
	int16_t accZero[3];
	int16_t angleTrim[2];
	uint16_t max_angle_inclination;
	int16_t MINTHROTTLE;
	int16_t MAXTHROTTLE;
	int16_t MINCOMMAND;
	int16_t MIDRC;
	int16_t MINCHECK;
	int16_t MAXCHECK;
	uint8_t YAW_DIRECTION;
};

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_t {
    uint8_t numberMotor;
    const motorMixer_t *motor;
} mixer_t;

extern struct config conf;

extern int16_t axisPID[3];
extern int16_t motor[4];

extern int16_t Zadd; // Todo implement

extern uint32_t cycleTime; // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

extern int16_t rcCommand[4]; // interval [1000;2000] for THROTTLE and [-500;+500] for
extern uint16_t msp_rc_timeout;

void mixerSetThrottleAngleCorrection(int16_t correctionValue);

#endif /* GLOBALS_H_ */
