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
#define PWM_MIN_PULSE			UINT16_C(24000)
#define ONESHOT_MIN_PULSE		UINT16_C(2400)

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define RCINPUT_LOOPTIME_US	20000	// 50Hz
#define MAINCONTROL_LOOPTIME_US	3000 // 333Hz
#define ssin(val) (val)
#define scos(val) 1.0f

#define RC_MAX 1811
#define RC_MIN 172

enum pid {
	PIDROLL,
	PIDPITCH,
	PIDYAW,
	PIDALT,
	PIDPOS,
	PIDPOSR,
	PIDNAVR,
	PIDLEVEL,
	PIDMAG,
	PIDVEL,     // not used currently
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
	uint8_t rcRate8;
	uint8_t rcExpo8;
	uint8_t rollPitchRate[2];
	uint8_t yawRate;
	uint8_t dynThrPID;
	uint8_t thrMid8;
	uint8_t thrExpo8;
	int16_t accZero[3];
	int16_t angleTrim[2];
	uint16_t max_angle_inclination;
	uint16_t activate[CHECKBOX_ITEM_COUNT];
	uint16_t tpa_breakpoint; // Breakpoint where TPA is activated
	uint8_t deadband;    // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
	uint8_t yawdeadband; // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
	uint8_t F3D;
	uint8_t MIDDLEDEADBAND;
	int16_t deadband3d_high;
	int16_t deadband3d_low;

	uint8_t sOneShot;

	uint8_t copterType;

	uint8_t RxType;

	int16_t MINTHROTTLE;
	int16_t MAXTHROTTLE;
	int16_t MINCOMMAND;
	int16_t MIDRC;
	int16_t MINCHECK;
	int16_t MAXCHECK;

	uint8_t  YAW_DIRECTION;

	uint8_t  ArmRoll;
	uint16_t  s3DMIDDLE;

	uint8_t calibState;
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

#define MAX_MOTORS 4

extern struct config conf;

extern int16_t axisPID[3];
extern int16_t motor[MAX_MOTORS];
extern int16_t servo[6];
extern int16_t Zadd;

extern uint32_t cycleTime; // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop


extern uint8_t s3D;
extern uint8_t NUMBER_MOTOR;
extern uint8_t MULTITYPE;
extern uint8_t throttleTest;
extern uint8_t rcOptions[CHECKBOX_ITEM_COUNT];
extern int16_t rcCommand[4]; // interval [1000;2000] for THROTTLE and [-500;+500] for

void mixerSetThrottleAngleCorrection(int16_t correctionValue);

#endif /* GLOBALS_H_ */
