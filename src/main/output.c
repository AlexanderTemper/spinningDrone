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

#include <tc.h>
#include <tc_interrupt.h>
//#include "eeprom_emulation.h"
#include "output.h"
#include "globals.h"
#include "rx/rx.h"


#define MAX_MOTORS 4

int16_t motor_disarmed[MAX_MOTORS];
int16_t motor[MAX_MOTORS];
struct tc_module tc_instance1,tc_instance2;

volatile bool tc_instance1_callback_flag;
volatile bool tc_instance2_callback_flag;

/**************************************************************************************/
/*****   Writes the Motors and Servos values to the Timer PWM compare registers   ******/
/**************************************************************************************/
void writeMotors() {

	if(tc_instance1_callback_flag && tc_instance2_callback_flag){

		uint16_t timer_val = COUNT_MAX_16BIT-ONESHOT_MIN_PULSE;

		tc_set_count_value(&tc_instance1,timer_val);
		tc_set_count_value(&tc_instance2,timer_val);

		tc_instance1_callback_flag = false;
		tc_instance2_callback_flag = false;


        tc_set_compare_value(&tc_instance1,0,COUNT_MAX_16BIT-ONESHOT_MIN_PULSE+motor[0]);
        tc_set_compare_value(&tc_instance1,1,COUNT_MAX_16BIT-ONESHOT_MIN_PULSE+motor[1]);
        tc_set_compare_value(&tc_instance2,0,COUNT_MAX_16BIT-ONESHOT_MIN_PULSE+motor[2]);
        tc_set_compare_value(&tc_instance2,1,COUNT_MAX_16BIT-ONESHOT_MIN_PULSE+motor[3]);


		tc_enable(&tc_instance1); 
		tc_enable(&tc_instance2);

	}
}


/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void tc_instance1_callback (struct tc_module *const module_inst_ptr)
{
	tc_instance1_callback_flag = true;
}
void tc_instance2_callback (struct tc_module *const module_inst_ptr)
{
	tc_instance2_callback_flag = true;
}


void initOutput() {
	struct	tc_config config_tc1;
	tc_get_config_defaults(&config_tc1);
	config_tc1.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc1.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
	config_tc1.clock_source = GCLK_GENERATOR_4;
	config_tc1.oneshot = true;
	config_tc1.pwm_channel[0].enabled = true;
	config_tc1.pwm_channel[0].pin_out = PIN_PA22F_TC4_WO0;
	config_tc1.pwm_channel[0].pin_mux = MUX_PA22F_TC4_WO0;
	config_tc1.pwm_channel[1].enabled = true;
	config_tc1.pwm_channel[1].pin_out = PIN_PA23F_TC4_WO1;
	config_tc1.pwm_channel[1].pin_mux = MUX_PA23F_TC4_WO1;
	tc_init(&tc_instance1, TC4, &config_tc1); //PA22,23
	tc_register_callback(&tc_instance1, tc_instance1_callback,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance1, TC_CALLBACK_OVERFLOW);
	tc_instance1_callback_flag = true;

	struct	tc_config config_tc2;
	tc_get_config_defaults(&config_tc2);
	config_tc2.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc2.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
	config_tc2.clock_source = GCLK_GENERATOR_4;
	config_tc2.oneshot = true;
	config_tc2.pwm_channel[0].enabled = true;
	config_tc2.pwm_channel[0].pin_out = PIN_PB00F_TC7_WO0;
	config_tc2.pwm_channel[0].pin_mux = MUX_PB00F_TC7_WO0;
	config_tc2.pwm_channel[1].enabled = true;
	config_tc2.pwm_channel[1].pin_out = PIN_PB01F_TC7_WO1;
	config_tc2.pwm_channel[1].pin_mux = MUX_PB01F_TC7_WO1;
	tc_init(&tc_instance2, TC7, &config_tc2); //PB00,01
	tc_register_callback(&tc_instance2, tc_instance2_callback,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance2, TC_CALLBACK_OVERFLOW);
	tc_instance2_callback_flag = true;
}


static uint8_t numberMotor = 0;
static motorMixer_t currentMixer[MAX_MOTORS];

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

static void mixerResetMotors(void)
{
    int i;
    // set disarmed motor values
    for (i = 0; i < MAX_MOTORS; i++){
        motor_disarmed[i] = conf.MINCOMMAND;
    }
}

void mixerInit(void)
{
    int i;
    numberMotor = MAX_MOTORS;
    // copy motor-based mixers
    for (i = 0; i < MAX_MOTORS; i++) {
        currentMixer[i] = mixerQuadX[i];
    }
    mixerResetMotors();
}



void mixTable(void)
{
    int16_t maxMotor;
    uint32_t i;

    if (numberMotor > 3) {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }

    // motors for non-servo mixes
    if (numberMotor > 1) {
        for (i = 0; i < numberMotor; i++) {
            motor[i] = rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + -conf.YAW_DIRECTION * axisPID[YAW] * currentMixer[i].yaw;
        }
    }

    maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++) {
        if (motor[i] > maxMotor) {
            maxMotor = motor[i];
        }
    }
    for (i = 0; i < numberMotor; i++) {
        if (maxMotor > conf.MAXTHROTTLE) {    // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxMotor - conf.MAXTHROTTLE;
        }

        motor[i] = constrain(motor[i], conf.MINTHROTTLE, conf.MAXTHROTTLE);
        if ((rcData[THROTTLE]) < conf.MINCHECK) {
            motor[i] = conf.MINCOMMAND;
        }

        if (!f.ARMED) {
            motor[i] = motor_disarmed[i];
        }
    }
}

