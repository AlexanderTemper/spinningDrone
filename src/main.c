#include "asf.h"
#include <math.h>
#include <float.h>
#include "vl53l0x.h"
#include "gyro.h"
#include "acc.h"
#include "mag.h"
#include "imu.h"


#include "clock_support.h"
#include "spi_support.h"
#include "i2c_support.h"
#include "tc_support.h"
#include "usart_support.h"



/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! Sensors data are read in accordance with TC6 callback. */
#define READ_SENSORS_FLAG				tc6_callback_flag
/*! USART command process is executed in accordance with USART receive callback. */
#define USART_COMMAND_PROCESS_FLAG		usart_callback_receive_flag


typedef struct {
	int32_t roll;
	int32_t pitch;
	int32_t yaw;
} attitude_32;


typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
} sensor_32;

float yaw_offset(float yaw);
void gets32att(attitude_32 *att32, attitude_t *attf);
void toSensor(sensor_32 * sensor,float x, float y, float z);
enum status_code sendData(attitude_t *att,imuData *data);

int main(void)
{
	/********************* Initialize global variables **********************/
	
	/*! This structure holds acceleration data of x, y and z axes. */
	accData accel_data = {0,0,0};
	/*! This structure holds angular velocity data of x, y and z axes. */
	gyroData gyro_data = {0,0,0};
	/*! This structure holds magnetic field data of x, y and z axes. */
	magData mag_data = {0,0,0};
	
	attitude_t att;
	imuData todoweg;



	uint16_t timer = 0;
	uint16_t startup = 0;
	twoKp = 100.0f;

	/************************* Initializations ******************************/
	
	/*Initialize SAMD20 MCU*/
	system_init();
	
	/*Initialize clock module of SAMD20 MCU - Internal RC clock*/
	clock_initialize();
	
	/*SPI master for communicating with sensors*/
	spi_initialize();
	
	/*Initialize timers */
	tc_initialize();
	
	/*Initialize UART for communication with PC*/
	usart_initialize();
	
	/*Initialize I2C for communication*/
	i2c_initialize();

	/*Enable the system interrupts*/
	system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */
	
	/* Initialize the sensors */
	gyroInit();
	accInit();
	magInit();
	//initVL53L0X(false);
	//setTimeout(400);
	//startContinuous(0);
	/************************** Infinite Loop *******************************/
	while (true)
	{
		/* Print sensor data periodically regarding TC6 interrupt flag (Default Period 10 ms)*/
		if (READ_SENSORS_FLAG)
		{
			/* Read accelerometer's data */
			readAccData(&accel_data);

			/* Read gyroscope's data */
			readGyroData(&gyro_data);

			/* Read magnetometer's data */
			readMagData(&mag_data);

			updateAtt(&att,&todoweg,&accel_data, &gyro_data, &mag_data);

			if(att.yaw<0){
				att.yaw += 360;
			}

			if(startup < 100){
				startup ++;
			} else if (startup == 100){
				twoKp = 5.0f;
				startup ++;
			} else {
				att.yaw = yaw_offset(att.yaw);
			}

			// 10MS * 10 == 100ms
			if(timer > 10){
				sendData(&att,&todoweg);
				timer = 0;
			} else {
				timer++;
			}

			/* Reset TC flag */
			READ_SENSORS_FLAG = false;
		}
		
	} /* !while (true) */
		
}

enum status_code sendData(attitude_t *att,imuData *data){

	if(usart_callback_transmit_flag){
		attitude_32 att32;
		sensor_32 gyro;
		sensor_32 acc;
		sensor_32 mag;
		static uint8_t buffer[75];
		static int32_t counter = 0;
		counter ++;

		if(counter > 100){
			counter = 0;
		}
		sensor_32 tof;
		tof.x = counter;
		tof.y = -counter;
		tof.z = tc1_ticks;

		gets32att(&att32, att);
		toSensor(&gyro,data->g.x*1000, data->g.y*1000, data->g.z*1000);
		toSensor(&acc,data->a.x, data->a.y, data->a.z);
		toSensor(&mag,data->m.x, data->m.y, data->m.z);

		memcpy(&buffer[0],"ABO",3);
		memcpy(&buffer[3],(uint8_t *)&att32,sizeof(attitude_32));

		memcpy(&buffer[15],"ABG",3);
		memcpy(&buffer[18],(uint8_t *)&gyro,sizeof(sensor_32));

		memcpy(&buffer[30],"ABR",3);
		memcpy(&buffer[33],(uint8_t *)&acc,sizeof(sensor_32));

		memcpy(&buffer[45],"ABM",3);
		memcpy(&buffer[48],(uint8_t *)&mag,sizeof(sensor_32));

		memcpy(&buffer[60],"ABT",3);
		memcpy(&buffer[63],(uint8_t *)&tof,sizeof(sensor_32));

		usart_callback_transmit_flag = false;
		return usart_write_buffer_job(&usart_instance, buffer,75);
	}

	return STATUS_BUSY;
}

float yaw_offset(float yaw){
	static uint8_t counter = 0;

	static float yaw_bias = 0;
	static float yaw_min = 360;
	static float yaw_max = 0;


	if(counter <40){
		yaw_min = yaw < yaw_min ? yaw : yaw_min;
		yaw_max = yaw > yaw_max ? yaw : yaw_max;
		counter++;
		return yaw;
	} else if (counter == 40){
		yaw_bias = (yaw_max + yaw_min)/2;
		counter++;
	}
	yaw = yaw - yaw_bias;
	if(yaw<0){
		yaw += 360;
	}
	return yaw;
}


void gets32att(attitude_32 *att32, attitude_t *attf) {
	att32->roll = (int32_t)(attf->roll*100);
	att32->pitch = (int32_t)(attf->pitch*100);
	att32->yaw = (int32_t)(attf->yaw*100);
}


void toSensor(sensor_32 * sensor,float x, float y, float z){
	sensor->x = (int32_t)x;
	sensor->y = (int32_t)y;
	sensor->z = (int32_t)z;
}
