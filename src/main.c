/**\mainpage
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		main.c
*
* Date:		2015/02/02
*
* Revision:	1.0
*
* Usage:	Part of BMF055 Data Stream Project
*
**************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*
*
*************************************************************************/
/*!
*
* @file		main.c
* @author	Bosch Sensortec
*
* @brief	Main Source File
*
*/


/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include "bmf055.h"
#include "MahonyAHRS.h"
#include <math.h>
#include <float.h>

/************************************************************************/
/* Main Function Definition                                             */
/************************************************************************/

/*!
* @brief	Initializes the whole system and runs the desired application
*
* This is the main function of the project. It calls initialization functions
* of the MCU and the sensors. In the infinite loop it repeatedly checks
* the USART module read buffer and Streams sensor data periodically (100 ms) via USART.
*
*/
int main(void)
{
	/********************* Initialize global variables **********************/
	
	bmf055_input_state = USART_INPUT_STATE_PRINT_DATA;
	/*! This structure holds acceleration data of x, y and z axes. */
	struct bma2x2_accel_data accel_data;
	/*! This structure holds angular velocity data of x, y and z axes. */
	struct bmg160_data_t gyro_data;
	/*! This structure holds magnetic field data of x, y and z axes. */
	struct bmm050_mag_data_float_t mag_data;
	
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
	
	/*Enable the system interrupts*/
	system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */
	
	/* Initialize the sensors */
	bmf055_sensors_initialize();
	uint16_t timer = 0;
	
	/************************** Infinite Loop *******************************/
	while (true)
	{
		/* Process USART inputs */
		if (USART_COMMAND_PROCESS_FLAG)
		{
			bmf055_usart_read_process(usart_rx_string[0]);
			
			/* Reset USART Flag */
			USART_COMMAND_PROCESS_FLAG = false;
		}

		/* Print sensor data periodically regarding TC6 interrupt flag (Default Period 10 ms)*/
		if (READ_SENSORS_FLAG)
		{

			/* Read accelerometer's data */
			bma2x2_read_accel_xyz(&accel_data);

			/* Read gyroscope's data */
			bmg160_get_data_XYZ(&gyro_data);

			/* Read magnetometer's data */
			bmm050_read_mag_data_XYZ_float(&mag_data);

			uint16_t acc_1g = 1024;
			float ax = (float)accel_data.x/acc_1g;
			float ay = (float)accel_data.y/acc_1g;
			float az = (float)accel_data.z/acc_1g;

			float gyro_scale = 16.3835f;
			float gx = (((float)gyro_data.datax/gyro_scale)*M_PI)/180;
			float gy = (((float)gyro_data.datay/gyro_scale)*M_PI)/180;
			float gz = (((float)gyro_data.dataz/gyro_scale)*M_PI)/180;

			float mx = mag_data.datay*0.01;
			float my = -mag_data.datax*0.01;
			float mz = -mag_data.dataz*0.01;


			MahonyAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);


			attitude_t att;
			getMahAttitude(&att);
			//att.pitch = 180 * atan (ax/sqrt(ay*ay + az*az))/M_PI;
			//att.roll = 180 * atan (ay/sqrt(ax*ax + az*az))/M_PI;
			//att.yaw = 180 * atan (az/sqrt(ax*ax + az*az))/M_PI;


			if(att.yaw<0){
				att.yaw += 360.0f;
			}

			// 10MS * 10 == 100ms
			if(timer > 5){
				uint8_t usart_buffer_tx[81] = {0};
				sprintf((char *)usart_buffer_tx, "Orientation: %.3f %.3f %.3f\r\n",att.yaw,att.pitch,att.roll);
				usart_write_buffer_wait(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));
				sprintf((char *)usart_buffer_tx, "DATA: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\r\n",ax,ay,az,gx,gy,gz,mx*100, my*100, mz*100);
				usart_write_buffer_wait(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));

				//sprintf((char *)usart_buffer_tx, "Ac:%.3f %.3f %.3f  Mag:%.3f %.3f %.3f Gyro:%.0f %.0f %.0f \r\n",ax,ay,az,mx, my, mz,gx,gy,gz);
				//usart_write_buffer_wait(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));
				//sprintf((char *)usart_buffer_tx, "Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",accel_data.x,accel_data.y,accel_data.z,gyro_data.datay,gyro_data.datax,gyro_data.dataz,mag_data.datay,mag_data.datax,mag_data.dataz);
				//usart_write_buffer_wait(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));
				//bmf055_sensors_data_print();
				timer = 0;
			} else {
				timer++;
			}

			
			/* Reset TC flag */
			READ_SENSORS_FLAG = false;
		}
		
	} /* !while (true) */
		
}

