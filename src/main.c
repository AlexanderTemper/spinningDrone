#include "bmf055.h"
#include "MahonyAHRS.h"
#include <math.h>
#include <float.h>

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




uint16_t calc_hardiron(struct bmm050_mag_s32_data_t* mag_data);
void correct_mag(struct bmm050_mag_s32_data_t* mag_data);
float yaw_offset(float yaw);
void gets32att(attitude_32 *att32, attitude_t *attf);
void toSensor(sensor_32 * sensor,float x, float y, float z);
enum status_code sendData(attitude_t *att,float gx,float gy,float gz,float ax, float ay, float az, float mx, float my , float mz);

int main(void)
{
	/********************* Initialize global variables **********************/
	
	bmf055_input_state = USART_INPUT_STATE_PRINT_DATA;
	/*! This structure holds acceleration data of x, y and z axes. */
	struct bma2x2_accel_data accel_data;
	/*! This structure holds angular velocity data of x, y and z axes. */
	struct bmg160_data_t gyro_data;
	/*! This structure holds magnetic field data of x, y and z axes. */
	struct bmm050_mag_s32_data_t mag_data;
	
	attitude_t att;
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
	
	/*Enable the system interrupts*/
	system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */
	
	/* Initialize the sensors */
	bmf055_sensors_initialize();

	/************************** Infinite Loop *******************************/
	while (true)
	{
		/* Print sensor data periodically regarding TC6 interrupt flag (Default Period 10 ms)*/
		if (READ_SENSORS_FLAG)
		{

			/* Read accelerometer's data */
			bma2x2_read_accel_xyz(&accel_data);

			/* Read gyroscope's data */
			bmg160_get_data_XYZ(&gyro_data);

			/* Read magnetometer's data */
			bmm050_read_mag_data_XYZ_s32(&mag_data);

			//uint16_t acc_1g = 1024;
			float ax = (float)accel_data.x;///acc_1g;
			float ay = (float)accel_data.y;///acc_1g;
			float az = (float)accel_data.z;///acc_1g;

			float gyro_scale = 16.3835f;
			float gx = (((float)gyro_data.datax/gyro_scale)*M_PI)/180;
			float gy = (((float)gyro_data.datay/gyro_scale)*M_PI)/180;
			float gz = (((float)gyro_data.dataz/gyro_scale)*M_PI)/180;

			//calc_hardiron(&mag_data);
			correct_mag(&mag_data);
			float mx = mag_data.datax;
			float my = mag_data.datay;
			float mz = mag_data.dataz;


			MahonyAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);

			getMahAttitude(&att);

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

			/*attitude_32 att32;
			sensor_32 gyro;
			sensor_32 acc;
			sensor_32 mag;
			gets32att(&att32, &att);
			toSensor(&gyro,gx*1000, gy*1000, gz*1000);
			toSensor(&acc,ax, ay, az);
			toSensor(&mag,mx, my, mz);*/


			// 10MS * 10 == 100ms
			if(timer > 5){

				sendData(&att, gx, gy, gz, ax, ay, az, mx, my , mz);
				/*usart_write_buffer_wait(&usart_instance, (uint8_t *)"ABO",3);
				usart_write_buffer_wait(&usart_instance, (uint8_t *)&att32,sizeof(att32));
				usart_write_buffer_wait(&usart_instance, (uint8_t *)"ABG",3);
				usart_write_buffer_wait(&usart_instance, (uint8_t *)&gyro,sizeof(gyro));
				usart_write_buffer_wait(&usart_instance, (uint8_t *)"ABR",3);
				usart_write_buffer_wait(&usart_instance, (uint8_t *)&acc,sizeof(acc));
				usart_write_buffer_wait(&usart_instance, (uint8_t *)"ABM",3);
				usart_write_buffer_wait(&usart_instance, (uint8_t *)&mag,sizeof(mag));*/
				//sprintf((char *)usart_buffer_tx, "%3ld %3ld %3ld \r\n",att32.yaw,att32.pitch,att32.roll);
				//usart_write_buffer_job(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));
				//sprintf((char *)usart_buffer_tx, "DATA: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\r\n",ax,ay,az,gx,gy,gz,mx, my, mz);
				//usart_write_buffer_job(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));

				timer = 0;
			} else {
				timer++;
			}

			/* Reset TC flag */
			READ_SENSORS_FLAG = false;
		}
		
	} /* !while (true) */
		
}

enum status_code sendData(attitude_t *att,float gx,float gy,float gz,float ax, float ay, float az, float mx, float my , float mz){

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
		tof.z = 0;

		gets32att(&att32, att);
		toSensor(&gyro,gx*1000, gy*1000, gz*1000);
		toSensor(&acc,ax, ay, az);
		toSensor(&mag,mx, my, mz);

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

void correct_mag(struct bmm050_mag_s32_data_t* mag_data){
	/*! mag bias error x,y,z */
	static int32_t mag_bias[3] = {188, 33, -38};
	//static int32_t mag_bias[3] = {0.0,0.0,0.0};
	int32_t tempx = mag_data->datax;

	mag_data->datax = (mag_data->datay-mag_bias[0]);
	mag_data->datay = (-tempx-mag_bias[1]);
	mag_data->dataz = (-mag_data->dataz-mag_bias[2]);
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

uint16_t calc_hardiron(struct bmm050_mag_s32_data_t* mag_data){
	static uint16_t calibrate = 400;
	static float mag_bias[3] = {0.0, 0.0, 0.0};
	static int32_t mag_max[3] = {INT32_MIN, INT32_MIN, INT32_MIN},mag_min[3] = {INT32_MAX, INT32_MAX, INT32_MAX};
	static int t = 0;

	int32_t mag_temp[3] = {0.0, 0.0, 0.0};


	// remap sensor data
	mag_temp[0] = mag_data->datay;
	mag_temp[1] = -mag_data->datax;
	mag_temp[2] = -mag_data->dataz;

	if(t<10){
		t++;
	} else {
		if(calibrate > 0){
				for (int jj = 0; jj < 3; jj++) {
				  if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				  if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
				}

				mag_bias[0]  = (float)((mag_max[0] + mag_min[0]))/2;  // get average x mag bias in counts
				mag_bias[1]  = (float)((mag_max[1] + mag_min[1]))/2;  // get average y mag bias in counts
				mag_bias[2]  = (float)((mag_max[2] + mag_min[2]))/2;  // get average z mag bias in counts

				calibrate--;
				uint8_t usart_buffer_tx[81] = {0};
				if(calibrate == 0){

					sprintf((char *)usart_buffer_tx, "Kalibrierung fertig %0.6f %0.6f %0.6f\r\n",mag_bias[0],mag_bias[1],mag_bias[2]);
					usart_write_buffer_wait(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));
				} else {
					sprintf((char *)usart_buffer_tx, "Kalibriere %d %0.6f %0.6f %0.6f\r\n",calibrate,mag_bias[0],mag_bias[1],mag_bias[2]);
					usart_write_buffer_wait(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));
				}
		}
		t=0;
	}

	return calibrate;
}

