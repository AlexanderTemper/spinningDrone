#include "imu.h"

void getMahAttitude(attitude_t *attp);

void updateAtt(attitude_t *att,imuData *data, accData *acc, gyroData *gyro, magData *mag){
	//uint16_t acc_1g = 1024;
	float ax = (float)acc->x;///acc_1g;
	float ay = (float)acc->y;///acc_1g;
	float az = (float)acc->z;///acc_1g;

	float gyro_scale = 16.3835f;
	float gx = (((float)gyro->x/gyro_scale)*M_PI)/180;
	float gy = (((float)gyro->y/gyro_scale)*M_PI)/180;
	float gz = (((float)gyro->z/gyro_scale)*M_PI)/180;

	//calc_hardiron(&mag_data);
	float mx = mag->x;
	float my = mag->y;
	float mz = mag->z;


	data->a.x = ax;
	data->a.y = ay;
	data->a.z = az;

	data->g.x = gx;
	data->g.y = gy;
	data->g.z = gz;

	data->m.x = mx;
	data->m.y = my;
	data->m.z = mz;

	MahonyAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);
	getMahAttitude(att);
}


void getMahAttitude(attitude_t *attp) {

  attp->roll = radiansToDegrees(atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2));
  attp->pitch = radiansToDegrees(asin(-2.0f * (q1*q3 - q0*q2)));
  attp->yaw = radiansToDegrees(atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3));
  /*attp->roll = (180* atan2(2.0f*q2*q3 - 2.0f*q0*q1, 2.0f*q0*q0 + 2.0f*q3*q3 -1.0f))/M_PI;
  attp->pitch = (180*-asin(2.0f * (q1*q3 + q0*q2)))/M_PI;
  attp->yaw = (180*atan2(2.0f*q1*q2 - 2.0f*q0*q3, 2.0f*q2*q2 + 2.0f*q3*q3 -1.0f))/M_PI;*/
}
