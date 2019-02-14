#include "gyro.h"

void rawToGyroData(struct bmg160_data_t *raw,gyroData * gyroData);

void gyroInit(void){
	bmg_init();
	bmg160_set_range_reg(0x00);
	bmg160_set_bw(C_BMG160_BW_116HZ_U8X);
	bmg160_set_power_mode(BMG160_MODE_NORMAL);
}

SENSOR_OPERATION_STATUS readGyroData(gyroData *data){
	struct bmg160_data_t rawData;
	if(bmg160_get_data_XYZ(&rawData)!=SENSOR_SUCCESS){
			return SENSOR_ERROR;
	}
	rawToGyroData(&rawData,data);
	return SENSOR_SUCCESS;
}

void rawToGyroData(struct bmg160_data_t *raw,gyroData *gyrodata){
	gyrodata->x = raw->datax;
	gyrodata->y = raw->datay;
	gyrodata->z = raw->dataz;
}
	
