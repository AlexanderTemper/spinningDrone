#include "acc.h"

void rawToAccData(struct bma2x2_accel_data *raw,accData *accdata);

void accInit(void){
	/* Initialize BMA280 */
	bma_init();
	bma2x2_set_range(BMA2x2_RANGE_8G);
    bma2x2_set_bw(BMA2x2_BW_500HZ);
    bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
}

SENSOR_OPERATION_STATUS readAccData(accData *data){
	struct bma2x2_accel_data rawData;
	if(bma2x2_read_accel_xyz(&rawData)!=SENSOR_SUCCESS){
			return SENSOR_ERROR;
	}
	rawToAccData(&rawData,data);
	return SENSOR_SUCCESS;
}

void rawToAccData(struct bma2x2_accel_data *raw,accData *accdata){
	accdata->x = raw->x;
	accdata->y = raw->y;
	accdata->z = raw->z;
}


