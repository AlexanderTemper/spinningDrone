#include "mag.h"

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
SENSOR_DATA magData;
rawMagData_t rawMagData;
/************************************************************************/

void rawToMagData(struct bmm050_mag_s32_data_t *raw);
void correctHardIronOffset(struct bmm050_mag_s32_data_t* data);
void remapAxis(struct bmm050_mag_s32_data_t* data);

void magInit(void) {
    /* Initialize BMM150 */
    bmm_init();
    bmm050_set_data_rate(BMM050_DR_10HZ);
    bmm050_set_presetmode(BMM050_PRESETMODE_REGULAR);
    bmm050_set_functional_state(BMM050_NORMAL_MODE);
    sensorsSet(SENSOR_MAG);
}

SENSOR_OPERATION_STATUS readMagData(void) {
    struct bmm050_mag_s32_data_t rawData;
    if (bmm050_read_mag_data_XYZ_s32(&rawData) != SENSOR_SUCCESS) {
        return SENSOR_ERROR;
    }
    remapAxis(&rawData);
    correctHardIronOffset(&rawData);
    rawToMagData(&rawData);
    return SENSOR_SUCCESS;
}

void rawToMagData(struct bmm050_mag_s32_data_t *raw) {
    rawMagData.x = raw->datax;
    rawMagData.y = raw->datay;
    rawMagData.z = raw->dataz;

    magData.x = raw->datax;
    magData.y = raw->datay;
    magData.z = raw->dataz;

}

void remapAxis(struct bmm050_mag_s32_data_t* data) {
    int32_t tempx = data->datax;
    data->datax = data->datay;
    data->datay = -tempx;
    data->dataz = -data->dataz;
}

void correctHardIronOffset(struct bmm050_mag_s32_data_t* data) {
    /*! mag bias error x,y,z */
    static int32_t mag_bias[3] = { 188, 33, -38 };
    //static int32_t mag_bias[3] = {0.0,0.0,0.0};

    data->datax = data->datax - mag_bias[0];
    data->datay = data->datay - mag_bias[1];
    data->dataz = data->dataz - mag_bias[2];
}

/*uint16_t calc_hardiron(struct bmm050_mag_s32_data_t* mag_data){
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
 }*/

