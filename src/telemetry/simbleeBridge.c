#include "simbleeBridge.h"


enum status_code sendData(void) {

   /* if (usart_callback_transmit_flag) {
        sensorData32s_t buffer32s;
        static uint8_t buffer[75];
        static int32_t counter = 0;
        counter++;

        if (counter > 100) {
            counter = 0;
        }

        buffer32s.x = attitude.values.roll;
        buffer32s.y = attitude.values.pitch;
        buffer32s.z = attitude.values.yaw;

        //Start IMU Frame
        memcpy(&buffer[0], "ABI", 3);
        //Attitude
        memcpy(&buffer[3], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));
        //Gyro
        sensorFloatToS32(&buffer32s, &gyroData, 1000);
        memcpy(&buffer[15], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));
        //ACC
        sensorFloatToS32(&buffer32s, &accData, 1000);
        memcpy(&buffer[27], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));
        //Mag
        sensorFloatToS32(&buffer32s, &magData, 1);
        memcpy(&buffer[39], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));
        //end IMU Frame

        //Start Time Frame
        memcpy(&buffer[51], "ABZ", 3);
        int8_t t = timeing.imuLoop;
        memcpy(&buffer[54], (uint8_t *) &t, 1);
        memcpy(&buffer[55], (uint8_t *) &t, 1);
        memcpy(&buffer[56], (uint8_t *) &timeing.total, 4);
        //end Time Frame

        //Start TOF Frame
        memcpy(&buffer[60], "ABT", 3);
        buffer32s.x = (uint32_t) tofData.x;
        buffer32s.y = 0;
        buffer32s.z = 0;

        memcpy(&buffer[63], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));

        usart_callback_transmit_flag = false;
        return usart_write_buffer_job(&usart_instance, buffer, 75);
    }*/

    return STATUS_BUSY;
}
