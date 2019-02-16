#include "simbleeBridge.h"

enum status_code sendData(void) {

    if (usart_callback_transmit_flag) {
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

        memcpy(&buffer[0], "ABO", 3);
        memcpy(&buffer[3], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));

        memcpy(&buffer[15], "ABG", 3);

        sensorFloatToS32(&buffer32s, &gyroData, 1000);
        memcpy(&buffer[18], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));

        memcpy(&buffer[30], "ABR", 3);
        sensorFloatToS32(&buffer32s, &accData, 1000);
        memcpy(&buffer[33], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));

        memcpy(&buffer[45], "ABM", 3);
        sensorFloatToS32(&buffer32s, &magData, 1);
        memcpy(&buffer[48], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));

        memcpy(&buffer[60], "ABT", 3);
        buffer32s.x = twoKp;
        buffer32s.y = -counter;
        buffer32s.z = tc1_ticks;

        memcpy(&buffer[63], (uint8_t *) &buffer32s, sizeof(sensorData32s_t));

        usart_callback_transmit_flag = false;
        return usart_write_buffer_job(&usart_instance, buffer, 75);
    }

    return STATUS_BUSY;
}
