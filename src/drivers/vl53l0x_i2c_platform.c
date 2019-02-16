#include "vl53l0x_i2c_platform.h"

#define TIMEOUT 1000
#define I2C_DEBUG

void debugwritei2c(struct i2c_master_packet * const packet) {
    uint16_t buffer_counter = 0;
    uint16_t tmp_data_length = packet->data_length;
    while (tmp_data_length--) {
        uint8_t usart_buffer_tx[81] = { 0 };
        uint8_t data = packet->data[buffer_counter++];
        sprintf((char *) usart_buffer_tx, "ABD send[%d/%d]:0x%x          ", packet->data_length, buffer_counter - 1, data);
        usart_write_buffer_wait(&usart_instance, usart_buffer_tx, sizeof(usart_buffer_tx));
    }
}

void debugreadi2c(struct i2c_master_packet * const packet) {
    uint16_t buffer_counter = 0;
    uint16_t tmp_data_length = packet->data_length;
    while (tmp_data_length--) {
        uint8_t read = packet->data[buffer_counter++];
        uint8_t usart_buffer_tx[81] = { 0 };
        sprintf((char *) usart_buffer_tx, "ABD read[%d/%d]:0x%x         ", packet->data_length, buffer_counter - 1, read);
        usart_write_buffer_wait(&usart_instance, usart_buffer_tx, sizeof(usart_buffer_tx));
    }
}

/**
 * @brief Writes the supplied byte buffer to the device
 *
 * Wrapper for SystemVerilog Write Multi task
 *
 * @code
 *
 * Example:
 *
 * uint8_t *spad_enables;
 *
 * int status = VL53L0X_write_multi(RET_SPAD_EN_0, spad_enables, 36);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to uint8_t buffer containing the data to be written
 * @param  count - number of bytes in the supplied byte buffer
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count) {
    uint16_t mytimeout = 0;
    struct i2c_master_packet packet = {
            .address = address,
            .data_length = 1,
            .data = &index,
            .ten_bit_address = false,
            .high_speed = false,
            .hs_master_code =0x0,
    };

#ifdef I2C_DEBUG
    debugwritei2c(&packet);
#endif

    //Send reg to write
    while (i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet) != STATUS_OK) {
        if (mytimeout++ == TIMEOUT) {
#ifdef I2C_DEBUG
            usart_write_buffer_wait(&usart_instance, (uint8_t *) "ABD timeout writ", 16);
#endif
            return VL53L0X_ERROR_TIME_OUT;
        }
    }
    mytimeout = 0;
    packet.data_length = count;
    packet.data = pdata;

#ifdef I2C_DEBUG
    debugwritei2c(&packet);
#endif

    while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
        if (mytimeout++ == TIMEOUT) {
#ifdef I2C_DEBUG
            usart_write_buffer_wait(&usart_instance, (uint8_t *) "ABD timeout", 11);
#endif
            return VL53L0X_ERROR_TIME_OUT;
        }
    }

    return VL53L0X_ERROR_NONE;
}

/**
 * @brief  Reads the requested number of bytes from the device
 *
 * Wrapper for SystemVerilog Read Multi task
 *
 * @code
 *
 * Example:
 *
 * uint8_t buffer[COMMS_BUFFER_SIZE];
 *
 * int status = status  = VL53L0X_read_multi(DEVICE_ID, buffer, 2)
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to the uint8_t buffer to store read data
 * @param  count - number of uint8_t's to read
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count) {
    uint16_t mytimeout = 0;
    struct i2c_master_packet packet = {
            .address = address,
            .data_length = 1,
            .data = &index,
            .ten_bit_address = false,
            .high_speed = false,
            .hs_master_code = 0x0,
    };

#ifdef I2C_DEBUG
    debugwritei2c(&packet);
#endif

    while (i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet) != STATUS_OK) {
        if (mytimeout++ == TIMEOUT) {
#ifdef I2C_DEBUG
            usart_write_buffer_wait(&usart_instance, (uint8_t *) "ABD timeout writ", 16);
#endif
            return VL53L0X_ERROR_TIME_OUT;
        }
    }

    mytimeout = 0;
    packet.data_length = count;
    packet.data = pdata;

    while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
        // Increment timeout counter and check if timed out.
        if (mytimeout++ == TIMEOUT) {
#ifdef I2C_DEBUG
            usart_write_buffer_wait(&usart_instance, (uint8_t *) "ABD timeout", 11);
#endif
            return VL53L0X_ERROR_TIME_OUT;
        }
    }
#ifdef I2C_DEBUG
    debugreadi2c(&packet);
#endif
    return VL53L0X_ERROR_NONE;
}

/**
 * @brief  Writes a single byte to the device
 *
 * Wrapper for SystemVerilog Write Byte task
 *
 * @code
 *
 * Example:
 *
 * uint8_t page_number = MAIN_SELECT_PAGE;
 *
 * int status = VL53L0X_write_byte(PAGE_SELECT, page_number);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uint8_t data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data) {
    return VL53L0X_write_multi(address, index, &data, 1);
}

/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the MS byte).
 * Uses SystemVerilog Write Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint16_t nvm_ctrl_pulse_width = 0x0004;
 *
 * int status = VL53L0X_write_word(NVM_CTRL__PULSE_WIDTH_MSB, nvm_ctrl_pulse_width);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uin16_t data value write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data) {
    uint8_t buff[2];
    buff[1] = data & 0xFF;
    buff[0] = data >> 8;
    return VL53L0X_write_multi(address, index, buff, 2);
}

/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the MS byte).
 * Uses SystemVerilog Write Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint32_t nvm_data = 0x0004;
 *
 * int status = VL53L0X_write_dword(NVM_CTRL__DATAIN_MMM, nvm_data);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uint32_t data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data) {
    uint8_t buff[4];

    buff[3] = data & 0xFF;
    buff[2] = data >> 8;
    buff[1] = data >> 16;
    buff[0] = data >> 24;

    return VL53L0X_write_multi(address, index, buff, 4);
}

/**
 * @brief  Reads a single byte from the device
 *
 * Uses SystemVerilog Read Byte task.
 *
 * @code
 *
 * Example:
 *
 * uint8_t device_status = 0;
 *
 * int status = VL53L0X_read_byte(STATUS, &device_status);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index  - uint8_t register index value
 * @param  pdata  - pointer to uint8_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata) {
    return VL53L0X_read_multi(address, index, pdata, 1);
}

/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 * Uses SystemVerilog Read Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint16_t timeout = 0;
 *
 * int status = VL53L0X_read_word(TIMEOUT_OVERALL_PERIODS_MSB, &timeout);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index  - uint8_t register index value
 * @param  pdata  - pointer to uint16_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata) {
    uint8_t buff[2];
    int r = VL53L0X_read_multi(address, index, buff, 2);

    uint16_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    *pdata = tmp;

    return r;
}

/**
 * @brief  Reads a single dword (32-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 * Uses SystemVerilog Read Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint32_t range_1 = 0;
 *
 * int status = VL53L0X_read_dword(RANGE_1_MMM, &range_1);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to uint32_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata) {
    uint8_t buff[4];
    int r = VL53L0X_read_multi(address, index, buff, 4);

    uint32_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    tmp <<= 8;
    tmp |= buff[2];
    tmp <<= 8;
    tmp |= buff[3];

    *pdata = tmp;

    return r;
}



/**
 * @brief Set GPIO value
 *
 * @param  level  - input  level - either 0 or 1
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_set_gpio(uint8_t level) {
    return VL53L0X_ERROR_NONE;
}

/**
 * @brief Get GPIO value
 *
 * @param  plevel - uint8_t pointer to store GPIO level (0 or 1)
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_get_gpio(uint8_t *plevel) {
    return VL53L0X_ERROR_NONE;
}

/**
 * @brief Release force on GPIO
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_release_gpio(void) {
    return VL53L0X_ERROR_NONE;
}
