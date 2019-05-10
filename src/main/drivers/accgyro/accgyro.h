/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#include "common/axis.h"
#include "drivers/sensor.h"

//#pragma GCC diagnostic warning "-Wpadded"

typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_BMG160
} gyroHardware_e;

typedef enum {
    GYRO_HARDWARE_LPF_NORMAL,
    GYRO_HARDWARE_LPF_1KHZ_SAMPLE,
} gyroHardwareLpf_e;

typedef enum {
    GYRO_32KHZ_HARDWARE_LPF_NORMAL,
    GYRO_32KHZ_HARDWARE_LPF_EXPERIMENTAL
} gyro32khzHardwareLpf;

typedef enum {
	GYRO_RATE_160_Hz,
    GYRO_RATE_1_kHz,
    GYRO_RATE_1100_Hz,
    GYRO_RATE_3200_Hz,
    GYRO_RATE_8_kHz,
    GYRO_RATE_9_kHz,
    GYRO_RATE_32_kHz,
} gyroRateKHz_e;

typedef struct gyroDev_s {
    sensorGyroInitFuncPtr initFn;                             // initialize function
    sensorGyroReadFuncPtr readFn;                             // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;                  // read temperature if available
    float scale;                                            // scalefactor
    float gyroZero[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];                        // gyro data after calibration and alignment
    float gyroADCf[XYZ_AXIS_COUNT];
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];
    int16_t temperature;
    sensor_align_e gyroAlign;
    gyroRateKHz_e gyroRateKHz;
    bool gyro_high_fsr;
    uint8_t hardware_lpf;
    uint8_t hardware_32khz_lpf;
    uint8_t mpuDividerDrops;
    uint8_t gyroHasOverflowProtection;
    gyroHardware_e gyroHardware;
} gyroDev_t;



typedef struct accDev_s {
    float acc_1G_rec;
    sensorAccInitFuncPtr initFn;                              // initialize function
    sensorAccReadFuncPtr readFn; // read 3 axis data function
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    sensor_align_e accAlign;
} accDev_t;
