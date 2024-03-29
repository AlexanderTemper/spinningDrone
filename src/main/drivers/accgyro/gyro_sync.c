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

/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/gyro_sync.h"
#include "globals.h"


bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{

    return true;
}

uint32_t gyroSetSampleRate(gyroDev_t *gyro, uint8_t lpf, uint8_t gyroSyncDenominator)
{
    float gyroSamplePeriod;

	gyro->gyroRateKHz = GYRO_RATE_400_Hz;
	gyroSamplePeriod = MAINCONTROL_LOOPTIME_US;
	gyroSyncDenominator = 1; // Always full Sampling 1khz


    // calculate gyro divider and targetLooptime (expected cycleTime)
    gyro->mpuDividerDrops  = gyroSyncDenominator - 1;
    const uint32_t targetLooptime = (uint32_t)(gyroSyncDenominator * gyroSamplePeriod);
    return targetLooptime;
}
