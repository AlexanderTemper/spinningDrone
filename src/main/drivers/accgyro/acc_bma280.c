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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "sensors/sensors.h"
#include "drivers/bma2x2_support.h"
#include "fc/runtime_config.h"
#include "acc_bma280.h"

static void bma280Init(accDev_t *acc) {
	bma_init();
	bma2x2_set_range(BMA2x2_RANGE_8G);
	bma2x2_set_bw(BMA2x2_BW_500HZ);
	bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
	//Normalizing Factor to 1G at +-8 with +-2^13 = 8192/8 = 1024
	acc->acc_1G = 1024;
	acc->accAlign = CW0_DEG;
}

static bool bma280Read(accDev_t *acc) {

	struct bma2x2_accel_data rawData;
	if (bma2x2_read_accel_xyz(&rawData) != 0) {
		return false;
	}
	acc->ADCRaw[0] = rawData.x;
	acc->ADCRaw[1] = rawData.y;
	acc->ADCRaw[2] = rawData.z;

	return true;
}

bool bma280Detect(accDev_t *acc) {
	acc->initFn = bma280Init;
	acc->readFn = bma280Read;
	return true;
}
