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
#include "drivers/bmg160_support.h"
#include "fc/runtime_config.h"
#include "gyro_bmg160.h"

static void bmg160Init(gyroDev_t *gyro) {
	bmg_init();
	bmg160_set_range_reg(0x00);
	bmg160_set_bw(C_BMG160_BW_116HZ_U8X);
	bmg160_set_power_mode(BMG160_MODE_NORMAL);
	gyro->scale = 1.0f / 16.4f;
	gyro->gyroAlign = CW0_DEG;
}

static bool bmg160Read(gyroDev_t *gyro) {

	struct bmg160_data_t rawData;
	if (bmg160_get_data_XYZ(&rawData) != 0) {
		return false;
	}
	gyro->gyroADCRaw[X] = rawData.datax;
	gyro->gyroADCRaw[Y] = rawData.datay;
	gyro->gyroADCRaw[Z] = rawData.dataz;

	return true;
}

uint8_t bmg160Detect(gyroDev_t *gyro) {
	gyro->initFn = bmg160Init;
	gyro->readFn = bmg160Read;

	return GYRO_BMG160;
}
