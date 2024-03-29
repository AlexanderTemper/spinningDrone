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

#define TARGET_BOARD_IDENTIFIER "B55A"

#define USBD_PRODUCT_STRING "Bmf055ATemper"

#define USE_ACC
#define USE_ACCGYRO_BMG160

#define LED0_PIN PA0
#define LED1_PIN PB1
#define LED2_PIN PB1

#define USE_SERIAL_RX
#define USE_SERIALRX_SBUS

#define RC_SMOOTHING_AUTO 0
#define INTERPOLATION_CHANNELS_RPYT 0
#define RC_SMOOTHING_TYPE_FILTER 0
#define RC_SMOOTHING_INPUT_BIQUAD 0
#define RC_SMOOTHING_DERIVATIVE_BIQUAD 0

#define USE_PWM_OUTPUT


