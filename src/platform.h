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


#define NOINLINE __attribute__((noinline))

//#if !defined(UNIT_TEST) && !defined(SIMULATOR_BUILD) && !(USBD_DEBUG_LEVEL > 0)
//#pragma GCC poison sprintf snprintf
//#endif

//#include "target/common_fc_pre.h"
//#include "target.h"
//#include "target/common_fc_post.h"
//#include "target/common_defaults_post.h"
