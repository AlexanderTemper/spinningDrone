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

#include "time.h"
#include "tc_support.h"

void delayMicroseconds(timeUs_t us){
   uint32_t now = micros();
   while (micros() - now < us);
}
void delay(timeMs_t ms){
    wait_for_msec(ms);
}

timeUs_t micros(void){
    return muTimer.hw->COUNT32.COUNT.reg;
}
timeUs_t microsISR(void){
    return muTimer.hw->COUNT32.COUNT.reg;
}

timeMs_t millis(void){
    return (4294967.296*muTimerOverflow)+(muTimer.hw->COUNT32.COUNT.reg/1000);
}

/*uint32_t ticks(void){

}
timeDelta_t ticks_diff_us(uint32_t begin, uint32_t end){

}*/
