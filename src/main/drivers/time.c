#include "drivers/time.h"

/*void delayMicroseconds(timeUs_t us){

}*/
void delay(timeMs_t ms){
	tc4_wait_for_msec(ms);
}

timeUs_t micros(void){
    return tc1_ticks*1000;
}
timeUs_t microsISR(void){
    return 1;
}
timeMs_t millis(void){
    return tc1_ticks;
}

uint32_t ticks(void){
    return tc1_ticks;
}
/*timeDelta_t ticks_diff_us(uint32_t begin, uint32_t end){
    return 1;
}*/

