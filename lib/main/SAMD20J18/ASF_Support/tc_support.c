#include "tc_support.h"



struct tc_module muTimer;
struct tc_module waitTimer;
uint32_t muTimerOverflow;

static volatile bool waitTimer_callback_flag;
//volatile bool tc6_callback_flag;

void tc_initialize(void)
{
    muTimer_configure ();
    muTimer_configure_callbacks();

    waitTimer_configure ();
    waitTimer_configure_callbacks();
}


void muTimer_configure (void)
{
	/* TC's configuration structure */
	struct tc_config config_tc;

	/* Get TC configuration default */
	tc_get_config_defaults(&config_tc);

	/* set the counter size */
	config_tc.counter_size = TC_COUNTER_SIZE_32BIT;
	/* set TC GLCK */
	config_tc.clock_source = GCLK_GENERATOR_3;
	/* set the initial counter register value */
	config_tc.counter_32_bit.value = 0;
	muTimerOverflow = 0;
	tc_init(&muTimer,TC0,&config_tc);

	/* enable the TC module */
	tc_enable(&muTimer);
}

void muTimer_configure_callbacks (void)
{
	tc_register_callback(&muTimer, muTimer_callback, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&muTimer, TC_CALLBACK_OVERFLOW);
}
// 1.19304647 hours
void muTimer_callback (struct tc_module *const module_inst_ptr)
{
	/* Reset the counter register value */
    muTimerOverflow++;
	tc_set_count_value(&muTimer, 0);
}

void waitTimer_configure (void)
{
	/* TC's configuration structure */
	struct tc_config config_tc;
	
	/* Get TC configuration default */
	tc_get_config_defaults(&config_tc);
	
	/* set TC GLCK */
	config_tc.clock_source = GCLK_GENERATOR_1;
	
	/* Set the initial compare value */
	config_tc.counter_16_bit.compare_capture_channel[TC_COMPARE_CAPTURE_CHANNEL_0] = 500;
	
	/* initialize TC4 with current configurations */
	tc_init(&waitTimer, TC2, &config_tc);
	
	/* enable the TC module */
	tc_enable(&waitTimer);
	
	/* Stop the counter */
	tc_stop_counter(&waitTimer);
}

void waitTimer_configure_callbacks (void)
{
	tc_register_callback(&waitTimer, waitTimer_callback, TC_CALLBACK_CC_CHANNEL0);
	waitTimer_callback_flag = false;
	tc_enable_callback(&waitTimer, TC_CALLBACK_CC_CHANNEL0);
}

void waitTimer_callback (struct tc_module *const module_inst_ptr)
{
	/* Set the corresponding interrupt flag */
    waitTimer_callback_flag = true;
}

void wait_for_msec (uint32_t msec)
{
	/* Set the compare value */
	tc_set_compare_value(&waitTimer, TC_COMPARE_CAPTURE_CHANNEL_0, (msec *500));
	
	/* start counting */
	tc_start_counter(&waitTimer);
	
	/* delay until required time is elapsed */
	while (!waitTimer_callback_flag);
	
	/* stop the counter */
	tc_stop_counter(&waitTimer);
	/* reset the interrupt flag */
	waitTimer_callback_flag = false;
}
