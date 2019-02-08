/**********************************************************************/

#include "i2c_support.h"


/************************************************************************/
/* Function Definitions                                                 */
/************************************************************************/

void i2c_initialize(void)
{
	i2c_configure_master();
}


void i2c_configure_master(void)
{
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);


	config_i2c_master.baud_rate = I2C_MASTER_BAUD_RATE_100KHZ;
	config_i2c_master.buffer_timeout = 10000;

	/* Initialize and enable device with config. */
    config_i2c_master.pinmux_pad0    = PINMUX_PA22C_SERCOM3_PAD0;
    config_i2c_master.pinmux_pad1    = PINMUX_PA23C_SERCOM3_PAD1;
	i2c_master_init(&i2c_master_instance, SERCOM3, &config_i2c_master);

	i2c_master_enable(&i2c_master_instance);

}
