#ifndef I2C_SUPPORT_H_
#define I2C_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include <i2c_common.h>
#include <i2c_master.h>


/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

struct i2c_master_module i2c_master_instance;

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/
void i2c_initialize(void);
void i2c_configure_master(void);


#endif /* I2C_SUPPORT_H_ */
