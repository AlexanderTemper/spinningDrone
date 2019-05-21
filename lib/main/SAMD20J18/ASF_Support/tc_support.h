#ifndef TC_SUPPORT_H_
#define TC_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include <tc.h>
#include <tc_interrupt.h>

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! Maximum value of a 32-bit counter */
#define COUNT_MAX_32BIT			UINT32_C(0xFFFFFFFF)

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

extern struct tc_module muTimer;
extern struct tc_module waitTimer;

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/
void tc_initialize(void);

void muTimer_configure(void);
void muTimer_configure_callbacks(void);
void muTimer_callback(struct tc_module *const module_inst_ptr);

void waitTimer_configure(void);
void waitTimer_configure_callbacks(void);
void waitTimer_callback(struct tc_module *const module_inst_ptr);
void wait_for_msec(uint32_t msec);

#endif /* TC_SUPPORT_H_ */

