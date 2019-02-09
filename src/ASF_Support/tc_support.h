/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		tc_support.h
*
* Date:		2015/02/02
*
* Revision:	1.0
*
* Usage:	Part of BMF055 Data Stream Project
*
**************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*
*
*************************************************************************/
/*!
*
* @file		tc_support.h
* @author	Bosch Sensortec
*
* @brief
*
*
*/



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
/*! TC6 count value to overflow after 1000 milliseconds */
#define TC6_PERIOD_1000MS		COUNT_MAX_32BIT - UINT32_C(500000)
/*! TC6 count value to overflow after 100 milliseconds */
#define TC6_PERIOD_100MS		COUNT_MAX_32BIT - UINT32_C(50000)
/*! TC6 count value to overflow after 10 milliseconds */
#define TC6_PERIOD_10MS			COUNT_MAX_32BIT - UINT32_C(5000)
/*! the value loaded onto TC6 count register */
#define TC6_COUNT_VALUE			TC6_PERIOD_10MS

/*! TC1 count value to overflow after 1 milliseconds */
#define TC1_COUNT_VALUE		0xFFFF - UINT16_C(2000)

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

struct tc_module tc1_instance;
struct tc_module tc4_instance;
struct tc_module tc6_instance;

volatile uint32_t tc1_ticks;
volatile bool tc4_callback_flag;
volatile bool tc6_callback_flag;

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/
void tc_initialize(void);

void tc1_configure(void);
void tc1_configure_callbacks(void);
void tc1_callback(struct tc_module *const module_inst_ptr);

void tc4_configure(void);
void tc4_configure_callbacks(void);
void tc4_callback(struct tc_module *const module_inst_ptr);
void tc4_wait_for_msec(uint32_t msec);

void tc6_configure(void);
void tc6_configure_callbacks(void);
void tc6_callback(struct tc_module *const module_inst_ptr);
void tc6_stop_counter(void);
void tc6_start_counter(void);

#endif /* TC_SUPPORT_H_ */

