/*******************************************************************************
 *   @file   AD5663.c
 *   @brief  Implementation of AD5663 Driver.
 *   @author Petr Belyaev (arj1939@gmail.com)
********************************************************************************
 *
 *   Copyright (C) 2019
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *   
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *   
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
*******************************************************************************/

#include "AD5663.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

int AD5663_Reset(AD5663_TypeDef* instance)
{
	if (NULL == instance->WriteRegister) 
		goto err;	
	if (0 != instance->WriteRegister(AD5663_COMM_RESET | instance->reset_mode))
		goto err;
	return 0;
err:
	return -1;
}

int AD5663_SetLatchMode(AD5663_TypeDef* instance)
{
	if (NULL == instance->WriteRegister) 
		goto err;
    if (0 != instance->WriteRegister(AD5663_COMM_LDAC_SETUP | instance->DAC_A_latch |
                                     (instance->DAC_B_latch << 1)))
        goto err;
    return 0;
err:
    return -1;
}

int AD5663_PwdnCtl(AD5663_TypeDef* instance, AD5663_Addr_TypeDef target)
{
	if (NULL == instance->WriteRegister)
		goto err;
	if (target == AD5663_DAC_B_ADDR) goto dac_b;
	if (0 != instance->WriteRegister(AD5663_COMM_PWDN_PWUP | (instance->DAC_A_pwdn_mode << 4) | 1))
		goto err;
	if (target == AD5663_DAC_A_ADDR) goto ret;
dac_b:
	if (0 != instance->WriteRegister(AD5663_COMM_PWDN_PWUP | (instance->DAC_B_pwdn_mode << 4) | 2))
		goto err;
ret:
	return 0;
err:
	return -1;
}

int AD5663_SetInputCode(AD5663_TypeDef* instance, AD5663_Addr_TypeDef target, AD5663_Cmd_TypeDef cmd, uint16_t value)
{
    if (NULL == instance->WriteRegister)
        goto err;
    if (cmd > AD5663_COMM_WRITE_UPDATE_TAR)
        goto err;
    if (0 != instance->WriteRegister(cmd | target | value))
        goto err;
    return 0;
err:
    return -1;
}
