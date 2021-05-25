/*******************************************************************************
 *   @file   AD5663.h
 *   @brief  Header file of AD5663 Driver.
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
#ifndef _AD5663_H_
#define _AD5663_H_

#include <stdint.h>
							         
#define AD5663_ADDR_POS 16
#define AD5663_COMM_POS 19

typedef enum
{
	AD5663_DAC_A_ADDR   = (0 << AD5663_ADDR_POS),
	AD5663_DAC_B_ADDR   = (1 << AD5663_ADDR_POS),
	AD5663_DAC_A_B_ADDR = (7 << AD5663_ADDR_POS)
} AD5663_Addr_TypeDef;

typedef enum
{
	AD5663_COMM_WRITE            = (0 << AD5663_COMM_POS),
	AD5663_COMM_UPDATE           = (1 << AD5663_COMM_POS),
	AD5663_COMM_WRITE_UPDATE_ALL = (2 << AD5663_COMM_POS),
	AD5663_COMM_WRITE_UPDATE_TAR = (3 << AD5663_COMM_POS),
	AD5663_COMM_PWDN_PWUP        = (4 << AD5663_COMM_POS),
	AD5663_COMM_RESET            = (5 << AD5663_COMM_POS),
	AD5663_COMM_LDAC_SETUP       = (6 << AD5663_COMM_POS)
} AD5663_Cmd_TypeDef;

typedef enum
{
	SOFT = 0,
	POR  = 1
} AD5663_RstMode_TypeDef;

typedef enum
{
	NORMAL_OP     = 0,
	_1KR_TO_GND   = 1,
	_100KR_TO_GND = 2,
	THREE_STATE   = 3
} AD5663_PwdnMode_TypeDef;

typedef enum
{
	HW    = 0,
	AUTO  = 1
} AD5663_LatchMode_TypeDef;

typedef struct
{	
/* Platform specific methods must be implemented by the user.
   All of them must return 0 on success or any nonzero value otherwise. */
	int (* WriteRegister)(uint32_t);
	int (* Latch)(void);
	int (* Clear)(void);
	
	AD5663_RstMode_TypeDef   reset_mode;
	AD5663_LatchMode_TypeDef DAC_A_latch;
	AD5663_LatchMode_TypeDef DAC_B_latch;
    AD5663_PwdnMode_TypeDef  DAC_A_pwdn_mode;
    AD5663_PwdnMode_TypeDef  DAC_B_pwdn_mode;
} AD5663_TypeDef;

/***************************************************************************//**
 * @brief   Resets the part.
 *          Reset behavior depends on the instance's 'reset_mode' value.
 *          Refer to table 9 on page 15 of the Datasheet.
 *
 * @param   AD5663_TypeDef* - Pointer to a dedicated structure.
 *
 * @return  0 on success, nonzero otherwise.    
*******************************************************************************/
int AD5663_Reset(AD5663_TypeDef*);

/***************************************************************************//**
 * @brief   Sets the latching mode.
 *          The latching mode depends on the instance's DAC_A.latch_mode and
 *          DAC_B.latch_mode.
 *          Refer to table 13 on page 17 of the Datasheet.
 *
 * @param   AD5663_TypeDef* - Pointer to a dedicated structure.
 *
 * @return  0 on success, nonzero otherwise.    
*******************************************************************************/
int AD5663_SetLatchMode(AD5663_TypeDef*);

/***************************************************************************//**
 * @brief   Controls the low power mode.
 *          The low power mode depends on the instance's DAC_A_state and
 *          DAC_B_state.
 *          Refer to table 11 on page 16 of the Datasheet.
 *
 * @param   AD5663_TypeDef*     - Pointer to a dedicated structure.
 *          AD5663_Addr_TypeDef - Address of the target DAC.
 *
 * @return  0 on success, nonzero otherwise.    
*******************************************************************************/
int AD5663_PwdnCtl(AD5663_TypeDef*, AD5663_Addr_TypeDef);

/***************************************************************************//**
 * @brief   Sets the input code for the DAC or for both DACs.
 *          Refer to table 7 on page 14 of the Datasheet.
 *
 * @param   AD5663_TypeDef*     - Pointer to a dedicated structure.
 *          AD5663_Addr_TypeDef - Address of the target DAC.
 *          AD5663_Cmd_TypeDef  - Command to be executed.
 *          uint16_t            - value.
 *
 * @return  0 on success, nonzero otherwise.    
*******************************************************************************/
int AD5663_SetInputCode(AD5663_TypeDef*, AD5663_Addr_TypeDef, AD5663_Cmd_TypeDef, uint16_t);

#endif // _AD5663_H_
