/*******************************************************************************
 *   @file   protocol.c
 *   @brief  Implementation of the device's communication protocol.
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

#include "protocol.h"
#include "stm32f3xx_hal.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

extern CRC_HandleTypeDef hcrc;

uint8_t _CRC_IS_CORRECT(_FRAME_TypeDef* frame)
{
    uint8_t crc = (uint8_t)HAL_CRC_Calculate(&hcrc, (uint32_t*)frame, 1);
    return crc == frame->crc ? 1 : 0;
}

uint8_t _CALCULATE_CRC(_FRAME_TypeDef* frame)
{
    frame->crc = (uint8_t)HAL_CRC_Calculate(&hcrc, (uint32_t*)frame, 1);
    return frame->crc;
}
