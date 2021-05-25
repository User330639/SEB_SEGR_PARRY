/*******************************************************************************
 *   @file   protocol.h
 *   @brief  Header file of the device's communication protocol.
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
#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <stdint.h>

#define RW_POS              15
#define CTL_DATA_POS        14
#define CP_POS              6
#define CMP_LATCH_POS       5
#define GAIN_POS            3
#define MSS_POS             2
                           
#define R_W_MASK           (1 << RW_POS)
#define CTL_DATA_MASK      (1 << CTL_DATA_POS)
#define CP_MASK            (3 << CP_POS)
#define CMP_LATCH_MASK     (1 << CMP_LATCH_POS)
#define GAIN_MASK          (3 << GAIN_POS)
#define MSS_MASK           (1 << MSS_POS)

#define R_W(payload)       ((payload) & R_W_MASK)     
#define CTL_DATA(payload)  ((payload) & CTL_DATA_MASK) 
#define CP(payload)        ((payload) & CP_MASK)
#define CMP_LATCH(payload) ((payload) & CMP_LATCH_MASK)
#define GAIN(payload)      ((payload) & GAIN_MASK)
#define MSS(payload)       ((payload) & MSS_MASK)

#define RD                 (0 << RW_POS)
#define WR                 (1 << RW_POS)
                           
#define CTL                (0 << CTL_DATA_POS)
#define DATA               (1 << CTL_DATA_POS)
                           
#define CP_COAX            (0 << CP_POS)
#define CP_TIA             (1 << CP_POS)
#define CP_DRAIN           (2 << CP_POS)
                           
#define CMP_LATCHED        (0 << CMP_LATCH_POS)
#define CMP_NORMAL         (1 << CMP_LATCH_POS)
                           
#define GAIN_1V_mA         (0 << GAIN_POS)
#define GAIN_10V_mA        (1 << GAIN_POS)
#define GAIN_100V_mA       (2 << GAIN_POS)
                           
#define MSS_CLOSED         (0 << MSS_POS)
#define MSS_OPEN           (1 << MSS_POS)

typedef uint16_t _FRAME_PAYLOAD;
typedef uint8_t  _CRC;

typedef struct __attribute__((packed))
{
    _FRAME_PAYLOAD    data;
    _CRC              crc;
} _FRAME_TypeDef;

/***************************************************************************//**
 * @brief   Checks if frame.crc is correct
 *
 * @param   _FRAME_TypeDef* - Pointer to a dedicated structure.
 *
 * @return  0 if CRC-8 (poly = 0x07, init = 0x00) is incorrect, 1 otherwise.
*******************************************************************************/
uint8_t _CRC_IS_CORRECT(_FRAME_TypeDef*);

/***************************************************************************//**
 * @brief   Calculates CRC for the frame provided.
 *
 * @param   _FRAME_TypeDef* - Pointer to a dedicated structure.
 *
 * @return  CRC value. The value is also stored in instance.crc.   
*******************************************************************************/
uint8_t _CALCULATE_CRC(_FRAME_TypeDef*);

#endif // _PROTOCOL_H_
