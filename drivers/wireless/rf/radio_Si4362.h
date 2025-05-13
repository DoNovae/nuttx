/*! @file radio.h
 * @brief This file is contains the public radio interface functions.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */
#include "bsp.h"

#ifndef SI4362_NSTDPREAMBLE_NOFLAG
#define SI4362_NSTDPREAMBLE_NOFLAG


/*****************************************************************************
 *  Global Function Declarations
 *****************************************************************************/
void  si4362_vRadio_Init(void);
void si4362_vRadio_Version(void);
uint8_t si4362_bRadio_Check_Tx_RX(void);
void si4362_vRadio_StartRX(uint8_t,uint8_t);
void si4362_vRadio_StartTx_Variable_Packet(uint8_t, uint8_t*, uint8_t);

#endif /* SI4362_NSTDPREAMBLE_NOFLAG */
