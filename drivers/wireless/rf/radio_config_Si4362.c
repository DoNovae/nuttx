/*! @file radio.c
 * @brief This file contains functions to interface with the radio chip.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#include "bsp.h"
#include "radio_config_Si4362.h"
#include "radio_Si4362.h"

//HBL230425 namespace SI4362_NSTDPREAMBLE_NOFLAG
//HBL230425 {

/*****************************************************************************
 *  Local Macros & Definitions
 *****************************************************************************/

/*****************************************************************************
 *  Global Variables
 *****************************************************************************/
const SEGMENT_VARIABLE(Radio_Configuration_Data_Array[], U8, SEG_CODE) = \
		RADIO_CONFIGURATION_DATA_ARRAY;

const SEGMENT_VARIABLE(RadioConfiguration, tRadioConfiguration, SEG_CODE) = \
		RADIO_CONFIGURATION_DATA;

const SEGMENT_VARIABLE_SEGMENT_POINTER(pRadioConfiguration, tRadioConfiguration*, SEG_CODE, SEG_CODE) = \
		&RadioConfiguration;


SEGMENT_VARIABLE(customRadioPacket[RADIO_MAX_PACKET_LENGTH], U8, SEG_XDATA);

/*****************************************************************************
 *  Local Function Declarations
 *****************************************************************************/
void vRadio_PowerUp(void);

/*!
 *  Power up the Radio.
 *
 *  @note
 *
 */
void vRadio_PowerUp(void)
{
	//SEGMENT_VARIABLE(wDelay,  U16, SEG_XDATA) = 0u;

	/* Hardware reset the chip */
	si446x_reset();

	/* Wait until reset timeout or Reset IT signal */
	//for (; wDelay < pRadioConfiguration->Radio_Delay_Cnt_After_Reset; wDelay++);
	delay(100);
}

/*
 * Configuration
 */
void vRadio_Version()
{
	log_i("NSTDPREAMBLE_NOFLAG");
}

/*!
 *  Radio Initialization.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 */
void vRadio_Init(void)
{
	//U16 wDelay;

	/* Power Up the radio chip */
	vRadio_PowerUp();

	log_d("vRadio_Init");
	/* Load radio configuration */
	while (SI446X_SUCCESS != si446x_configuration_init(pRadioConfiguration->Radio_ConfigurationArray))
	{
		/* Error hook */
#if !(defined SILABS_PLATFORM_WMB912)
		// HBL LED4 = !LED4;
#else
		vCio_ToggleLed(eHmi_Led4_c);
#endif
		//for (wDelay = 0x7FFF; wDelay--; )
		log_d("vRadio_Init");
		delay(100);
		/* Power Up the radio chip */
		vRadio_PowerUp();
	}
	return;
	// Read ITs, clear pending ones
	si446x_get_int_status(0u, 0u, 0u);
}

/*!
 *  Check if Packet sent IT flag or Packet Received IT is pending.
 *
 *  @return   SI4455_CMD_GET_INT_STATUS_REP_PACKET_SENT_PEND_BIT / SI4455_CMD_GET_INT_STATUS_REP_PACKET_RX_PEND_BIT
 *
 *  @note
 *
 */
U8 bRadio_Check_Tx_RX()
{
	U8 rtn_u8;
	rtn_u8=0;
	//HBL if (radio_hal_NirqLevel()==LOW)
	{
		/* Read ITs, clear pending ones */
		si446x_get_int_status(0u, 0u, 0u);
		rtn_u8=Si446xCmd.GET_INT_STATUS.PH_PEND;
		log_d("PH_PEND=%#01x - MODEM_PEND=%#01x - CHIP_PEND=%#01x",Si446xCmd.GET_INT_STATUS.PH_PEND,Si446xCmd.GET_INT_STATUS.MODEM_PEND,Si446xCmd.GET_INT_STATUS.CHIP_PEND);

		/*
		 * ***********************
		 * Check for CHIP IT
		 * ***********************
		 */

		/*
		 * CHIP_PEND_CMD_ERROR
		 */
		if (Si446xCmd.GET_INT_STATUS.CHIP_PEND & SI446X_CMD_GET_CHIP_STATUS_REP_CHIP_PEND_CMD_ERROR_PEND_BIT)
		{
			log_d("CHIP_PEND_CMD_ERROR");
			/* State change to */
			si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);

			/* Reset FIFO */
			si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);

			/* State change to */
			si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_RX);
		}

		/*
		 * ***********************
		 * Check for MODEM IT
		 * ***********************
		 */

		/*
		 * MODEM_STATUS_PREAMBLE_DETECT / MODEM_STATUS_INVALID_PREAMBLE
		 */
		if (Si446xCmd.GET_INT_STATUS.MODEM_PEND & SI446X_CMD_GET_MODEM_STATUS_REP_MODEM_STATUS_PREAMBLE_DETECT_BIT)
		{
			log_d("MODEM_STATUS_PREAMBLE_DETECT");
		}

		if (Si446xCmd.GET_INT_STATUS.MODEM_PEND & SI446X_CMD_GET_MODEM_STATUS_REP_MODEM_STATUS_INVALID_PREAMBLE_BIT)
		{
			log_d("MODEM_STATUS_INVALID_PREAMBLE");
		}

		/*
		 * MODEM_STATUS_SYNC_DETECT / MODEM_STATUS_INVALID_SYNC
		 */
		if (Si446xCmd.GET_INT_STATUS.MODEM_PEND & SI446X_CMD_GET_MODEM_STATUS_REP_MODEM_STATUS_SYNC_DETECT_BIT)
		{
			log_d("MODEM_STATUS_SYNC_DETECT");
		}

		if (Si446xCmd.GET_INT_STATUS.MODEM_PEND & SI446X_CMD_GET_MODEM_STATUS_REP_MODEM_STATUS_INVALID_SYNC_BIT)
		{
			log_d("MODEM_STATUS_INVALID_SYNC");
		}

		/*
		 * ***********************
		 * Check for MODEM IT
		 * ***********************
		 */

		/*
		 * PH_STATUS_CRC_ERROR
		 */
		if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_STATUS_CRC_ERROR_BIT)
		{
			log_d("PH_STATUS_CRC_ERROR");
			/* Reset FIFO */
			si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);
		}

		/*
		 * PH_PEND_PACKET_SENT
		 */
		if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
		{
			log_d("PH_PEND_PACKET_SENT");
		}

		/*
		 * PH_PEND_PACKET_RX
		 */
		if(Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)
		{
			log_d("PH_PEND_PACKET_RX");
			/* Packet RX */

			/* Get payload length */
			si446x_fifo_info(0x00);

			log_d("RX_FIFO_COUNT: %d",Si446xCmd.FIFO_INFO.RX_FIFO_COUNT);

			//HBL si446x_read_rx_fifo(Si446xCmd.FIFO_INFO.RX_FIFO_COUNT,&customRadioPacket[0]);
		}
	}

	return rtn_u8;
}

/*!
 *  Set Radio to RX mode. .
 *
 *  @param channel Freq. Channel,  packetLength : 0 Packet handler fields are used , nonzero: only Field1 is used
 *
 *  @note
 *
 */
void vRadio_StartRX(U8 channel, U8 packetLenght )
{
	// Read ITs, clear pending ones
	si446x_get_int_status(0u, 0u, 0u);

	// Reset the Rx Fifo
	si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_BIT);

	/* Start Receiving packet, channel 0, START immediately, Packet length used or not according to packetLength */
	si446x_start_rx(channel, 0u, packetLenght,
			SI446X_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
			SI446X_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_READY,
			SI446X_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX );
}

/*!
 *  Set Radio to TX mode, variable packet length.
 *
 *  @param channel Freq. Channel, Packet to be sent length of of the packet sent to TXFIFO
 *
 *  @note
 *
 */
void vRadio_StartTx_Variable_Packet(U8 channel, U8 *pioRadioPacket, U8 length)
{

	/* Leave RX state */
	si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);

	/* Read ITs, clear pending ones */
	si446x_get_int_status(0u, 0u, 0u);

	/* Reset the Tx Fifo */
	si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_FIFO_TX_BIT);

	/* Fill the TX fifo with datas */
	si446x_write_tx_fifo(length, pioRadioPacket);

	/* Start sending packet, channel 0, START immediately */
	si446x_start_tx(channel, 0x80, length);
}
//HBL230425 }//SI4362_NSTDPREAMBLE_NOFLAG

