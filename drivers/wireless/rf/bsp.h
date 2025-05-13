/*! @file bsp.h
 * @brief This file contains application specific definitions and includes.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include <debug.h>



/*
 * -------------------
 * SPI
 * -------------------
 * cf Bus_SPI.h
 * M5GFX.cpp
 *   Line 592 - SPI board_M5Tough
 * 	   bus_cfg.pin_mosi = 23;
 *     bus_cfg.pin_miso = 38;
 *     bus_cfg.pin_sclk = 18;
 *     bus_cfg.pin_dc   = 15;
 *     bus_cfg.spi_3wire = true;
 * 	   bus_cfg.freq_write = 40000000;
 *     bus_cfg.freq_read  = 15000000;
 *   Line 674 - I2C board_M5Tough
 *     fg.pin_int  = 39;   // INT pin number
 *     cfg.pin_sda  = 21;   // I2C SDA pin number
 *     cfg.pin_scl  = 22;   // I2C SCL pin number
 *     cfg.i2c_addr = 0x2E; // I2C device addr
 * CLK:
 *    Must divide 80MHz
 *    MAX SI446X 10Mhz
 *    SAMD21 1MHZ validated
 *    ESP32 4MHz, 5MHz, 10MHz validated
 */
#define VSPICLK5S 40000000
#define VSPICLK 8000000
#define VSPI_SCLK GPIO_NUM_18
#define VSPI_MISO   GPIO_NUM_38
#define VSPI_MOSI   GPIO_NUM_23

/*extern int8_t SI4463_IRQ_PIN;
extern int8_t SI4463_SDN_PIN;
extern int8_t  SI4463_SEL_PIN;
extern SPIClass * VSPI2;*/

/*
 *-------------------
 * si446x_nirq_process
 *-------------------
 *   HBL
 */
void si446x_nirq_process(void);

/*------------------------------------------------------------------------*/
/*            Application specific global definitions                     */
/*------------------------------------------------------------------------*/
/*! Platform definition */
/* Note: Plaform is defined in Silabs IDE project file as
 * a command line flag for the compiler. */
//#define SILABS_PLATFORM_WMB930

/*! Extended driver support 
 * Known issues: Some of the example projects 
 * might not build with some extended drivers 
 * due to data memory overflow */
#define RADIO_DRIVER_EXTENDED_SUPPORT
#define RADIO_DRIVER_FULL_SUPPORT

#undef  SPI_DRIVER_EXTENDED_SUPPORT
#undef  HMI_DRIVER_EXTENDED_SUPPORT
#undef  TIMER_DRIVER_EXTENDED_SUPPORT
#undef  UART_DRIVER_EXTENDED_SUPPORT

/*
 * <HBL>
 */
#define SILABS_RADIO_SI446X
#define U8 uint8_t
#define U16 uint16_t
#define S16 int16_t
#define U32 uint32_t
#ifdef BIT
#undef BIT
#endif
#define BIT uint8_t
#define FALSE 0

#define SEGMENT_VARIABLE(name,type,seg) type name
#define SEGMENT_VARIABLE_SEGMENT_POINTER(name,type,seg1,seg2) type name

//HBL050525 #define SPI_SELECT(nSELPin)(digitalWrite(nSELPin,LOW))
//HBL050525 #define SPI_DESELECT(nSELPin)(digitalWrite(nSELPin,HIGH))

/*****************************************************************************
 *  Global Macros & Definitions
 *****************************************************************************/
/*! Maximal packet length definition (FIFO size) */
#define RADIO_MAX_PACKET_LENGTH     64u

/*****************************************************************************
 *  Global Typedefs & Enums
 *****************************************************************************/
typedef struct
{
    uint8_t   *Radio_ConfigurationArray;

    uint8_t   Radio_ChannelNumber;
    uint8_t   Radio_PacketLength;
    uint8_t   Radio_State_After_Power_Up;

    uint16_t  Radio_Delay_Cnt_After_Reset;

    uint8_t   Radio_CustomPayload[RADIO_MAX_PACKET_LENGTH];
} tRadioConfiguration;

/*****************************************************************************
 *  Global Variable Declarations
 *****************************************************************************/
//HBL040525 extern const SEGMENT_VARIABLE_SEGMENT_POINTER(pRadioConfiguration, tRadioConfiguration*, SEG_CODE, SEG_CODE);
//HBL040525 extern SEGMENT_VARIABLE(customRadioPacket[RADIO_MAX_PACKET_LENGTH], uint8_t, SEG_XDATA);

/*! Si446x configuration array */
extern const SEGMENT_VARIABLE(Radio_Configuration_Data_Array[], uint8_t, SEG_CODE);



/*
 * ----------------------------------
 *   Writes one byte to the SPI peripheral
 *   and reads one byte
 *   at the same time
 * ----------------------------------
 */
inline U8 SpiReadWrite(U8 byte)
{
	//return (U8)SPI.transfer(byte);
	//HBL230425 return (U8)VSPI2->transfer(byte);
	return 0;
};

inline void SpiWriteData(U8 byteCount,U8* pData)
{
	//HBL230425 VSPI2->writeBytes(pData,byteCount);

	//printf2("SpiWriteData-pData=");
	/*{
		for(U8 i=0;i<byteCount;i++)
		{
			//SPI.transfer(pData[i]);
			VSPI2->transfer(pData[i]);
			//printf2("%#04x ",pData[i]);
		}
		//printf2("\n");
	}*/
};

inline void SpiReadData(U8 byteCount,U8* pData)
{
	//HBL230425 VSPI2->transferBytes(0, pData, byteCount);

	/*
	if(VSPI2->transfer(0xFF) == 0xFF)
	{
		// Get response data
		//printf2("SpiReadData-pData=");
		for(uint8_t i=0;i<byteCount;i++) {
			//pData[i] = SPI.transfer(0xFF);
			pData[i] = VSPI2->transfer(0xFF);
			//printf2("%#04x ",pData[i]);
		}
		//printf2("\n");
	}
	*/
};




/*
 * </HBL>
 */


/*------------------------------------------------------------------------*/
/*            Application specific includes                               */
/*------------------------------------------------------------------------*/

//#include "drivers\compiler_defs.h"
//#include "platform_defs.h"
#include "hardware_defs.h"

//#include "Si446x\application\application_defs.h"
//#include "application_defs.h"

//#include "drivers\cdd_common.h"
//#include "drivers\control_IO.h"
//#include "drivers\smbus.h"
//#include "drivers\spi.h"
//#include "drivers\hmi.h"
//#include "drivers\timer.h"
//#include "drivers\pca.h"
//#include "drivers\uart.h"
//#include "application\isr.h"

//#include "application\radio_config.h"
//#include <radio_config.h>
//#include "application\radio.h"
// HBL #include "radio.h"

//#include "application\sample_code_func.h"

#if ((defined SILABS_PLATFORM_LCDBB) || (defined SILABS_MCU_DC_EMIF_F930) || (defined SILABS_PLATFORM_WMB))
/* LCD driver includes */
#include "drivers\ascii5x7.h"
#include "drivers\dog_glcd.h"
#include "drivers\pictures.h"
#endif

//#include "drivers\radio\radio_hal.h"
#include "radio_hal.h"
//#include "drivers\radio\radio_comm.h"
#include "radio_comm.h"

#ifdef SILABS_RADIO_SI446X
/*
#include "drivers\radio\Si446x\si446x_api_lib.h"
#include "drivers\radio\Si446x\si446x_defs.h"
#include "drivers\radio\Si446x\si446x_nirq.h"
#include "drivers\radio\Si446x\si446x_patch.h"
 */

#include "si446x_api_lib.h"
#include "si446x_defs.h"
#include "si446x_nirq.h"
#include "si446x_patch.h"
#endif

#ifdef SILABS_RADIO_SI4455
#include "drivers\radio\Si4455\si4455_api_lib.h"
#include "drivers\radio\Si4455\si4455_defs.h"
#include "drivers\radio\Si4455\si4455_nirq.h"
#endif

#endif //BSP_H
