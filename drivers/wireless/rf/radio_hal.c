/*!
 * File:
 *  radio_hal.c
 *
 * Description:
 *  This file contains RADIO HAL.
 *
 * Silicon Laboratories Confidential
 * Copyright 2011 Silicon Laboratories, Inc.
 */

                /* ======================================= *
                 *              I N C L U D E              *
                 * ======================================= */

#include "bsp.h"

/*
 * HBL230425
int8_t SI4463_SEL_PIN=0xFF;
int8_t SI4463_IRQ_PIN=0xFF;
int8_t SI4463_SDN_PIN=0xFF;
SPIClass * VSPI2 = NULL;
*/


                /* ======================================= *
                 *          D E F I N I T I O N S          *
                 * ======================================= */

                /* ======================================= *
                 *     G L O B A L   V A R I A B L E S     *
                 * ======================================= */

                /* ======================================= *
                 *      L O C A L   F U N C T I O N S      *
                 * ======================================= */

                /* ======================================= *
                 *     P U B L I C   F U N C T I O N S     *
                 * ======================================= */

void radio_hal_AssertShutdown(void)
{
#if (defined SILABS_PLATFORM_RFSTICK) || (defined SILABS_PLATFORM_LCDBB) || (defined SILABS_PLATFORM_WMB)
  RF_PWRDN = 1;
#else
  // HBL PWRDN = 1;
  //if (SI4463_SDN_PIN!=0xFF) digitalWrite(SI4463_SDN_PIN,HIGH); // Shutdown
  //HBL230425 if (SI4463_SDN_PIN!=0xFF) dacWrite(SI4463_SDN_PIN,255); // Shutdown
#endif
}

void radio_hal_DeassertShutdown(void)
{
#if (defined SILABS_PLATFORM_RFSTICK) || (defined SILABS_PLATFORM_LCDBB) || (defined SILABS_PLATFORM_WMB)
  RF_PWRDN = 0;
#else
  // HBL PWRDN = 0;
  //if (SI4463_SDN_PIN!=0xFF) digitalWrite(SI4463_SDN_PIN,LOW); // In duty
  //HBL230425 if (SI4463_SDN_PIN!=0xFF) dacWrite(SI4463_SDN_PIN,0); // In duty
#endif
}

/*
 * Select
 */
void radio_hal_ClearNsel(void)
{
    // HBL RF_NSEL = 0
	//HBL230425 VSPI2->beginTransaction(SPISettings(VSPICLK,MSBFIRST,SPI_MODE0));
	//HBL230425 SPI_SELECT(SI4463_SEL_PIN);
}

/*
 * Deselect
 */
void radio_hal_SetNsel(void)
{
    // HBL RF_NSEL = 1;
	//HBL230425 SPI_DESELECT(SI4463_SEL_PIN);
	//SPI.endTransaction();
	//HBL230425 VSPI2->endTransaction();
}

BIT radio_hal_NirqLevel(void)
{
	//HBL230425 return digitalRead(SI4463_IRQ_PIN);
	return 0;
}

void radio_hal_SpiWriteByte(U8 byteToWrite)
{
  SpiReadWrite(byteToWrite);
}

U8 radio_hal_SpiReadByte(void)
{
  return SpiReadWrite(0xFF);
}

void radio_hal_SpiWriteData(U8 byteCount, U8* pData)
{
  SpiWriteData(byteCount, pData);
}

void radio_hal_SpiReadData(U8 byteCount, U8* pData)
{
  SpiReadData(byteCount, pData);
}

