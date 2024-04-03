/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

 /*****************************************************************************
 * Disclaimer:
 * This is beta version of BMS code. We are only providing it for Basic Measurements. 
 * Customer need to do modification according to their Applications.
 * ******************************************************************************/

/**
 * @file    main.c
 * @brief   Basic_Measurement_of_BMS_IC
 * @details This example uses SPI for basic measurements of BMS IC
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mxc_device.h"
#include "spi.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "board.h"
#include "ADBMSCOMMON.h"

/***** Initialization of MCU *****/
#define 	SPI               	MXC_SPI0	//SPI 0 
/*
typedef enum {
    SPI_MODE_0, ///< clock phase = 0, clock polarity = 0
    SPI_MODE_1, ///< clock phase = 0, clock polarity = 1
    SPI_MODE_2, ///< clock phase = 1, clock polarity = 0
    SPI_MODE_3, ///< clock phase = 1, clock polarity = 1
} mxc_spi_mode_t;
*/
int main(void)
{
	/* Start - Microcontroller Initialization */
	MXC_SPI_Init(SPI, Master_Mode, Mono_Mode, Slave_1, SS_Polarity, SPI_SPEED);
	MXC_SPI_SetDataSize(SPI, 8);			//Set Datasize of rxdata and txdata
	MXC_SPI_SetMode(SPI, SPI_MODE_0);		//Mode 0 SPI for BMS IC

	/* ADBMS6830 */
	adbmsmain();

    return 0;
}

void SPI_Transaction(uint8_t *tx_dat, uint8_t *rx_dat, uint8_t length)
{
  mxc_spi_req_t req;
  req.spi = SPI;
  req.txData = tx_dat; 		//Tx_Buffer
  req.rxData = rx_dat; 		//Rx_Buffer
  req.txLen = length;
  req.rxLen = length;
  req.ssIdx = 0; 			// Chip select index = 0 (if only one device is connected)
  req.txCnt = 0; 			// Initialise transmit counter to track no. of bytes transmitted
  req.rxCnt = 0; 			// Initialise receive counter to track no. of bytes received
  req.ssDeassert = 1; 		// Chip select to high after transaction is complete
  MXC_SPI_MasterTransaction(&req);
}

/* 	ISO SPI will go to IDLE after 4.3ms. So, Wake isoSPI up from IDlE state and enters the READY state and
	BMS IC will go to sleep state after 2s. So, Wake BMS IC from Sleep state to Standby state					*/
int WakeupIC(mxc_spi_regs_t *spi)
{
	int TOTAL_IC = 1;
	for(int i=0; i<TOTAL_IC; i++)
	{	
		/* Wake BMS IC from Sleep state to Standby state */ 
		spi->ctrl2 |= (MXC_S_SPI_CTRL2_SS_POL_SS0_HIGH);
		MXC_Delay(1000);
		spi->ctrl2 &= ~(MXC_S_SPI_CTRL2_SS_POL_SS0_HIGH);
		MXC_Delay(1000);
		/* Wake isoSPI up from IDlE state and enters the READY state */
		uint8_t a[1] = {0xff};
		SPI_Write(TOTAL_IC, &a[0]);	//Guarantees the isoSPI will be in ready state
		MXC_Delay(200);
	}
		return E_NO_ERROR;
}

void WakeupBMSIC(void)
{
	WakeupIC(SPI);
}
