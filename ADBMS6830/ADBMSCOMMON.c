/*
 * ADBMSCOMMON.c
 *
 *  Created on: Oct 7, 2023
 *      Author: HMuley
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ADBMSCOMMON.h>
#include "mxc_delay.h"

/*!
  @brief Write Dummy data Function to wakeup ISOSPI
*/
void SPI_Write(uint8_t tIC, uint8_t cmd_arg[0])
{
  uint8_t tx_data[1];//2 byte cmd + 2byte cmd pec
  uint8_t rx_data[1];//2 byte cmd + 2byte cmd pec
  tx_data[0] = cmd_arg[0];
  for(int i=0; i<tIC; i++)
  {
   SPI_Transaction(tx_data, rx_data, 1);
   #if Test_Print
        printf("\n%x\n",tx_data[0]);
   #endif
  }
}

/*!
  @brief Write Command Function 
*/
void ADBMS_Write_Cmd(uint8_t tIC, uint8_t cmd_arg[2])
{
  int Pec_Size = 2, Cmd_Size =2;
  int WRCmdSize = Cmd_Size + Pec_Size;
  uint16_t cmd_pec;
  uint8_t tx_data[WRCmdSize];//2 byte cmd + 2byte cmd pec
  uint8_t rx_data[WRCmdSize];//2 byte cmd + 2byte cmd pec
  tx_data[0] = cmd_arg[0];
  tx_data[1] = cmd_arg[1];
  cmd_pec = Pec15_Calc(Pec_Size, cmd_arg); //2byte cmd pec
  tx_data[2] = (uint8_t)(cmd_pec >> 8);
  tx_data[3] = (uint8_t)(cmd_pec);
  WakeupBMSIC();
  SPI_Transaction(tx_data, rx_data, WRCmdSize);
  #if Test_Print
      printf("\nCommand\n");
      printf("0x%x\t0x%x\t0x%x\t0x%x\n",tx_data[0], tx_data[1],tx_data[2], tx_data[3]);
  #endif
}

/*!
  @brief Write data Function 
*/
void ADBMS_Write_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *data_Write)
{
  int Cmd_Size = 2, Cmd_Pec_Size = 2, Data_Size = 6, Data_Pec_Size = 2;
  int WRCmdSize = Cmd_Size + Cmd_Pec_Size;
  int txdatabuff = WRCmdSize;
  int WRDataSize = (Data_Size + Data_Pec_Size)*tIC;
  int Reg_Size = WRCmdSize + WRDataSize;
  uint16_t cmd_pec, data_pec;
  uint8_t *tx_data;
  tx_data = (uint8_t*)calloc(Reg_Size, sizeof(uint8_t));//2 byte cmd + 2byte cmd pec + (x Byte data+ 2byte data pec)*tIC 
  uint8_t rx_data[Reg_Size];//2 byte cmd + 2byte cmd pec + 6 Byte data + 2byte data pec
  tx_data[0] = cmd_arg[0];
  tx_data[1] = cmd_arg[1];
  cmd_pec = Pec15_Calc(Cmd_Size, cmd_arg); //2byte cmd
  tx_data[2] = (uint8_t)(cmd_pec >> 8);
  tx_data[3] = (uint8_t)(cmd_pec);
  /*Executes for each ADBMS6xxx, loop starts with the last IC on the stack
    The first configuration written is received by the last IC in the chain*/
  for(int IC=tIC; IC>0; IC--)
  {
   for(int i=0; i<(Data_Size); i++)
   {
     tx_data[(txdatabuff)] = data_Write[i];
     txdatabuff++;
   }
   data_pec = pec10_calc(true, Data_Size, data_Write);
   tx_data[(txdatabuff)] = (uint8_t)(data_pec >> 8);
   txdatabuff++;
   tx_data[(txdatabuff)] = (uint8_t)(data_pec);
   txdatabuff++;
  }
  WakeupBMSIC();
  SPI_Transaction(tx_data, rx_data, Reg_Size);
  MXC_Delay(1000);
  #if Test_Print
  	  	txdatabuff = WRCmdSize;
        printf("\nWrite Data\n");
        for(int IC=1; IC<=tIC; IC++)
        {
          printf("IC:0%x\t",IC);
          for(int i=0; i<(Data_Size + Data_Pec_Size); i++)
          {
            printf("0x%x\t",tx_data[txdatabuff]);
            txdatabuff++;
          }
          printf("\n");
        }
  #endif
  free(tx_data);
}

/*!
  @brief Read data Function
*/
void ADBMS_Read_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *data_Read, RD_DATA_SIZE_ALL_TYPE type)
{
  int  RDDataSize;//6 Byte data + 2byte data pec
  if (type == RD_CV_ALL)			{RDDataSize = RDCVALL_SIZE;}
  else if (type == RD_AC_ALL)		{RDDataSize = RDACVALL_SIZE;}
  else if (type == RD_S_ALL)		{RDDataSize = RDSVALL_SIZE;}
  else if (type == RD_FC_ALL)		{RDDataSize = RDFCVALL_SIZE;}
  else if (type == RD_AUX_ALL)		{RDDataSize = RDAUXALL_SIZE;}
  else if (type == RD_C_S_ALL)		{RDDataSize = RDCSALL_SIZE;}
  else if (type == RD_AC_S_ALL)	{RDDataSize = RDACSALL_SIZE;}
  else if (type == RD_STA_ALL)		{RDDataSize = RDSTAALL_SIZE;}
  else if (type == RD_C_CFG_ALL)	{RDDataSize = RDCCFGALL_SIZE;}
  else								{RDDataSize = 8;}//6 Byte data + 2byte data pec
  int Cmd_Size = 2, Cmd_Pec_Size = 2, Data_Pec_Size =2;
  int WRCmdSize = Cmd_Size + Cmd_Pec_Size;
  int Reg_Size = WRCmdSize + RDDataSize*tIC;
  uint16_t cmd_pec;
  uint8_t *rx_data, *copyArray;
  copyArray = (uint8_t *)calloc(RDDataSize*tIC, sizeof(uint8_t)); //For Data pec Calculation and Verification
  rx_data = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));         //2 byte cmd + 2byte cmd pec + x Byte data + 2byte data pec
  uint8_t tx_data[Reg_Size];                                      //2 byte cmd + 2byte cmd pec + x Byte data + 2byte data pec
  tx_data[0] = cmd_arg[0];
  tx_data[1] = cmd_arg[1];
  cmd_pec = Pec15_Calc(Cmd_Size, cmd_arg); //2byte cmd
  tx_data[2] = (uint8_t)(cmd_pec >> 8);
  tx_data[3] = (uint8_t)(cmd_pec);
  WakeupBMSIC();  // To wakeup IsoSPI from IDLE to active
  SPI_Transaction(tx_data, rx_data, Reg_Size);
  MXC_Delay(1000);  //ADBMS will take 3.5ms time to read config register //This dalay in not required for maxim microcontroller due to it's spi driver code. but it may be important for other microcontroller.
  #if Test_Print
  	  	 int rxdatabuff = WRCmdSize;
     	 printf("\nData Read From BMS IC\n");
     	 for(int IC=1; IC<=tIC; IC++)
     	 {
     		 printf("IC:0%x\t",IC);
     		 for (int i=0; i<(RDDataSize); i++)
     		 {
     			 printf("0x%x\t", rx_data[rxdatabuff]); // (inital 4 byte - invalid data - SPI Full Duplex)
     			 rxdatabuff++;
     		 }
     		 printf("\n");
     	 }
  #endif
  /* executes for each ic in the daisy chain and packs the data */
  for (uint8_t current_ic = 0; current_ic < tIC; current_ic++)
  {
    for (uint8_t data_byte = 0; data_byte < (RDDataSize-2); data_byte++)
     {
       data_Read[data_byte + (current_ic*(RDDataSize-2))] = rx_data[data_byte + WRCmdSize + (current_ic)*RDDataSize]; // Copy received Data only
     }
    for (uint8_t data_byte = 0; data_byte < (RDDataSize); data_byte++)
     {
       copyArray[data_byte + (current_ic*RDDataSize)] = rx_data[data_byte + WRCmdSize + (current_ic)*RDDataSize]; // Copy received Data + Data pec for data pec calc and Verification
     }
  
    uint16_t Received_Data_Pec, Cal_Data_Pec;
    int Reg_Curr_Size = WRCmdSize + (current_ic+1)*RDDataSize;
    #if Test_Print
        uint8_t Received_Command_Cntr;
        Received_Command_Cntr 	= rx_data[Reg_Curr_Size - 2U] >> 2U;
    #endif
    Received_Data_Pec 		= ((rx_data[( Reg_Curr_Size - 2U)] & 3U) << 8U) | (rx_data[( Reg_Curr_Size - 1U)]);
    Cal_Data_Pec				  =  pec10_calc(true, (RDDataSize-Data_Pec_Size), &copyArray[(current_ic*RDDataSize)]);
    #if Test_Print
        printf("Received_Command_Cntr=%x\t Received_Data_Pec=%x\t Cal_Data_Pec=%x\n", Received_Command_Cntr, Received_Data_Pec, Cal_Data_Pec);
    #endif
    if (Received_Data_Pec != Cal_Data_Pec)
    {
      printf("\nError in data Received From the ADBMS Board\n");
    }
  }
  free(copyArray);
  free(rx_data);
}

/*!
  @brief Precalculated CRC Table
*/
const uint16_t Crc15Table[256] =
{
  0x0000,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
  0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
  0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
  0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
  0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
  0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
  0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
  0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
  0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
  0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
  0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
  0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
  0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
  0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
  0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
  0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
  0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
  0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
  0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
  0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
  0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
  0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
  0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

/*!
  @brief CRC15 Pec Calculation Function
*/
uint16_t Pec15_Calc( uint8_t len, uint8_t *data)
{
  uint16_t remainder,addr;
  remainder = 16;                /* initialize the PEC */
  for (uint8_t i = 0; i<len; i++)/* loops for each byte in data array */
  {
    addr = (((remainder>>7)^data[i])&0xff);/* calculate PEC table address */
    remainder = ((remainder<<8)^Crc15Table[addr]);
  }
  return(remainder*2);/* The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
}

/*!
  @brief CRC10 Pec Calculation Function
*/
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data)
{
  uint16_t remainder = 16; /* PEC_SEED;   0000010000 */
  uint16_t polynom = 0x8F; /* x10 + x7 + x3 + x2 + x + 1 <- the CRC15 polynomial         100 1000 1111   48F */

  /* Perform modulo-2 division, a byte at a time. */
  for (uint8_t pbyte = 0; pbyte < len; ++pbyte)
  {
    /* Bring the next byte into the remainder. */
    remainder ^= (uint16_t)(data[pbyte] << 2);
    /* Perform modulo-2 division, a bit at a time.*/
    for (uint8_t bit_ = 8; bit_ > 0; --bit_)
    {
      /* Try to divide the current data bit. */
      if ((remainder & 0x200) > 0)//equivalent to remainder & 2^14 simply check for MSB
      {
        remainder = (uint16_t)((remainder << 1));
        remainder = (uint16_t)(remainder ^ polynom);
      }
      else
      {
        remainder = (uint16_t)(remainder << 1);
      }
    }
  }
  if (rx_cmd == true)
  {
    remainder ^= (uint16_t)((data[len] & 0xFC) << 2);
    /* Perform modulo-2 division, a bit at a time */
    for (uint8_t bit_ = 6; bit_ > 0; --bit_)
    {
      /* Try to divide the current data bit */
      if ((remainder & 0x200) > 0)//equivalent to remainder & 2^14 simply check for MSB
      {
        remainder = (uint16_t)((remainder << 1));
        remainder = (uint16_t)(remainder ^ polynom);
      }
      else
      {
        remainder = (uint16_t)((remainder << 1));
      }
    }
  }
  return ((uint16_t)(remainder & 0x3FF));
}

/*!
  @brief Combine Cell Voltage in 16 bit Hex format
*/
int16_t twos_complement_to_int(uint16_t value, uint8_t num_bits)
{
  int16_t result = value; //result is returned as-is if MSB is 0
  if (value & (1 <<(num_bits-1)))//if MSB is 1(negative)
  {
    result = value -(1 <<num_bits); //subtract the overflow from 2's complement value
  }
    return result;
}

/*!
  @brief Function: SetUnderVoltageThreshold fo ADBMS
*/
uint16_t SetUnderVoltageThreshold(double voltage)
{
  uint16_t vuv_value;
  uint8_t rbits = 12;
  voltage = (voltage - (double)1.5);
  voltage = voltage / ((double)16 * (double)0.000150);
  vuv_value = (uint16_t )(voltage + 2 * (1 << (rbits - 1)));
  vuv_value &= 0xFFF;
  return vuv_value;
}

/*!
  @brief Function: SetOverVoltageThreshold for ADBMS
*/
uint16_t SetOverVoltageThreshold(double voltage)
{
  uint16_t vov_value;
  uint8_t rbits = 12;
  voltage = (voltage - (double)1.5);
  voltage = voltage / ((double)16 * (double)0.000150);
  vov_value = (uint16_t )(voltage + 2 * (1 << (rbits - 1)));
  vov_value &= 0xFFF;
  return vov_value;
}

/*!
  @brief Write UV and OV in CFGB Reg
*/
void WRCFGB_data_Set(uint8_t tIC, uint8_t cmd_arg[], float UV_THSD, float OV_THSD, uint8_t *data_Write)
{
  uint16_t UV_data, OV_data;
  UV_data = SetUnderVoltageThreshold(UV_THSD);
  data_Write[0] = (uint8_t)(UV_data);
  data_Write[1] = (uint8_t)((UV_data | 0xFF)>>8);
  OV_data = SetOverVoltageThreshold(OV_THSD);
  data_Write[1] |= (uint8_t)(OV_data<<4);
  data_Write[2] = (uint8_t)(OV_data>>4);
  data_Write[3] = 0x00;
  data_Write[4] = 0x00;
  data_Write[5] = 0x00;
}
