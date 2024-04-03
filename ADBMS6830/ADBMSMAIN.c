/*
 * ADBMSMAIN.c
 *
 *  Created on: Oct 7, 2023
 *      Author: HMuley
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "ADBMSCMDLIST.h"
#include "ADBMSCOMMON.h"

/***** Initialization of BMS IC *****/
uint8_t 	TOTAL_IC	=	  1;
uint8_t 	TOTAL_CELLS	=	  16;
static bool balanceFlags[16];

/* Set Under Voltage and Over Voltage Thresholds */
const double OV_THRESHOLD = 4.2;                 /* Volt */
const double UV_THRESHOLD = 3.0;                 /* Volt */

//Refon = 1 and All GPIO are OFF
uint8_t CFGA_data[6] = {0x81, 0x00, 0x00, 0xff, 0x03, 0x00};// 0x02, 0x8E
uint8_t CFGB_data[6];
//uint8_t tIC

void adbmsmain(void)
{
	printMenu();
	ADBMS_Init(TOTAL_IC);
	while(1)
	 {
	   int user_command;

	   scanf("%d", &user_command);
	   printf("Enter command:%d\n", user_command);
	   run_command(user_command);
	 }
}

void printMenu(void)
{ 
  printf("\nSPI Initialization\n");
	printf("Four Wire Full Duplex SPI \nCLK Speed = %d\n", SPI_SPEED);
  printf("SPI Mode = %d \nNo of Slaves = %d\nSlave Select Polarity = %d\n\n", SPI_MODE_0, Slave_1, SS_Polarity);
  printf("List of ADBMS6830 commands and their numbers:\n");
  printf("Command No. : Command description\n");
  printf("1           : Write and read configuration\n");
  printf("2           : Measure cell voltages\n");
  printf("3           : Measure s-channel voltages\n");
  printf("4           : Measure average cell voltages\n");
  printf("5           : Measure gpio pin voltages\n");
  printf("6           : Measure status register voltages\n");
  printf("\n");
  printf("Enter '0' for menu\n");
  printf("Please enter command: \n");
}


void run_command(int command)
{
 switch(command)
 {
 case 0:
    printMenu();
    break;

 case 1:
    ADBMS_Write_Read_Config(TOTAL_IC);
    break;

 case 2:
    ADC_Cell_Voltage_Measurement(TOTAL_IC);
    break;

  case 3:
    ADC_S_Voltage_Measurement(TOTAL_IC);
    break;

  case 4:
    ADC_AVG_Cell_Voltage_Measurement(TOTAL_IC);
    break;

  case 5:
	  ADC_GPIO_Voltage_Measurement(TOTAL_IC);
    break;

  case 6:
	  ADBMS_Status_Reg_voltage_measurment(TOTAL_IC);
    break;

 default:
    printf("Incorrect Option\n\n");
    break;
 }
}

/*!
  @brief Initialized  BMS IC 
*/
void ADBMS_Init(uint8_t TOTAL_IC)
{
  int Reg_Size =RECEIVE_RD_PACKET_SIZE;
	uint8_t *Data_Read;
	Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); 
	ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
	ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], GRPA); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC(); 
  WRCFGB_data_Set(TOTAL_IC, WRCFGB, UV_THRESHOLD, OV_THRESHOLD, &CFGB_data[0]);
	ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGB_data[0]); // Set UV and OV Threshold for all Cells
  ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPB);
  free(Data_Read);
}

/*!
  @brief Set Refon = 1, UV = 3V and OV = 4.2V in CFG Ref
*/
void ADBMS_Write_Read_Config(uint8_t TOTAL_IC)
{
  int Reg_Size =RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
  WakeupBMSIC(); 
	ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
	ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], GRPA); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  printf("\nWRCFGA\n");
  ADBMS_Print_WRCFG_Data(TOTAL_IC,  WRCFGA, CFGA_data, GRPA);
  printf("\nRDCFGA\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDCFGA, Data_Read, GRPA);
  WRCFGB_data_Set(TOTAL_IC, WRCFGB, UV_THRESHOLD, OV_THRESHOLD, &CFGB_data[0]);
	ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGB_data[0]); // Set UV and OV Threshold for all Cells
  ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPB);
  printf("\nWRCFGB\n");
  ADBMS_Print_WRCFG_Data(TOTAL_IC,  WRCFGB, CFGB_data, GRPB);
  printf("\nRDCFGB\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDCFGB, Data_Read, GRPB);
  free(Data_Read);
}

/*!
  @brief Measure C-ADC Voltage in Single shot mode
*/
void ADC_Cell_Voltage_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADCV(SINGLESHOT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, ADCV);
  ADBMS_Write_Cmd(TOTAL_IC, ADCV); //Send ADCV command with Single shot, No Redundancy, no discharge
  ADBMSPollADC(TOTAL_IC, PLCADC);  //Send poll ADC command to Check ADC Conversion
  //MXC_Delay(1100);               //We can use delay as C-ADC conversion completed after 1msec
  WakeupBMSIC();
  ADBMS_Read_Data(TOTAL_IC,   RDCVA, &Data_Read[0], GRPA); // Read Cell Voltages
  ADBMS_Print_Data(TOTAL_IC,  RDCVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDCVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDCVB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDCVC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDCVC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDCVD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDCVD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDCVE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDCVE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDCVF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDCVF, Data_Read, GRPF);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRCELL);                       //Clear cell Voltages
  ADBMS_Read_Data(TOTAL_IC,   RDCVA, &Data_Read[0], GRPA);  //read default value after clearcell command
  ADBMS_Print_Data(TOTAL_IC,  RDCVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDCVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDCVB, Data_Read, GRPB);
  free(Data_Read);
}

/*!
  @brief Measure S-ADC Voltage in Single shot mode
*/
void ADC_S_Voltage_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADSV(Singleshot_ADSV, NODISCHARGE_ADSV, ALL_CH_OW_OFF_ADSV, ADSV);
  ADBMS_Write_Cmd(TOTAL_IC, ADSV);
  ADBMSPollADC(TOTAL_IC, PLADC);
  //MXC_Delay(15000);
  ADBMS_Read_Data(TOTAL_IC,   RDSVA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDSVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDSVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDSVB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDSVC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDSVC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDSVD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDSVD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDSVE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDSVE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDSVF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDSVF, Data_Read, GRPF);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRSPIN);
  ADBMS_Read_Data(TOTAL_IC,   RDSVA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDSVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDSVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDSVB, Data_Read, GRPB);
  free(Data_Read);
}

/*!
  @brief Measure Averge C-ADC Voltage
*/
void ADC_AVG_Cell_Voltage_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADCV(CONT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, ADCV);
  ADBMS_Write_Cmd(TOTAL_IC, ADCV);
  pollVoltCTSconversion();
  ADBMS_Read_Data(TOTAL_IC,   RDACA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDACA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDACB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDACB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDACC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDACC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDACD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDACD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDACE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDACE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDACF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDACF, Data_Read, GRPF);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRCELL);
  free(Data_Read);
}

/*!
  @brief Measure GPIO Voltage
*/
void ADC_GPIO_Voltage_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADAX(OW_AUX_OFF_ADAX, ALL_ADAX, PULLDOWN_ADAX, ADAX);
  ADBMS_Write_Cmd(TOTAL_IC, ADAX);
  ADBMSPollADC(TOTAL_IC, PLAUX1);
  //MXC_Delay(20000); // 20msec delay to get data of all GPIO Regs
  ADBMS_Read_Data(TOTAL_IC,       RDAUXA, &Data_Read[0], GRPA);
  ADBMS_Print_AUX_Data(TOTAL_IC,  RDAUXA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,       RDAUXB, &Data_Read[0], GRPB);
  ADBMS_Print_AUX_Data(TOTAL_IC,  RDAUXB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,       RDAUXC, &Data_Read[0], GRPC);
  ADBMS_Print_AUX_Data(TOTAL_IC,  RDAUXC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,       RDAUXD, &Data_Read[0], GRPD);
  ADBMS_Print_AUX_Data(TOTAL_IC,  RDAUXD, Data_Read, GRPD);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRAUX);
  free(Data_Read);
}

/*!
  @brief Measure Status Voltage
*/
void ADBMS_Status_Reg_voltage_measurment(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
	uint8_t *Data_Read;
	Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
	ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
	ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
	ADBMS_ADAX(OW_AUX_OFF_ADAX, ALL_ADAX, PULLDOWN_ADAX, ADAX);
	ADBMS_Write_Cmd(TOTAL_IC, ADAX);
	ADBMSPollADC(TOTAL_IC, PLAUX1);
  //MXC_Delay(20000); // 20msec delay to get data of all GPIO Regs
  WakeupBMSIC();
  ADBMS_Read_Data(TOTAL_IC,         RDSTATA, &Data_Read[0], GRPA);
  ADBMS_Print_Status_Data(TOTAL_IC, RDSTATA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,         RDSTATB, &Data_Read[0], GRPB);
  ADBMS_Print_Status_Data(TOTAL_IC, RDSTATB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,         RDSTATC, &Data_Read[0], GRPC);
  ADBMS_Read_Data(TOTAL_IC,         RDSTATD, &Data_Read[0], GRPD);
  ADBMS_Read_Data(TOTAL_IC,         RDSTATE, &Data_Read[0], GRPE);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRAUX);
  ADBMS_Write_Cmd(TOTAL_IC, CLRFLAG);
  free(Data_Read);
}

/**************************Print Function********************************/

/*!
  @brief Print Write CNFG Reg Data
*/
void ADBMS_Print_WRCFG_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{ 
  int Data_Size = 6;
  for(int IC=1; IC<=tIC; IC++)
  {
    for(int i=0; i<(Data_Size); i++)
    {
      printf("0x%x\t",buff[i]);
    }
    printf("\n");
  }
}

/*!
  @brief Print Read CNFG Reg Data
*/
void ADBMS_Print_RDCFG_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{ 
  int Data_Size = 6;
  int buffdata = 0;
  for(int IC=1; IC<=tIC; IC++)
  {
    for(int i=0; i<(Data_Size); i++)
    {
      printf("0x%x\t",buff[buffdata]);
      buffdata++;
    }
    printf("\n");
  }
}

/*!
  @brief Print Cell , S, Avg , Fillterced cell voltages
*/
/*void ADBMS_Print_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  int16_t x, y, z; //to hold raw data
  uint8_t i = 0;
  for(uint8_t IC=0;IC<tIC;IC++)
  {
    x = twos_complement_to_int((buff[i+1]<< 8)|buff[i],16);//Combine lower and upper bytes
    y = twos_complement_to_int((buff[i+3]<< 8)|buff[i+2],16);
    z = twos_complement_to_int((buff[i+5]<< 8)|buff[i+4],16);
    if			  ( type == GRPA){printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if		( type == GRPB){printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if 	( type == GRPC){printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if 	( type == GRPD){printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if 	( type == GRPE){printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if 	( type == GRPF){printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x));}
    else	    {printf("error");}
    i +=6;
  }
}*/
void ADBMS_Print_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)

{

	int16_t x=0, y=0, z=0; //to hold raw data

	uint8_t i=0,j = 0;

	int16_t value[3];
	double voltage[4][16];


	for(uint8_t IC=0;IC<tIC;IC++)//need to change

	{
		x = twos_complement_to_int((buff[i+1]<< 8)|buff[i],16);//Combine lower and upper bytes
		y = twos_complement_to_int((buff[i+3]<< 8)|buff[i+2],16);
		z = twos_complement_to_int((buff[i+5]<< 8)|buff[i+4],16);

		if			( type == GRPA){printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
		else if		( type == GRPB){printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
		else if 	( type == GRPC){printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
		else if 	( type == GRPD){printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
		else if 	( type == GRPE){printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
		else if 	( type == GRPF){printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x));}
		else	    {printf("error");}
	//	i +=6;

	//}

	//for(uint8_t IC=0;IC<tIC;IC++)

	//{

		value[0] = x;
		value[1] = y;
		value[2] = z;
		uint8_t idx = 0;	//voltage index
		if( type == GRPA)

		{
			printf("looking for voltage");
			printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));
			for(i=0;i<3;i++)
			{
				voltage[IC][i]=ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(value[idx]);
				printf("Voltage at IC %d, i %d: %f\n", IC, i, voltage[IC][i]);
				idx++;
				WakeupBMSIC(); //Wakeup BMS IC and IsoSPI

				//printf("value of GRPA %u",idx);
				ADBMS6830_EnableCellBalancing(TOTAL_IC,voltage);
			}
		}

		else if	( type == GRPB)
		{
			//printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));
			for(i=3;i<6;i++)
			{
				voltage[IC][i]=ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(value[idx]);
				printf("Voltage at IC %d, i %d: %f\n", IC, i, voltage[IC][i]);
				idx++;
				//printf("value of GRPB %u",idx);
				 WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
				 ADBMS6830_EnableCellBalancing(TOTAL_IC,voltage);
			}
		}
		else if ( type == GRPC)
		{
			//printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));
			for(i=6;i<9;i++)
			{
				voltage[IC][i]=ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(value[idx]);
				printf("Voltage at IC %d, i %d: %f\n", IC, i, voltage[IC][i]);
				idx++;
				//printf("idx%u",idx);
				//printf("value of GRPC %u",idx);
				WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
				ADBMS6830_EnableCellBalancing(TOTAL_IC,voltage);
			}
		}
		else if( type == GRPD)
		{
			//printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));
			for(i=9;i<12;i++)
			{
				voltage[IC][i]=ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(value[idx]);
				printf("Voltage at IC %d, i %d: %f\n", IC, i, voltage[IC][i]);
				idx++;
				//printf("value of GRPD %u",idx);
				WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
				ADBMS6830_EnableCellBalancing(TOTAL_IC,voltage);
			}
		}
		else if ( type == GRPE)

		{
			//printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));
			for(i=12;i<15;i++)
			{
				voltage[IC][i]=ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(value[idx]);
				printf("Voltage at IC %d, i %d: %f\n", IC, i, voltage[IC][i]);
				idx++;
				//printf("value of GRPE %u",idx);
				WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
				ADBMS6830_EnableCellBalancing(TOTAL_IC,voltage);
			}
		}
		else if( type == GRPF)

		{
			//printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x));
			for(i=15;i<16;i++)
			{
				voltage[IC][i]=ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(value[idx]);
				printf("Voltage at IC %d, i %d: %f\n", IC, i, voltage[IC][i]);
				idx++;
				//printf("value of GRPF %u",idx);
				WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
				ADBMS6830_EnableCellBalancing(TOTAL_IC,voltage);
			}
		}
		else
		{
			printf("error");
			idx=0;
			 WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
		}
		j+=6;
		ADSV_PWM_discharge_Measurement_test();

	}
}
/*!
  @brief Print GPIO voltages
*/
void ADBMS_Print_AUX_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  int16_t x, y, z; //to hold raw data
  uint8_t i = 0;
  for(uint8_t IC=0;IC<tIC;IC++)
  {
    x = twos_complement_to_int((buff[i+1]<< 8)|buff[i],16);//Combine lower and upper bytes
    y = twos_complement_to_int((buff[i+3]<< 8)|buff[i+2],16);
    z = twos_complement_to_int((buff[i+5]<< 8)|buff[i+4],16);
    if			  ( type == GRPA){printf("\nIC:0%x->\t  GPIO1  =%1.3lf\t GPIO2 =%1.3lf\t GPIO3 =%1.3lf\n",              (IC+1), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(y),  ADBMS_CONVERT_GPIO_HEX_TO_VOLT(z));}
    else if		( type == GRPB){printf("\nIC:0%x->\t  GPIO4  =%1.3lf\t GPIO5 =%1.3lf\t GPIO6 =%1.3lf\n",              (IC+1), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(y),  ADBMS_CONVERT_GPIO_HEX_TO_VOLT(z));}
    else if 	( type == GRPC){printf("\nIC:0%x->\t  GPIO7  =%1.3lf\t GPIO8 =%1.3lf\t GPIO9 =%1.3lf\n",              (IC+1), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(y),  ADBMS_CONVERT_GPIO_HEX_TO_VOLT(z));}
    else if 	( type == GRPD){printf("\nIC:0%x->\t  GPIO10 =%1.3lf\t VM  = %1.3lf\t  Total Cell Voltage = %1.3lf\n",(IC+1), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x), ADBMS_CONVERT_VM_HEX_TO_VOLT(y),    ADBMS_CONVERT_VP_HEX_TO_VOLT(z));}
    else	    {printf("error");}
    i +=6;
  }
}

/*!
  @brief Print Status Reg data
*/
void ADBMS_Print_Status_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  int16_t x, y, z; //to hold raw data
  uint8_t i = 0;
  for(uint8_t IC=0;IC<tIC;IC++)
  {
    x = twos_complement_to_int((buff[i+1]<< 8)|buff[i],16);//Combine lower and upper bytes
    y = twos_complement_to_int((buff[i+3]<< 8)|buff[i+2],16);
    z = twos_complement_to_int((buff[i+5]<< 8)|buff[i+4],16);
    if			  ( type == GRPA){printf("\nIC:0%x->\t  VREF2     = %1.3lf\t IC Dia Temperature = %1.3lf\n", (IC+1), ADBMS_CONVERT_VREF2_HEX_TO_VOLT(x), ADBMS_CONVERT_ITEMP_HEX_TO_VOLT(y));}
    else if		( type == GRPB){printf("\nIC:0%x->\t  VDigital  = %1.3lf\t VAnalog  = %1.3lf\t VRES = %1.3lf\n", (IC+1), ADBMS_CONVERT_VD_HEX_TO_VOLT(x), ADBMS_CONVERT_VA_HEX_TO_VOLT(y), ADBMS_CONVERT_VRES_HEX_TO_VOLT(z));}
    else	    {printf("error");}
    i +=6;
  }
}

/*!
  @brief Check Singleshot ADC conversion
*/
//To check ADC(Single shot) conversion
//Single shot mode only
//Not applicable to Continuous mode
void ADBMSPollADC(uint8_t tIC, uint8_t cmd_arg[2])
{
  int Pec_Size = 2, Cmd_Size =2, Check_ADC_Conversion = 1;
  int WRCmdSize = Cmd_Size + Pec_Size + Check_ADC_Conversion;
  uint16_t cmd_pec;
  uint8_t tx_data[WRCmdSize];//2 byte cmd + 2byte cmd pec
  uint8_t rx_data[WRCmdSize];//2 byte cmd + 2byte cmd pec
  //WRCmdPec(cmd_arg, tx_data);
  tx_data[0] = cmd_arg[0];
  tx_data[1] = cmd_arg[1];
  cmd_pec = Pec15_Calc(Pec_Size, cmd_arg); //2byte cmd pec
  tx_data[2] = (uint8_t)(cmd_pec >> 8);
  tx_data[3] = (uint8_t)(cmd_pec);
  WakeupBMSIC();
  do
  {
  SPI_Transaction(tx_data, rx_data, WRCmdSize);
  }while(rx_data[4]<0x03); //It make sure that SDO will go high after ADC conversion will done.
  #if Test_Print
      printf("\nCommand\n");
      printf("0x%x\t0x%x\t0x%x\t0x%x\t0x%x\n",tx_data[0], tx_data[1],tx_data[2], tx_data[3], tx_data[4]);
  #endif
}

/*!
  @brief This function polls the STATC register until CTS_V bit is reset back to 0
*/
//To check C-ADC(Contimuous) conversion
//Contimuous mode only
//Not applicable to singleshot mode
void pollVoltCTSconversion(void)
{
  uint8_t Data_Read[8];

  do
  {
    /*Read Status Register*/
    ADBMS_Read_Data(TOTAL_IC, RDSTATC, &Data_Read[0], GRPC);
    #if Test_Print
        printf("\n%x\n", (Data_Read[1] & 3U));
    #endif
  } while (((Data_Read[3] & 3U) != 3U));

  do
  {
    /*Read Status Register*/
    ADBMS_Read_Data(TOTAL_IC, RDSTATC, &Data_Read[0], GRPC);
    #if Test_Print
        printf("%x\n", (Data_Read[1] & 3U));
    #endif
  } while (Data_Read[3] & 3U); // It will come out when CTS_V become 0
}

/*!
  @brief This function polls the STATG register until CTS_V bit is reset back to 0, 8 times
*/
//Only for C ADC and S ADC
//Contimuous mode only
//Not applicable to singleshot mode
void poll8msCtsVconversion(void)
{
  uint8_t vpoll_count = 8;
  for (uint8_t i = 0; i < vpoll_count; i++)
  {
    pollVoltCTSconversion();
  }
}
void ADBMS6830_EnableCellBalancing( uint8_t tIC,double voltage[TOTAL_IC][TOTAL_CELLS])
{
    const double balanceThreshold = 2.0;//4.1
    WakeupBMSIC(); //Wakeup BMS IC and IsoSPI

    // Array to indicate cells needs balancing initially all set to false
    for (int k = 0; k < 16; k++)  // 16 Cells / IC
     {
    	  balanceFlags[k] = false;

     }
    WakeupBMSIC(); //Wakeup BMS IC and IsoSPI

    // Determine which cells need balancing
    for (int ic = 0; ic < tIC; ++ic)
    {
    	for (int cell = 0; cell < 16; ++cell)  // 16 Cells / IC
        {
    		if (voltage[ic][cell] > balanceThreshold)
            {
                balanceFlags[cell] = true; // Updating this cell for Balancing
            }
        }
    	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
    }
    ADBMS6830_ActivateCellBalancing(balanceFlags, 16); // Assuming you want to check all 16 flags
}
 void ADBMS6830_ActivateCellBalancing(bool balanceFlags[], int numCells)
 {
	 uint8_t dutyCycle_dis =0x0F;
	 ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
     WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
     // Cell balancing would involve controlling the discharge MOSFETs for the marked cells.
    for (int numCells = 0; numCells < 16; numCells++)
    {
    	if(balanceFlags[numCells]==true)
        {
            ADBMS6830_SetCellDischargePWM(numCells,dutyCycle_dis);
            SetConfigB_DischargeTimeOutValue(TOTAL_IC, &balanceFlags[numCells],RANG_0_TO_63_MIN,TIME_1MIN_OR_0_26HR);
            adBms6830_Adcv(RD_OFF, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
            ConfigB_DccBit(DCC16, DCC_BIT_SET);
        }
    }
    WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
    // Clear and reset logic as per existing implementation
    ADBMS_Write_Cmd(TOTAL_IC, CLRCELL); // Clear cell Voltages after balancing actions are determined

}
 void ADBMS6830_SetCellDischargePWM(uint8_t cellNumber, uint8_t dutyCycle)
 {
     //WRPWM1--PWM Register Group A address
	 //Need to enable DCC bit here
     uint8_t pwmData[6] = {0}; // Data to be written to PWM registers

     // The PWM Register Group A contains 6 registers (PWMR0 to PWMR5)
     // Each register controls the duty cycle for 2 cells:
     // PWMR0 for cells 1 and 2, PWMR1 for cells 3 and 4, ..., PWMR5 for cells 11 and 12
     // For the 10th cell, it's controlled by PWMR4. Bits [7:4] control cell 10, bits [3:0] control cell 9.
     // Duty cycle configuration: 4'b1111 = 100% duty cycle, 4'b0001 = 6.6% duty cycle, 4'b0000 = disabled

     // Calculate the register index and bit position based on the cell number
     if(cellNumber==0)
     {
    	 cellNumber=1;
     }
     int registerIndex = (cellNumber - 1) / 2;
     bool isEvenCell = cellNumber % 2 == 0;
     // Set the duty cycle for the specified cell
     if (isEvenCell)
     {
         // For even cells, duty cycle bits are [7:4] in the register
         pwmData[registerIndex] = dutyCycle << 4;
         for (int i = 0; i < registerIndex; i++)
         {
           printf("pwmData[%d] = %d\n", i, pwmData[i]);
           ADBMS_Write_Data(TOTAL_IC, WRPWM1, &pwmData[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
         }
     }
     else
     {
         // For odd cells, duty cycle bits are [3:0] in the register
    	 pwmData[registerIndex] = dutyCycle;
         for (int i = 0; i <= registerIndex; i++)
         {
           printf("pwmData[%d] = %d\n", i, pwmData[i]);
           ADBMS_Write_Data(TOTAL_IC, WRPWM1, &pwmData[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
         }
     }
     //Send the SRST command.
     //ADBMS_Write_Data(TOTAL_IC, &SRST[0]);
     //Wakeup the device followed with CLRFLAG command.
     //ADBMS_Write_Data(TOTAL_IC, CLRFLAG);
     //Send the RDSTATC command.
     //ADBMS_Write_Data(TOTAL_IC, RDSTATC);
     //Verify all the STATC register group contents are as 0, no PEC error set and Command counter value is 2.
     //Save the received STATC (8bytes) data, Command Counter value, Command Counter check status and PEC error check status into RAM.
     //Send the RDSTATD command.
     //Verify all the STATD register group contents are as 0, no PEC error set and Command counter value is 2.
     //Save the received STATD (8bytes) data, Command Counter value, Command Counter check status and PEC error check status into RAM.
     //Send the ADSV Commands with CONT=0, DCP=0.
     //Send the PLSADC Commands and wait for the ADC conversion time.
     ADBMS_Write_Cmd(TOTAL_IC, &PLSADC[0]);
     // Add conversion time into RAM.
     // Send the RDACALL/RDSALL command and save all the voltage register group values into the RAM.
     ADBMS_Write_Cmd(TOTAL_IC, &RDACALL[0]);
     ADBMS_Write_Cmd(TOTAL_IC, &RDSALL[0]);
     // Send the CLRCELL/CLRSPIN command to clear the cell voltage register group values.
     // Send the RDSTATC command.
     // Send the RDSTATG command.
     ADBMS_Write_Cmd(TOTAL_IC, &RDSTATG[0]);
     // Enable PWM bit on particular cell, will set on each cell one by one and set DCT0 to 3 mins.
     // Send ADSV command with DCP=0, verify PWM discharge not happening.
     // Send the PLSADC Commands and wait for the ADC conversion time.
     // Add conversion time into RAM.
     // Send the RDACALL/RDSALL command and save all the voltage register group values into the RAM.
     // Send the CLRCELL/CLRSPIN command to clear the cell voltage register group values.
     // Send ADSV command with DCP=1, to verify discharge happening.
     // Send the PLSADC Commands and wait for the ADC conversion time.
     // Add conversion time into RAM.
     // Send the RDACALL/RDSALL command and save all the voltage register group values into the RAM.
     // Validate the S-pin voltage drop and save the result into RAM.
     // Send the CLRCELL/CLRSPIN command to clear the cell voltage register group values.
     // Repeat the below step form 16 to 30 for PWM configuration.
     // Verify all the STATC register group contents are 0, no PEC error and command counter value.
     // Save the received STATC (8bytes) data, command Counter value, command Counter check status and PEC error check status into RAM.
     // Send the RDSTATD command.
     // Verify all the STATD register group contents are 0, no PEC error and command counter value is 37.
     // Save the received STATD (8bytes) data, command Counter value, command Counter check status and PEC error check status into RAM
     // Write the configuration to the PWM Register Group A
 }
 void SetConfigB_DischargeTimeOutValue(uint8_t tIC, cell_asic *ic, DTRNG timer_rang, DCTO timeout_value)
 {
   for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
   {
	 ic[curr_ic].tx_cfgb.dtrng = timer_rang;
     if(timer_rang == RANG_0_TO_63_MIN)
     {
       ic[curr_ic].tx_cfgb.dcto = timeout_value;
     }
     else if(timer_rang == RANG_0_TO_16_8_HR)
     {
       ic[curr_ic].tx_cfgb.dcto = timeout_value;
     }
   }
 }
 void adBms6830_Adcv(RD rd,CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs)
 {
   uint8_t cmd_arg[2];
   cmd_arg[0] = 0x02 + rd;
   cmd_arg[1] = (cont<<7)+(dcp<<4)+(rstf<<2)+(owcs & 0x03) + 0x60;
   ADBMS_Write_Cmd(14,cmd_arg[0]);
 }
 uint16_t ConfigB_DccBit(DCC dcc, DCC_BIT dccbit)
 {
   uint16_t dccvalue;
   if(dccbit == DCC_BIT_SET)
   {
	  dccvalue = (1 << dcc);
      {
    	 printf("DCC bit  = %d\n", dccvalue);
      }
   }
   else
   {
	 dccvalue = (0 << dcc);
     {
        printf("DCC bit = %d\n", dccvalue);
     }
   }
   return(dccvalue);
 }
 /*void ADBMS6830_PWM_Discharge_Managemnet(uint8_t cellNumber, uint8_t dutyCycle)
 {
     uint8_t pwmRegisterAddress = WRPWMA; // PWM Register Group A address
     uint8_t pwmData[6] = {0}; // Data to be written to PWM registers

     // The PWM Register Group A contains 6 registers (PWMR0 to PWMR5)
     // Each register controls the duty cycle for 2 cells:
     // PWMR0 for cells 1 and 2, PWMR1 for cells 3 and 4, ..., PWMR5 for cells 11 and 12
     // For the 10th cell, it's controlled by PWMR4. Bits [7:4] control cell 10, bits [3:0] control cell 9.
     // Duty cycle configuration: 4'b1111 = 100% duty cycle, 4'b0001 = 6.6% duty cycle, 4'b0000 = disabled

     // Calculate the register index and bit position based on the cell number
     int registerIndex = (cellNumber - 1) / 2;
     bool isEvenCell = cellNumber % 2 == 0;

     // Set the duty cycle for the specified cell
     if (isEvenCell) {
         // For even cells, duty cycle bits are [7:4] in the register
         pwmData[registerIndex] = dutyCycle << 4;
     } else {
         // For odd cells, duty cycle bits are [3:0] in the register
         pwmData[registerIndex] = dutyCycle;
     }

     // Write the configuration to the PWM Register Group A
     ADBMS6830_WriteData(pwmRegisterAddress, pwmData, sizeof(pwmData));
 }
 ****************************************************************************************************************
*/
 void ADSV_PWM_discharge_Measurement_test()
 {
     //uint8_t cmd[2];
     uint8_t data[8];
     //bool pecCheckStatus = false;
     //uint8_t commandCounterValue = 0;

     // 1. Send the SRST command
     ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
     // 2. Wakeup the device followed with CLRFLAG command
     Delay_ms(10); // Wakeup delay
     ADBMS_Write_Cmd(TOTAL_IC, CLRCELL); // Clear cell Voltages after balancing actions are determined
     // 3. Send the RDSTATC command
     ADBMS_Write_Cmd(TOTAL_IC, &RDSTATC[0]); // Reset ADBMS (ADBMS will goes into sleep state)
     // 4. Verify all the STATC register group contents are as 0, no PEC error set and Command counter value is 2.
       bool result = true;
      // uint8_t *Data_Read;
      // if ( ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPC))
       //{
       //    printf("Failed to read STATD register.\n");
       //    //return false;
      // }
       // Verify all data bytes are 0
       for (int i = 0; i < sizeof(data); i++)
       {
           if (data[i] != 0x00)
           {
             result = false;
             break;
           }
       }
       // Check for no PEC error and command counter value
       if (result)
       {
         // Assuming the last two bytes are for PEC and command counter, respectively
         // This part may need adjustment based on your actual data structure
         uint16_t pec = CalculatePEC(data, sizeof(data) - 2);
         uint8_t commandCounter = data[sizeof(data) - 1];

           if (pec != ((data[sizeof(data) - 2] << 8) | data[sizeof(data) - 3]))
           {
             printf("PEC error detected.\n");
             result = false;
           }

           if (commandCounter != 2)
           {
             printf("Command counter value is not 2.\n");
             result = false;
           }
       }

     //5.Save the received STATC (8bytes) data, Command Counter value, Command Counter check status and PEC error check status into RAM.

     //6.Send the RDSTATD command.
     ADBMS_Write_Cmd(TOTAL_IC, &RDSTATD[0]); // Reset ADBMS (ADBMS will goes into sleep state)

     //7.Verify all the STATD register group contents are as 0, no PEC error set and Command counter value is 2.

     //bool result = true;
   //  if ( ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPD))
   //  {
   //      printf("Failed to read STATD register.\n");
   //      //return false;
   //  }
     // Verify all data bytes are 0
     for (int i = 0; i < sizeof(data); i++)
     {

         if (data[i] != 0x00) {
             result = false;
             break;
         }
     }
     // Check for no PEC error and command counter value
     if (result)
     {
         // Assuming the last two bytes are for PEC and command counter, respectively
         // This part may need adjustment based on your actual data structure
         uint16_t pec = CalculatePEC(data, sizeof(data) - 2);
         uint8_t commandCounter = data[sizeof(data) - 1];

         if (pec != ((data[sizeof(data) - 2] << 8) | data[sizeof(data) - 3])) {
             printf("PEC error detected.\n");
             result = false;
         }

         if (commandCounter != 2) {
             printf("Command counter value is not 2.\n");
             result = false;
         }
     }

 //8.Save the received STATD (8bytes) data, Command Counter value, Command Counter check status and PEC error check status into RAM.

 //9. Send the ADSV Commands with CONT=0, DCP=0.
 adBms6830_Adcv(RD_OFF, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
 //10.Send the PLSADC Commands and wait for the ADC conversion time.
 ADBMS_Write_Cmd(TOTAL_IC, &PLSADC[0]);
 //11.Add conversion time into RAM.

 //12 Send the RDACALL/RDSALL command and save all the voltage register group values into the RAM.
 ADBMS_Write_Cmd(TOTAL_IC, &RDACALL[0]);
 //13 Send the CLRCELL/CLRSPIN command to clear the cell voltage register group values.
 ADBMS_Write_Cmd(TOTAL_IC, &CLRCELL[0]);
 //14 Send the RDSTATC command.
 ADBMS_Write_Cmd(TOTAL_IC, &RDSTATC[0]);
 //15 Send the RDSTATG command.
 ADBMS_Write_Cmd(TOTAL_IC, &RDSTATG[0]);
 //16 Enable PWM bit on particular cell, will set on each cell one by one and set DCT0 to 3 mins.
 uint8_t cell;
 for (cell = 0; cell < 16; cell++)
 {

 //17 Send ADSV command with DCP=0, verify PWM discharge not happening.
 adBms6830_Adcv(RD_OFF, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
 //18 Send the PLSADC Commands and wait for the ADC conversion time.
 ADBMS_Write_Cmd(TOTAL_IC, &PLSADC[0]);
 //19 Add conversion time into RAM.
 MXC_Delay(1100);
 //20 Send the RDACALL/RDSALL command and save all the voltage register group values into the RAM.
 ADBMS_Write_Cmd(TOTAL_IC, &RDACALL[0]);
 //21 Send the CLRCELL/CLRSPIN command to clear the cell voltage register group values.
 ADBMS_Write_Cmd(TOTAL_IC, &CLRCELL[0]);
 //22 Send ADSV command with DCP=1, to verify discharge happening.
 adBms6830_Adcv(RD_OFF, CONTINUOUS, DCP_ON, RSTF_OFF, OW_OFF_ALL_CH);
 //23 Send the PLSADC Commands and wait for the ADC conversion time.
 ADBMS_Write_Cmd(TOTAL_IC, &PLSADC[0]);
 //24 wait for the ADC conversion time.
 MXC_Delay(1100);
 //25 Send the RDACALL/RDSALL command and save all the S-voltage register group values into the RAM.
 ADBMS_Write_Cmd(TOTAL_IC, &RDACALL[0]);
 //26 Add conversion time into RAM.
 MXC_Delay(1100);
 //27 Validate DCC discharge S-pin voltage drop and add result into RAM.
 //28 Send the CLRCELL/CLRSPIN command to clear the cell voltages register group values.
 ADBMS_Write_Cmd(TOTAL_IC, &CLRCELL[0]);

 //29 Repeat the below step form 16 to 30 for DCC bit in CFGB of all 16 discharge cell.

 //30 Verify all the STATC register group contents are 0, no PEC error and command counter value.

     //bool result = true;
   //  if ( ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPC))
   //  {
   //      printf("Failed to read STATD register.\n");
   //      //return false;
    // }
     // Verify all data bytes are 0
     for (int i = 0; i < sizeof(data); i++)
     {
         if (data[i] != 0x00)
         {
             result = false;
             break;
         }
     // Check for no PEC error and command counter value
     if (result)
     {
         // Assuming the last two bytes are for PEC and command counter, respectively
         // This part may need adjustment based on your actual data structure
         uint16_t pec = CalculatePEC(data, sizeof(data) - 2);
         uint8_t commandCounter = data[sizeof(data) - 1];

         if (pec != ((data[sizeof(data) - 2] << 8) | data[sizeof(data) - 3])) {
             printf("PEC error detected.\n");
             result = false;
         }

         if (commandCounter != 2) {
             printf("Command counter value is not 2.\n");
             result = false;
         }
     }

     //return result;
 //31 Save the received STATC (8bytes) data, command counter value, command counter check status and PEC error check status into RAM

 //32 Send the RDSTATD command.
 ADBMS_Write_Cmd(TOTAL_IC, &RDSTATD[0]);
 //33 Verify all the STATD register group contents are 0, no PEC error and command counter value.

     //bool result = true;
  //   if ( ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPD))
  //   {
  //       printf("Failed to read STATD register.\n");
  //       return false;
  //   }
     // Verify all data bytes are 0
     for (int i = 0; i < sizeof(data); i++)
     {
         if (data[i] != 0x00)
         {
             result = false;
             break;
         }
     }
     // Check for no PEC error and command counter value
     if (result)
     {
         // Assuming the last two bytes are for PEC and command counter, respectively
         // This part may need adjustment based on your actual data structure
         uint16_t pec = CalculatePEC(data, sizeof(data) - 2);
         uint8_t commandCounter = data[sizeof(data) - 1];

         if (pec != ((data[sizeof(data) - 2] << 8) | data[sizeof(data) - 3]))
         {
             printf("PEC error detected.\n");
             result = false;
         }

         if (commandCounter != 2)
         {
             printf("Command counter value is not 2.\n");
             result = false;
         }
      }
     }
 }
 }
  //  return result;
 //34 Save the received STATD (8bytes) data, command counter value, command counter check status and PEC error check status into RAM

 uint16_t CalculatePEC(uint8_t len, uint8_t *data) {
     uint16_t remainder = 16; // PEC seed
     uint16_t addr;

     for (uint8_t i = 0; i < len; i++) {
         addr = ((remainder >> 7) ^ data[i]) & 0xff; // Calculate XOR input
         remainder = (remainder << 8) ^ (addr << 7) ^ (addr << 4) ^ (addr << 3) ^ (addr << 1) ^ addr;
         remainder = remainder & 0xffff;
     }
     return (remainder << 1); // The PEC is the remainder shifted one bit
 }
