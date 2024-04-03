/*
 * ADBMSCOMMON.h
 *
 *  Created on: Oct 7, 2023
 *      Author: HMuley
 */

#ifndef ADBMSCOMMON_H_
#define ADBMSCOMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include "spi_reva1.h"

/*=============D E F I N E S =============*/

/***** Initialization of MCU *****/
#define 	SPI_SPEED         	500000		//Clock speed for ADBMS6830 //LTC6820 - Mx CLK Speed 1MHz // ADBMS6821/22 - Mx CLK Speed 2MHz
#define		Master_Mode			    1			    //MCU in Master Mode
#define		Slave_Mode			    0			    //MCU in Slave Mode (Target in Master Mode)
#define		Quad_Mode			      1			    //4-bits per SCK cycle (Quad mode SPI).
#define		Mono_Mode			      0			    //1-bit per SCK cycle (Three-wire half-duplex SPI and Four-wire full-duplex SPI)
#define 	Slave_1				      1			    //1 Slave  connected to the MCU
#define 	Slave_2				      2			    //2 Slaves connected to the MCU				
#define 	Slave_3				      3			    //3 Slaves connected to the MCU
#define 	Slave_4				      4			    //4 Slaves connected to the MCU
#define		SS_Polarity			    0         //Slave Select at Active low polarity

/***** BMS IC *****/
/*Number of cells*/
#define N_CELLS 16U
/*Number of cells*/
#define N_CELLS_PER_REGISTERS 3U
/*Number of GPIOs*/
#define N_GPIOS 10U
/*Number of bytes for a cell*/
#define BYTES_IN_CELL 2U
/*Number of bytes in data PEC*/
#define PEC_SIZE 2U
/*Number of registers in a group*/
#define REGISTER_CNT_IN_GRP 6U
/*Number of bytes receive for Cell, S, Aux, Status Read Commands*/
#define RECEIVE_RD_PACKET_SIZE ((N_CELLS_PER_REGISTERS * BYTES_IN_CELL) + PEC_SIZE)*TOTAL_IC
/*Number of bytes receive for All Read Commands*/
#define RECEIVE_RD_ALL_PACKET_SIZE ((N_CELLS * BYTES_IN_CELL) + PEC_SIZE)   //  32 + 2
/*Number of bytes on the command packet 2byte Cmd and 2 byte PEC*/
#define COMMAND_PACKET_SIZE 4U
/*Number of byte receive for RDCVALL command*/
#define RDCVALL_SIZE  RECEIVE_RD_ALL_PACKET_SIZE                //  32 + 2
/*Number of byte receive for RDACVALL command*/
#define RDACVALL_SIZE  RECEIVE_RD_ALL_PACKET_SIZE               //  32 + 2
/*Number of byte receive for RDSVALL command*/
#define RDSVALL_SIZE  RECEIVE_RD_ALL_PACKET_SIZE                //  32 + 2
/*Number of byte receive for RDFCVALL command*/
#define RDFCVALL_SIZE  RECEIVE_RD_ALL_PACKET_SIZE               //  32 + 2
/*Number of Bytes for Read All GPIOs*/
#define RDGPALL_SIZE ((N_GPIOS * BYTES_IN_CELL) + PEC_SIZE)       //  22 + 2                          
#define RDAUXALL_SIZE ((N_GPIOS * BYTES_IN_CELL) + PEC_SIZE)      //  22 + 2
/*Number of Bytes for Read All Redundant GPIOs*/
#define RDRGPALL_SIZE ((N_GPIOS * BYTES_IN_CELL) + PEC_SIZE)      //  22 + 2
/*Number of Bytes for RDCSALL command*/
#define RDCSALL_SIZE (((N_CELLS * BYTES_IN_CELL) * 2) + PEC_SIZE)     //  32*2 + 2
/*Number of Bytes for RDACSALL command*/
#define RDACSALL_SIZE (((N_CELLS * BYTES_IN_CELL) * 2) + PEC_SIZE)     //  32*2 + 2
/*Number of Bytes for RDACSALL command*/
#define RDSTAALL_SIZE 0
/*Number of Bytes for RDACSALL command*/
#define RDCCFGALL_SIZE 0

///**** Definations *****///
#define Test_Print      1
#define ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x)    ((double)(x) * (double)(0.000150) + (double)(1.5))  //Cell Voltage
#define ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x)           ((double)(x) * (double)(0.000150) + (double)(1.5))  //GPIO(AUX) Voltage
//STATA
#define ADBMS_CONVERT_VREF2_HEX_TO_VOLT(x)          ((double)(x) * (double)(0.000150) + (double)(1.5))  //VREF2 (2.988V to 3.012V)
#define ADBMS_CONVERT_ITEMP_HEX_TO_VOLT(x)          ((double)(x) * (double)(0.000150) + (double)(1.5))/(double)(0.0075) - (double)(273)  //Dia Temp of IC
//STATB
#define ADBMS_CONVERT_VD_HEX_TO_VOLT(x)             ((double)(x) * (double)(0.000150) + (double)(1.5))  //Digital power supply (2.7 to 3.6V)
#define ADBMS_CONVERT_VA_HEX_TO_VOLT(x)             ((double)(x) * (double)(0.000150) + (double)(1.5))  //Analog Power Supply (4.5V to 5.5V)
#define ADBMS_CONVERT_VRES_HEX_TO_VOLT(x)           ((double)(x) * (double)(0.000150) + (double)(1.5))  //Voltage Accross 4K Resistor (2.9V to 3.1V)
//AUX4//GPIO4
#define ADBMS_CONVERT_VM_HEX_TO_VOLT(x)             ((double)(x) * (double)(0.000150) + (double)(1.5))  //Voltage Accross -V and Exposed PAD (~0V)
#define ADBMS_CONVERT_VP_HEX_TO_VOLT(x)             ((double)(x) * (double)(0.000150) + (double)(1.5))*(double)(25)  //Total Cell Voltage(12V to 72V) (+v to -V)

///***** Set ADCV CMD *****/
//ADBMS_ADCV(CONT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, &cmd[0]);
#define CONT_ADCV					0x1
#define SINGLESHOT_ADCV				0x0
#define RD_OFF_ADCV					0x0
#define RD_ON_ADCV					0x1
#define NO_RESET_ADCV				0x0
#define RESET_ADCV					0x0
#define DISCHARGE_ADCV				0x1
#define NODISCHARGE_ADCV			0x0
#define DISCHARGE_ADCV				0x1
#define ALL_CH_OW_OFF_ADCV		    0x0
#define EVEN_CH_OW_ADCV				0x1
#define ODD_CH_OW_ADCV				0x2
#define ALL_CH_OW_OFF2_ADCV		    0x3

///***** Set ADSV CMD *****/
//ADBMS_ADSV(CONT_ADSV, NODISCHARGE_ADSV, ALL_CH_OW_OFF_ADSV, &cmd[0]);
#define CONT_ADSV					    0x1
#define Singleshot_ADSV				0x0
#define DISCHARGE_ADSV				0x1
#define NODISCHARGE_ADSV			0x0
#define ALL_CH_OW_OFF_ADSV		0x0
#define EVEN_CH_OW_ADSV				0x1
#define ODD_CH_OW_ADSV				0x2
#define ALL_CH_OW_OFF2_ADSV		0x3
#define CELL 16                 /* Bms ic number of cell              */
#define AUX  12                 /* Bms ic number of Aux               */
#define RAUX 10                 /* Bms ic number of RAux              */
#define PWMA 12                 /* Bms ic number of PWMA              */
#define PWMB 4                  /* Bms ic number of PWMB              */
#define COMM 3                  /* GPIO communication comm reg        */
#define RSID 6                  /* Bms ic number of SID byte          */
#define TX_DATA 6               /* Bms tx data byte                   */
#define RX_DATA 8               /* Bms rx data byte                   */
typedef enum  { RANG_0_TO_63_MIN = 0x0, RANG_0_TO_16_8_HR = 0x1 } DTRNG;
/*!
*  \enum DCTO
* DCTO: DCTO timeout values.
*/
typedef enum
{
  DCTO_TIMEOUT = 0,
  TIME_1MIN_OR_0_26HR,
  TIME_2MIN_OR_0_53HR,
  TIME_3MIN_OR_0_8HR,
  TIME_4MIN_OR_1_06HR,
  TIME_5MIN_OR_1_33HR,
  TIME_6MIN_OR_1_6HR,
  /* If required more time out value add here */
} DCTO;
//typedef enum  { RANG_0_TO_63_MIN = 0x0, RANG_0_TO_16_8_HR = 0x1 } DTRNG;
///***** Set ADAX amd ADAX2 CMD *****/
// void ADBMS_ADAX(uint8_t AUX_OW, uint8_t CH_ADAX, uint8_t PUP, uint8_t *cmd);
// void ADBMS_ADAX2(uint8_t CH_ADAX2, uint8_t *cmd);
#define OW_AUX_OFF_ADAX       0x0
#define OW_AUX_ON_ADAX        0x1
#define PULLDOWN_ADAX         0x0
#define PULLUP_ADAX           0x1
/*! \enum   CH_ADAX_t
    \brief  Channel Select
*/
typedef enum
{
  ALL_ADAX = 0x0,	              /*!<All Channel Select*/
  GPIO1TO7_ADAX = 0xE,	        /*!<GPIO 1-7*/
  GPIO8TO10_VP_VM_ADAX = 0xF,	  /*!<GPIO 8-10, V+, V-*/
  GPIO1_ADAX = 0x1,	            /*!<Select GPIO 1*/
  GPIO2_ADAX = 0x2,	            /*!<Select GPIO2*/
  GPIO3_ADAX = 0x3,	            /*!<Select GPIO3*/
  GPIO4_ADAX = 0x4,	            /*!<Select GPIO4*/
  GPIO5_ADAX = 0x5,	            /*!<Select GPIO5*/
  GPIO6_ADAX = 0x6,	            /*!<Select GPIO6*/
  GPIO7_ADAX = 0x7,	            /*!<Selet GPIO7*/
  GPIO8_ADAX = 0x8,	            /*!<Select GPIO8*/
  GPIO9_ADAX = 0x9,	            /*!<Select GPIO9*/
  GPIO10_ADAX = 0xA,	          /*!<Select GPIO10*/
  VREF2_ADAX = 0x10,	          /*!<Select Vref2*/
  LDO3V_ADAX = 0x11,	          /*!<Select LDO3V*/
  LDO5V_ADAX = 0x12,	          /*!<Select LDO5V*/
  TEMP_ADAX = 0x13,	            /*!<Select TEMP*/
  VP2M_ADAX = 0x14,	            /*!<Select VP2M*/
  VM_ADAX = 0x15,	              /*!<Select VM*/
  VR4K_ADAX = 0x16,	            /*!<Select VR4K*/
  VREF1_DIV_ADAX = 0x17	        /*!<Select VREF1_DIV*/
}CH_ADAX_t;

/*! \enum   CH_ADAX2_t
    \brief  Channel Select
*/

typedef enum
{
  ALL_ADAX2 = 0x0,	        /*!<All Channel Select*/
  GPIO8TO10_ADAX2 = 0xF,	  /*!<GPIO 8-10*/
  GPIO1TO7_ADAX2 = 0xE,	    /*!<GPIO 1-7*/
  GPIO1_ADAX2 = 0x1,	      /*!<Select GPIO 1*/
  GPIO2_ADAX2 = 0x2,	      /*!<Select GPIO2*/
  GPIO3_ADAX2 = 0x3,	      /*!<Select GPIO3*/
  GPIO4_ADAX2 = 0x4,	      /*!<Select GPIO4*/
  GPIO5_ADAX2 = 0x5,	      /*!<Select GPIO5*/
  GPIO6_ADAX2 = 0x6,	      /*!<Select GPIO6*/
  GPIO7_ADAX2 = 0x7,	      /*!<Selet GPIO7*/
  GPIO8_ADAX2 = 0x8,	      /*!<Select GPIO8*/
  GPIO9_ADAX2 = 0x9,	      /*!<Select GPIO9*/
  GPIO10_ADAX2 = 0xA	      /*!<Select GPIO10*/
}CH_ADAX2_t;
typedef enum  { RSTF_OFF = 0x0, RSTF_ON = 0x1 } RSTF;
typedef enum { OW_OFF_ALL_CH = 0X0, OW_ON_EVEN_CH, OW_ON_ODD_CH, OW_ON_ALL_CH} OW_C_S;

typedef enum
{
  DCC1 = 0,
  DCC2,
  DCC3,
  DCC4,
  DCC5,
  DCC6,
  DCC7,
  DCC8,
  DCC9,
  DCC10,
  DCC11,
  DCC12,
  DCC13,
  DCC14,
  DCC15,
  DCC16,
} DCC;
typedef struct
{
  uint8_t       refon   :1;
  uint8_t       cth     :3;
  uint8_t       flag_d  :8;
  uint8_t       soakon  :1;
  uint8_t       owrng   :1;
  uint8_t       owa     :3;
  uint16_t      gpo     :10;
  uint8_t       snap    :1;
  uint8_t       mute_st :1;
  uint8_t       comm_bk :1;
  uint8_t       fc      :3;
}cfa_;
/* For ADBMS6830 config register structure */
typedef struct
{
  uint16_t  vuv     :16;
  uint16_t  vov     :16;
  uint8_t   dtmen   :1;
  uint8_t   dtrng   :1;
  uint8_t   dcto    :6;
  uint16_t  dcc     :16;
}cfb_;
typedef struct
{
  uint16_t      cl_csflt;
  uint8_t       cl_smed   :1;
  uint8_t       cl_sed    :1;
  uint8_t       cl_cmed   :1;
  uint8_t       cl_ced    :1;
  uint8_t       cl_vduv   :1;
  uint8_t       cl_vdov   :1;
  uint8_t       cl_vauv   :1;
  uint8_t       cl_vaov   :1;
  uint8_t       cl_oscchk :1;
  uint8_t       cl_tmode  :1;
  uint8_t       cl_thsd   :1;
  uint8_t       cl_sleep  :1;
  uint8_t       cl_spiflt :1;
  uint8_t       cl_vdel   :1;
  uint8_t       cl_vde    :1;
} clrflag_;
/* Cell Voltage Data structure */
typedef struct
{
  int16_t c_codes[CELL]; /* Cell Voltage Codes */
} cv_;

typedef struct
{
  int16_t ac_codes[CELL]; /* Average Cell Voltage Codes */
} acv_;

/* S Voltage Data structure */
typedef struct
{
  int16_t sc_codes[CELL]; /* S Voltage Codes */
} scv_;

/* Filtered Cell Voltage Data structure */
typedef struct
{
  int16_t fc_codes[CELL]; /* filtered Cell Voltage Codes */
} fcv_;

/* Aux Voltage Data Structure*/
typedef struct
{
  int16_t a_codes[AUX]; /* Aux Voltage Codes */
} ax_;
typedef struct
{
  int16_t ra_codes[RAUX]; /* Aux Voltage Codes */
} rax_;

/* Status A register Data structure*/
typedef struct
{
  uint16_t  vref2;
  uint16_t  itmp;
  uint16_t  vref3;
} sta_;

/* Status B register Data structure*/
typedef struct
{
  uint16_t vd;
  uint16_t va;
  uint16_t vr4k;
} stb_;

/* Status C register Data structure*/
typedef struct
{
  uint16_t      cs_flt;
  uint8_t       va_ov   :1;
  uint8_t       va_uv   :1;
  uint8_t       vd_ov   :1;
  uint8_t       vd_uv   :1;
  uint8_t       otp1_ed :1;
  uint8_t       otp1_med:1;
  uint8_t       otp2_ed :1;
  uint8_t       otp2_med:1;
  uint8_t       vde     :1;
  uint8_t       vdel    :1;
  uint8_t       comp    :1;
  uint8_t       spiflt  :1;
  uint8_t       sleep   :1;
  uint8_t       thsd    :1;
  uint8_t       tmodchk :1;
  uint8_t       oscchk  :1;
} stc_;
typedef struct
{
  uint8_t c_ov[CELL];
  uint8_t c_uv[CELL];
  uint8_t ct            :6;
  uint8_t cts           :2;
  uint8_t oc_cntr;
} std_;
/* Status E register Data structure*/
typedef struct
{
  uint16_t gpi          :10;
  uint8_t rev           :4;
} ste_;

/* Pwm register Data structure*/
typedef struct
{
  uint8_t pwma[PWMA];
} pwma_;

/*PWMB Register Structure */
typedef struct
{
  uint8_t pwmb[PWMB];
} pwmb_;

/* COMM register Data structure*/
typedef struct
{
  uint8_t fcomm[COMM];
  uint8_t icomm[COMM];
  uint8_t data[COMM];
} com_;
/*SID Register Structure */
typedef struct
{
  uint8_t sid[RSID];
} sid_;

/* Transmit byte and recived byte data structure */
typedef struct
{
  uint8_t tx_data[TX_DATA];
  uint8_t rx_data[RX_DATA];
} ic_register_;
/* Aux open wire data structure */
typedef struct
{
  int cell_ow_even[CELL];
  int cell_ow_odd[CELL];
} cell_ow_;

/* Aux open wire data structure */
typedef struct
{
  int aux_pup_up[(AUX-2)];
  int aux_pup_down[(AUX-2)];
} aux_ow_;
/* Command counter and pec error data Structure */
typedef struct
{
  uint8_t cmd_cntr;
  uint8_t cfgr_pec;
  uint8_t cell_pec;
  uint8_t acell_pec;
  uint8_t scell_pec;
  uint8_t fcell_pec;
  uint8_t aux_pec;
  uint8_t raux_pec;
  uint8_t stat_pec;
  uint8_t comm_pec;
  uint8_t pwm_pec;
  uint8_t sid_pec;
} cmdcnt_pec_;
typedef struct
{
  uint8_t osc_mismatch;
  uint8_t supply_error;
  uint8_t supply_ovuv;
  uint8_t thsd;
  uint8_t fuse_ed;
  uint8_t fuse_med;
  uint8_t tmodchk;
  uint8_t cell_ow[CELL];
  uint8_t cellred_ow[CELL];
  uint8_t aux_ow[(AUX-2)];
} diag_test_;
typedef struct
{
  cfa_ tx_cfga;
  cfa_ rx_cfga;
  cfb_ tx_cfgb;
  cfb_ rx_cfgb;
  clrflag_ clflag;
  cv_  cell;
  acv_ acell;
  scv_ scell;
  fcv_ fcell;
  ax_  aux;
  rax_ raux;
  sta_ stata;
  stb_ statb;
  stc_ statc;
  std_ statd;
  ste_ state;
  com_ comm;
  pwma_ PwmA;
  pwmb_ PwmB;
  sid_ sid;
  ic_register_ configa;
  ic_register_ configb;
  ic_register_ clrflag;
  ic_register_ stat;
  ic_register_ com;
  ic_register_ pwma;
  ic_register_ pwmb;
  ic_register_ rsid;
  cmdcnt_pec_ cccrc;
  aux_ow_ gpio;
  cell_ow_ owcell;
  diag_test_ diag_result;
} cell_asic;
typedef enum  { DCC_BIT_CLR = 0x0, DCC_BIT_SET = 0x1 } DCC_BIT;
typedef enum { RD_OFF = 0X0, RD_ON = 0X1} RD;
typedef enum { DCP_OFF = 0X0, DCP_ON = 0X1} DCP;
typedef enum { SINGLE = 0X0, CONTINUOUS = 0X1} CONT;

typedef enum
{
RD_CV_ALL,        /*!< Read All Cell Voltage Result Registers*/
RD_AC_ALL,        /*!< Read All Averaged Cell Voltage Result Registers*/
RD_S_ALL,         /*!< Read All S-Voltage Result Registers*/
RD_FC_ALL,        /*!< Read All Filtered Cell Voltage Result Registers*/
RD_AUX_ALL,
RD_C_S_ALL,       /*!< Read all C & S Results*/
RD_AC_S_ALL,      /*!< Read all Avg (C & S) Results*/
RD_STA_ALL,		    /*!< Read All Status Registers Group*/
RD_C_CFG_ALL,		  /*!< Read All Configuration Registers Group*/
NOT_ALL,
GRP_NONE,
GRPA,
GRPB,
GRPC,
GRPD,
GRPE,
GRPF
}RD_DATA_SIZE_ALL_TYPE;

/*=============P U B L I C P R O T O T Y P E S =============*/

/*============= Function Declaration  =============*/
//main.c
void SPI_Transaction(uint8_t *tx_data, uint8_t *rx_data, uint8_t data_size);
int WakeupIC(mxc_spi_regs_t *spi);
void WakeupBMSIC(void);

//ADBMSMAIN.C
void adbmsmain(void);
void printMenu(void);
void run_command(int command);
void ADBMS_Init(uint8_t TOTAL_IC);
void ADBMS_Write_Read_Config(uint8_t TOTAL_IC);
void ADC_Cell_Voltage_Measurement(uint8_t TOTAL_IC);
void ADC_S_Voltage_Measurement(uint8_t TOTAL_IC);
void ADC_AVG_Cell_Voltage_Measurement(uint8_t TOTAL_IC);
void ADC_GPIO_Voltage_Measurement(uint8_t TOTAL_IC);
void ADBMS_Status_Reg_voltage_measurment(uint8_t TOTAL_IC);
void ADBMS_Print_WRCFG_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_RDCFG_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_AUX_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_Status_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMSPollADC(uint8_t tIC, uint8_t cmd_arg[2]);
void pollVoltCTSconversion(void);
void poll8msCtsVconversion(void);
void Delay_ms(uint32_t delay)
{

}

//ADBMSCOMMON.c
//uint16_t CalculatePEC(uint8_t *data, int len);

uint16_t CalculatePEC(uint8_t len, uint8_t *data);


void SPI_Write(uint8_t tIC, uint8_t cmd_arg[0]);
void ADBMS_Write_Cmd(uint8_t tIC, uint8_t cmd_arg[2]);
void ADBMS_Write_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *data_Write);
void ADBMS_Read_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *data_Read, RD_DATA_SIZE_ALL_TYPE type);
uint16_t Pec15_Calc( uint8_t len, uint8_t *data);
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data);
int16_t twos_complement_to_int(uint16_t value, uint8_t num_bits);
uint16_t SetUnderVoltageThreshold(double voltage);
uint16_t SetOverVoltageThreshold(double voltage);
void WRCFGB_data_Set(uint8_t tIC, uint8_t cmd_arg[], float UV_THSD, float OV_THSD, uint8_t *data_Write);

void ADSV_PWM_discharge_Measurement_test(void);

//ADBMSCMDLIST.h
void ADBMS_ADCV(uint8_t CV_CONT, uint8_t CV_RD, uint8_t CV_RSTF, uint8_t CV_SSDP, uint8_t C_OW, uint8_t *cmd);
void ADBMS_ADSV(uint8_t SV_CONT, uint8_t SV_SSDP, uint8_t S_OW, uint8_t *cmd);
void ADBMS_ADAX(uint8_t AUX_OW, CH_ADAX_t CH_ADAX, uint8_t PUP, uint8_t *cmd);
void ADBMS_ADAX2(CH_ADAX2_t CH_ADAX2, uint8_t *cmd);
void ADBMS_RDSTATC(uint8_t ERR, uint8_t *cmd);
//void ADBMS6830_ActivateCellBalancing(int target_cell);
void ADBMS6830_ActivateCellBalancing(bool balanceFlags[], int numCells);
void ADBMS6830_EnableCellBalancing(uint8_t tIC,double voltage[4][12]);
void SetConfigB_DischargeTimeOutValue(uint8_t tIC, cell_asic *ic, DTRNG timer_rang, DCTO timeout_value);
void adBms6830_Adcv(RD rd,CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs);
uint16_t ConfigB_DccBit(DCC dcc, DCC_BIT dccbit);
void ADBMS6830_SetCellDischargePWM(uint8_t cellNumber, uint8_t dutyCycle);
//void ADBMS6830_SetCellDischarge();
#endif /* ADBMSCOMMON_H_ */
