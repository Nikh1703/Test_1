/*
 * ADBMSCMDLIST.h
 *
 *  Created on: Oct 7, 2023
 *      Author: HMuley
 */

#ifndef ADBMSCMDLIST_H_
#define ADBMSCMDLIST_H_

#include "ADBMSCOMMON.h"

/* configuration registers commands */
uint8_t WRCFGA[2]        = { 0x00, 0x01 };
uint8_t WRCFGB[2]        = { 0x00, 0x24 };
uint8_t RDCFGA[2]        = { 0x00, 0x02 };
uint8_t RDCFGB[2]        = { 0x00, 0x26 };
uint8_t RDCFGC[2]        = { 0x00, 0x82 };                    //ADBMS6948
uint8_t WRCFGC[2]        = { 0x00, 0x81 };                    //ADBMS6948
uint8_t RDCFGD[2]        = { 0x00, 0xA6 };                    //ADBMS6948
uint8_t WRCFGD[2]        = { 0x00, 0xA4 };                    //ADBMS6948
uint8_t RDCFGE[2]        = { 0x00, 0x73 };                    //ADBMS6948
uint8_t WRCFGE[2]        = { 0x00, 0x74 };                    //ADBMS6948
uint8_t RDCFGF[2]        = { 0x00, 0x75 };                    //ADBMS6948
uint8_t WRCFGF[2]        = { 0x00, 0x76 };                    //ADBMS6948
uint8_t RDCFGG[2]        = { 0x00, 0x77 };                    //ADBMS6948
uint8_t WRCFGG[2]        = { 0x00, 0x78 };                    //ADBMS6948
uint8_t RDCFGH[2]        = { 0x00, 0x79 };                    //ADBMS6948
uint8_t WRCFGH[2]        = { 0x00, 0x7A };                    //ADBMS6948
uint8_t RDCFGI[2]        = { 0x00, 0x7B };                    //ADBMS6948                 
uint8_t WRCFGI[2]        = { 0x00, 0x7C };                    //ADBMS6948


/* Read cell voltage result registers commands */
uint8_t RDCVA[2]         = { 0x00, 0x04 };
uint8_t RDCVB[2]         = { 0x00, 0x06 };
uint8_t RDCVC[2]         = { 0x00, 0x08 };
uint8_t RDCVD[2]         = { 0x00, 0x0A };
uint8_t RDCVE[2]         = { 0x00, 0x09 };
uint8_t RDCVF[2]         = { 0x00, 0x0B };
uint8_t RDCVALL[2]       = { 0x00, 0x0C };

/* Read average cell voltage result registers commands commands */
uint8_t RDACA[2]         = { 0x00, 0x44 };
uint8_t RDACB[2]         = { 0x00, 0x46 };
uint8_t RDACC[2]         = { 0x00, 0x48 };
uint8_t RDACD[2]         = { 0x00, 0x4A };
uint8_t RDACE[2]         = { 0x00, 0x49 };
uint8_t RDACF[2]         = { 0x00, 0x4B };
uint8_t RDACALL[2]       = { 0x00, 0x4C };

/* Read s voltage result registers commands */
uint8_t RDSVA[2]         = { 0x00, 0x03 };
uint8_t RDSVB[2]         = { 0x00, 0x05 };
uint8_t RDSVC[2]         = { 0x00, 0x07 };
uint8_t RDSVD[2]         = { 0x00, 0x0D };
uint8_t RDSVE[2]         = { 0x00, 0x0E };
uint8_t RDSVF[2]         = { 0x00, 0x0F };
uint8_t RDSALL[2]        = { 0x00, 0x10 };

/* Read filtered cell voltage result registers*/
uint8_t RDFCA[2]         = { 0x00, 0x12 };
uint8_t RDFCB[2]         = { 0x00, 0x13 };
uint8_t RDFCC[2]         = { 0x00, 0x14 };
uint8_t RDFCD[2]         = { 0x00, 0x15 };
uint8_t RDFCE[2]         = { 0x00, 0x16 };
uint8_t RDFCF[2]         = { 0x00, 0x17 };
uint8_t RDFCALL[2]       = { 0x00, 0x18 };

/* Read aux results */
uint8_t RDAUXA[2]        = { 0x00, 0x19 };
uint8_t RDAUXB[2]        = { 0x00, 0x1A };
uint8_t RDAUXC[2]        = { 0x00, 0x1B };
uint8_t RDAUXD[2]        = { 0x00, 0x1F };

/* Read redundant aux results */
uint8_t RDRAXA[2]        = { 0x00, 0x1C };
uint8_t RDRAXB[2]        = { 0x00, 0x1D };
uint8_t RDRAXC[2]        = { 0x00, 0x1E };
uint8_t RDRAXD[2]        = { 0x00, 0x25 };

/* Read c and s results */
uint8_t RDCSALL[2]       = { 0x00, 0x11 };
uint8_t RDACSALL[2]      = { 0x00, 0x51 };

/* Read all AUX and Status Registers */
uint8_t RDASALL[2]		   = { 0x00, 0x35 }; 

/* Read all AUX and Redundent AUX Registers */
uint8_t RDAUXALL[2]		   = { 0x00, 0x35 };                    //ADBMS6948

/* Read all Status Registers (A to G first four Byte) */
uint8_t RDSTAALL[2]		   = { 0x00, 0x36 };                    //ADBMS6948

/* All Reg  */
uint8_t RDCSIVALL[2]     = { 0x00, 0x94 };                    //ADBMS6948
uint8_t RDACSIVALL[2]    = { 0x00, 0xD4 };                    //ADBMS6948
uint8_t RDCCFGALL[2]     = { 0x07, 0x4C };                    //ADBMS6948
uint8_t RDCIVALL[2]      = { 0x00, 0x90 };                    //ADBMS6948
uint8_t RDACIVALL[2]     = { 0x00, 0xD0 };                    //ADBMS6948
uint8_t RDSVAIALL[2]     = { 0x00, 0xD2 };                    //ADBMS6948

/* Read status registers */
uint8_t RDSTATA[2]       = { 0x00, 0x30 };
uint8_t RDSTATB[2]       = { 0x00, 0x31 };
uint8_t RDSTATC[2]       = { 0x00, 0x32 };
uint8_t RDSTATCERR[2]    = { 0x00, 0x72 };                    /* ERR */
uint8_t RDSTATD[2]       = { 0x00, 0x33 };
uint8_t RDSTATE[2]       = { 0x00, 0x34 };
uint8_t RDSTATF[2]       = { 0x00, 0xB0 };                    //ADBMS6948
uint8_t RDSTATG[2]       = { 0x00, 0xB1 };                    //ADBMS6948

/* Read Current registers */
uint8_t RDCT[2]          = { 0x00, 0x86 };                    //ADBMS6948                
uint8_t RDI[2]           = { 0x00, 0x84 };                    //ADBMS6948
uint8_t RDIAV[2]         = { 0x00, 0xC4 };                    //ADBMS6948

/* Coulomb Counter Enable Command */
uint8_t CCEN[2]          = { 0x00, 0x56 };                    //ADBMS6948

/* Pwm registers commands */
uint8_t WRPWM1[2]        = { 0x00, 0x20 };
uint8_t RDPWM1[2]        = { 0x00, 0x22 };

uint8_t WRPWM2[2]        = { 0x00, 0x21 };
uint8_t RDPWM2[2]        = { 0x00, 0x23 };

/* Clear commands */
uint8_t CLRCELL[2]       = { 0x07, 0x11 };
uint8_t CLRAUX [2]       = { 0x07, 0x12 };
uint8_t CLRSPIN[2]       = { 0x07, 0x16 };
uint8_t CLRFLAG[2]       = { 0x07, 0x17 };
uint8_t CLRFC[2]         = { 0x07, 0x14 };
uint8_t CLOVUV[2]        = { 0x07, 0x15 };
uint8_t CLRCI[2]         = { 0x07, 0x90 };                    //ADBMS6948
uint8_t CLRSI[2]         = { 0x07, 0x92 };                    //ADBMS6948
uint8_t CLRIAVG[2]       = { 0x07, 0x98 };                    //ADBMS6948
uint8_t CLRCC[2]         = { 0x07, 0x94 };                    //ADBMS6948

/* Poll adc command */
uint8_t PLADC[2]         = { 0x07, 0x18 };
uint8_t PLCADC[2]        = { 0x07, 0x1C };
uint8_t PLSADC[2]        = { 0x07, 0x1D };
uint8_t PLAUX1[2]        = { 0x07, 0x1E };
uint8_t PLAUX2[2]        = { 0x07, 0x1F };
uint8_t PLI1ADC[2]       = { 0x07, 0x1A };                    //ADBMS6948
uint8_t PLI2ADC[2]       = { 0x07, 0x1B };                    //ADBMS6948

/* Default ADC command */
uint8_t ADCV[2]        	= { 0x02, 0x60 };
uint8_t ADSV[2]        	= { 0x01, 0x68 };
uint8_t ADI1[2]        	= { 0x02, 0x00 };                    //ADBMS6948
uint8_t ADI2[2]        	= { 0x01, 0x08 };                    //ADBMS6948
uint8_t ADCIV[2]        = { 0x02, 0x40 };                    //ADBMS6948
uint8_t ADAX[2]        	= { 0x04, 0x10 };
uint8_t ADAX2[2]        = { 0x04, 0x00 };
//uint8_t ADCC[2]        	= { 0x04, 0x1F };

/* Diagn command */
uint8_t DIAGN[2]         = {0x07 , 0x15};                    //ADBMS6948

/* GPIOs Comm commands */
uint8_t WRCOMM[2]        = { 0x07, 0x21 };
uint8_t RDCOMM[2]        = { 0x07, 0x22 };
uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/* Mute and Unmute commands */
uint8_t MUTE[2] 	     = { 0x00, 0x28 };
uint8_t UNMUTE[2]        = { 0x00, 0x29 };

/*Control Command */
uint8_t RSTCC[2]         = { 0x00, 0x2E };
uint8_t SNAP[2]          = { 0x00, 0x2D };
uint8_t UNSNAP[2]        = { 0x00, 0x2F };
uint8_t SRST[2]          = { 0x00, 0x27 };

/* Read SID command */
uint8_t RDSID[2]         = { 0x00, 0x2C };

/* always On Commands -ADBMS6948*/
uint8_t ULAO[2]         = { 0x00, 0x38 };/* Retention Reg Commands -ADBMS6830*/ 
uint8_t WRAO[2]         = { 0x00, 0x39 };/* Retention Reg Commands -ADBMS6830*/ 
uint8_t RDAO[2]         = { 0x00, 0x3A };/* Retention Reg Commands -ADBMS6830*/ 
uint8_t RDAOOC3A[2]     = { 0x00, 0x3C };                    //ADBMS6948
uint8_t WRAOOC3A[2]     = { 0x00, 0x3E };                    //ADBMS6948

/* LPCM Commands*/
uint8_t CMDIS[2]		= { 0x00, 0x40 };
uint8_t CMEN[2]			= { 0x00, 0x41 };
uint8_t CMHB[2]			= { 0x00, 0x43 };
uint8_t CMWRCFG[2]		= { 0x00, 0x58 };
uint8_t CMRDCFG[2]		= { 0x00, 0x59 };
uint8_t CMWRCELLT[2]	= { 0x00, 0x5A };
uint8_t CMRDCELLT[2]	= { 0x00, 0x5B };
uint8_t CMWRGPIOT[2]	= { 0x00, 0x5C };
uint8_t CMRDGPIOT[2]	= { 0x00, 0x5D };
uint8_t CMWRCURRT[2]	= { 0x00, 0x7E };                    //ADBMS6948
uint8_t CMRDCURRT[2]	= { 0x00, 0x7F };                    //ADBMS6948
uint8_t CMCLRFLAG[2]	= { 0x00, 0x5E };
uint8_t CMRDFLAG[2]		= { 0x00, 0x5F };

/*=============D E F I N E S =============*/

#define	BITP_ADCV_CV_CONT 7U
#define	BITP_ADCV_CV_RD 8U
#define	BITP_ADCV_CV_RSTF 2U
#define	BITP_ADCV_CV_SSDP 4U
#define	BITM_ADCV_C_OW 0x3U

#define	BITP_ADSV_SV_CONT 7U
#define	BITP_ADSV_SV_SSDP 4U
#define	BITM_ADSV_S_OW 0x3U

#define	BITP_ADI1_I1_CONT 7U
#define	BITP_ADI1_I1_DIAG_OW 4U
#define	BITP_ADI1_I1_RD 8U
#define	BITM_ADI1_I1_DIAGSEL 0x7U

#define	BITP_ADI2_I2_CONT 7U
#define	BITP_ADI2_I2_DIAG_OW 4U
#define	BITM_ADI2_I2_DIAGSEL 0x7U

#define	BITP_ADCIV_CI_CONT 7U
#define	BITP_ADCIV_CI_RD 8U
#define	BITP_ADCIV_CI_RSTF 2U
#define	BITP_ADCIV_CI_SSDP 4U
#define	BITM_ADCIV_CI_OW 0x3U

#define	BITP_ADAX_AUX_OW 8U
#define	BITP_ADAX_PUP 7U
#define	BITM_ADAX_CH_ADAX 0x4fU

#define	BITM_ADAX2_CH_ADAX2 0xfU

#define	BITP_CMEN_I_MON_EN 2U
#define	BITP_RDSTATC_ERR 6U
#define	BITP_CCEN_CCEN_CONT 7U
#define	BITP_CCEN_CCEN_EN 8U

/*=============C O D E =============*/

/*!
@brief The function creates ADCV Command
@param	[in]	CV_CONT value for command bit field CV_CONT
@param	[in]	CV_RD value for command bit field CV_RD
@param	[in]	CV_RSTF value for command bit field CV_RSTF
@param	[in]	CV_SSDP value for command bit field CV_SSDP
@param	[in]	C_OW value for command bit field C_OW
@param	[out]	 cmd Command buffer
*/
void ADBMS_ADCV(uint8_t CV_CONT, uint8_t CV_RD, uint8_t CV_RSTF, uint8_t CV_SSDP, uint8_t C_OW, uint8_t *cmd)
{
  cmd[0] = CV_RD + 0x2U;
  cmd[1] = (CV_CONT << BITP_ADCV_CV_CONT) + (CV_RSTF << BITP_ADCV_CV_RSTF) + (CV_SSDP << BITP_ADCV_CV_SSDP) + (C_OW & BITM_ADCV_C_OW) + 0x60U;
}

/*!
@brief The function creates ADSV Command
@param	[in]	SV_CONT value for command bit field SV_CONT
@param	[in]	SV_SSDP value for command bit field SV_SSDP
@param	[in]	S_OW value for command bit field S_OW
@param	[out]	 cmd Command buffer
*/
void ADBMS_ADSV(uint8_t SV_CONT, uint8_t SV_SSDP, uint8_t S_OW, uint8_t *cmd)
{
  cmd[0] = 0x1U;
  cmd[1] = (SV_CONT << BITP_ADSV_SV_CONT) + (SV_SSDP << BITP_ADSV_SV_SSDP) + (S_OW & BITM_ADSV_S_OW) + 0x68U;
}

/*!
@brief The function creates ADAX Command
@param	[in]	AUX_OW value for command bit field AUX_OW
@param	[in]	CH_ADAX value for command bit field CH_ADAX
@param	[in]	PUP value for command bit field PUP
@param	[out]	 cmd Command buffer
*/
void ADBMS_ADAX(uint8_t AUX_OW, CH_ADAX_t CH_ADAX, uint8_t PUP, uint8_t *cmd)
{
  cmd[0] = AUX_OW + 0x4U;
  cmd[1] = (PUP << BITP_ADAX_PUP) + (CH_ADAX & 0x0F) + ((CH_ADAX & 0x10) << 2) + 0x10U;
}

/*!
@brief The function creates ADAX2 Command
@param	[in]	CH_ADAX2 value for command bit field CH_ADAX2
@param	[out]	 cmd Command buffer
*/
void ADBMS_ADAX2(CH_ADAX2_t CH_ADAX2, uint8_t *cmd)
{
  cmd[0] = 0x4U;
  cmd[1] = (CH_ADAX2 & BITM_ADAX2_CH_ADAX2) + 0x0U;
}

/*!
@brief The function creates RDSTATC Command
@param	[in]	ERR value for command bit field ERR
@param	[out]	 cmd Command buffer
*/
void ADBMS_RDSTATC(uint8_t ERR, uint8_t *cmd)
{
  cmd[0] = 0x0U;
  cmd[1] = (ERR << BITP_RDSTATC_ERR) + 0x32U;
}

#endif /* ADBMSCMDLIST_H_ */
