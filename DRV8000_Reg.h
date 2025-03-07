/*
 * DRV8000_Reg.h
 *
 *  Created on: Mar 6, 2025
 *      Author: thinh
 */

#ifndef GDU_TI_DRV8000_DRV8000_REG_H_
#define GDU_TI_DRV8000_DRV8000_REG_H_

/* **********************************************************************/
/* ***              System and library files included                 ***/
/* **********************************************************************/
#include <stdint.h>

/* **********************************************************************/
/* ***           Definition of global plain CONSTants                 ***/
/* **********************************************************************/

/* ********************************************************************/
/* ***            Register Addresses                                ***/
/* ********************************************************************/

/* **************************************************************** */
/* ************* STATUS REGISTERS ********************************* */
/* **************************************************************** */
#define DRV8000_ADDREG_IC_STAT1 0x00u
#define DRV8000_ADDREG_IC_STAT2 0x01u
#define DRV8000_ADDREG_GD_STAT 0x02u
#define DRV8000_ADDREG_HB_STAT1 0x03u
#define DRV8000_ADDREG_HB_STAT2 0x04u
#define DRV8000_ADDREG_EC_HEAT_IT_RIP_STAT 0x05u
#define DRV8000_ADDREG_HS_STAT 0x06u
#define DRV8000_ADDREG_SPARE_STAT1 0x07u
#define DRV8000_ADDREG_DEVICE_ID 0x08u

/* **************************************************************** */
/* ************* CONFIG REGISTERS ********************************* */
/* **************************************************************** */
#define DRV8000_ADDREG_IC_CNFG1 0x0Au
#define DRV8000_ADDREG_IC_CNFG2 0x0Au
#define DRV8000_ADDREG_GD_CNFG 0x0Bu
#define DRV8000_ADDREG_GD_IDRV_CNFG 0x0Cu
#define DRV8000_ADDREG_GD_VGS_CNFG 0x0Du
#define DRV8000_ADDREG_GD_VDS_CNFG 0x0Eu
#define DRV8000_ADDREG_GD_CSA_CNFG 0x0Fu
#define DRV8000_ADDREG_GD_AGD_CNFG 0x10u
#define DRV8000_ADDREG_GD_PDR_CNFG 0x11u
#define DRV8000_ADDREG_GD_STC_CNFG 0x12u
#define DRV8000_ADDREG_GD_SPARE_CNFG1 0x13u
#define DRV8000_ADDREG_HB_ITRIP_DG 0x14u
#define DRV8000_ADDREG_HB_OUT_CNFG1 0x15u
#define DRV8000_ADDREG_HB_OUT_CNFG2 0x16u
#define DRV8000_ADDREG_HB_OCP_CNFG 0x17u
#define DRV8000_ADDREG_HB_OL_CNFG1 0x18u
#define DRV8000_ADDREG_HB_OL_CNFG2 0x19u
#define DRV8000_ADDREG_HB_SR_CNFG 0x1Au
#define DRV8000_ADDREG_HB_ITRIP_CNFG 0x1Bu
#define DRV8000_ADDREG_HB_ITRIP_FREQ 0x1Cu
#define DRV8000_ADDREG_HS_HEAT_OUT_CNFG 0x1Du
#define DRV8000_ADDREG_HS_OC_CNFG 0x1Eu
#define DRV8000_ADDREG_HS_OL_CNFG 0x1Fu
#define DRV8000_ADDREG_HS_REG_CNFG1 0x20u
#define DRV8000_ADDREG_HS_REG_CNFG2 0x21u
#define DRV8000_ADDREG_HS_PWM_FREQ_CNFG 0x22u
#define DRV8000_ADDREG_HEAT_CNFG 0x23u
#define DRV8000_ADDREG_EC_CNFG 0x24u
#define DRV8000_ADDREG_HS_OCP_DG 0x25u
#define DRV8000_ADDREG_SPARE_CNFG2 0x26u
#define DRV8000_ADDREG_SPARE_CNFG3 0x27u
#define DRV8000_ADDREG_SPARE_CNFG4 0x28u

/* **************************************************************** */
/* ************* CONTROL REGISTERS ******************************** */
/* **************************************************************** */
#define DRV8000_ADDREG_IC_CTRL 0x29u
#define DRV8000_ADDREG_GD_HB_CTRL 0x2Au
#define DRV8000_ADDREG_HS_EC_HEAT_CTRL 0x2Bu
#define DRV8000_ADDREG_OUT7_PWM_DC 0x2Cu
#define DRV8000_ADDREG_OUT8_PWM_DC 0x2Du
#define DRV8000_ADDREG_OUT9_PWM_DC 0x2Eu
#define DRV8000_ADDREG_OUT10_PWM_DC 0x2Fu
#define DRV8000_ADDREG_OUT11_PWM_DC 0x30u
#define DRV8000_ADDREG_OUT12_PWM_DC 0x31u

/* ********************************************************************/
/* ***            Register Default Values                           ***/
/* ********************************************************************/

/* **************************************************************** */
/* ************* STATUS REGISTERS ******************************** */
/* **************************************************************** */
#define DRV8000_DEFVAL_IC_STAT1 0x8000u // SPI_OK = 1; POR = 0
#define DRV8000_DEFVAL_IC_STAT2 0x00u
#define DRV8000_DEFVAL_GD_STAT 0x00u
#define DRV8000_DEFVAL_HB_STAT1 0x00u
#define DRV8000_DEFVAL_HB_STAT2 0x00u
#define DRV8000_DEFVAL_EC_HEAT_IT_RIP_STAT 0x00u
#define DRV8000_DEFVAL_HS_STAT 0x00u
#define DRV8000_DEFVAL_DEVICE_ID 0x01u

/* **************************************************************** */
/* ************* CONFIG  REGISTERS ******************************** */
/* **************************************************************** */
#define DRV8000_DEFVAL_IC_CNFG1 0x02u
#define DRV8000_DEFVAL_IC_CNFG2 0x00u
#define DRV8000_DEFVAL_GD_CNFG 0x00u
#define DRV8000_DEFVAL_GD_IDRV_CNFG 0xFFFFu
#define DRV8000_DEFVAL_GD_VGS_CNFG 0x30u
#define DRV8000_DEFVAL_GD_VDS_CNFG 0xD2Du
#define DRV8000_DEFVAL_GD_CSA_CNFG 0x04u
#define DRV8000_DEFVAL_GD_AGD_CNFG 0x402u
#define DRV8000_DEFVAL_GD_PDR_CNFG 0xAF6u
#define DRV8000_DEFVAL_GD_STC_CNFG 0x26u
#define DRV8000_DEFVAL_HB_ITRIP_DG 0x00u
#define DRV8000_DEFVAL_HB_OUT_CNFG1 0x00u
#define DRV8000_DEFVAL_HB_OUT_CNFG2 0x00u
#define DRV8000_DEFVAL_HB_OCP_CNFG 0x00u
#define DRV8000_DEFVAL_HB_OL_CNFG1 0x00u
#define DRV8000_DEFVAL_HB_OL_CNFG2 0x00u
#define DRV8000_DEFVAL_HB_SR_CNFG 0x00u
#define DRV8000_DEFVAL_HB_ITRIP_CNFG 0x00u
#define DRV8000_DEFVAL_HB_ITRIP_FREQ 0x00u
#define DRV8000_DEFVAL_HS_HEAT_OUT_CNFG 0x00u
#define DRV8000_DEFVAL_HS_OC_CNFG 0x1000u
#define DRV8000_DEFVAL_HS_OL_CNFG 0x00u
#define DRV8000_DEFVAL_HS_REG_CNFG1 0x00u
#define DRV8000_DEFVAL_HS_REG_CNFG2 0x00u
#define DRV8000_DEFVAL_HS_PWM_FREQ_CNFG 0x00u
#define DRV8000_DEFVAL_HEAT_CNFG 0xA3Cu
#define DRV8000_DEFVAL_EC_CNFG 0x00u
#define DRV8000_DEFVAL_HS_OCP_DG 0x00u

/* **************************************************************** */
/* ************* CONTROL REGISTERS ******************************** */
/* **************************************************************** */
#define DRV8000_DEFVAL_IC_CTRL 0x6Cu
#define DRV8000_DEFVAL_GD_HB_CTRL 0x00u
#define DRV8000_DEFVAL_HS_EC_HEAT_CTRL 0x00u
#define DRV8000_DEFVAL_OUT7_PWM_DC 0x00u
#define DRV8000_DEFVAL_OUT8_PWM_DC 0x00u
#define DRV8000_DEFVAL_OUT9_PWM_DC 0x00u
#define DRV8000_DEFVAL_OUT10_PWM_DC 0x00u
#define DRV8000_DEFVAL_OUT11_PWM_DC 0x00u
#define DRV8000_DEFVAL_OUT12_PWM_DC 0x00u

/* **********************************************************************/
/* ***            Declaration of global register types                ***/
/* **********************************************************************/

/* **************************************************************** */
/* ************* STATUS REGISTERS ********************************* */
/* **************************************************************** */
typedef struct
{
    uint16_t OUT7_ITRIP_TO : 1;
    uint16_t ITRIP : 1;
    uint16_t WD_FLT : 1;
    uint16_t OTSD : 1;
    uint16_t OTW : 1;
    uint16_t VCP_UV : 1;
    uint16_t PVDD_OV : 2;
    uint16_t PVDD_UV : 1;
    uint16_t HS : 1;
    uint16_t EC_HEAT : 2;
    uint16_t HB : 2;
    uint16_t GD : 1;
    uint16_t WARN : 1;
    uint16_t FAULT : 1;
    uint16_t POR : 1;
    uint16_t SPI_OK : 1;
} REG_IC_STAT1_R_00h_t;

typedef struct
{
    uint16_t ZONE1_OTW_L : 1;
    uint16_t ZONE2_OTW_L : 1;
    uint16_t ZONE3_OTW_L : 1;
    uint16_t ZONE4_OTW_L : 1;
    uint16_t ZONE1_OTW_H : 1;
    uint16_t ZONE2_OTW_H : 1;
    uint16_t ZONE3_OTW_H : 1;
    uint16_t ZONE4_OTW_H : 1;
    uint16_t ZONE1_OTSD : 1;
    uint16_t ZONE2_OTSD : 1;
    uint16_t ZONE3_OTSD : 1;
    uint16_t ZONE4_OTSD : 1;
    uint16_t : 1;
    uint16_t SCLK_FLT : 1;
    uint16_t : 2;
} REG_IC_STAT2_R_01h_t;

typedef struct
{
    uint16_t VDS_H1 : 1;
    uint16_t VDS_L1 : 1;
    uint16_t VDS_H2 : 1;
    uint16_t VDS_L2 : 1;
    uint16_t VGS_H1 : 1;
    uint16_t VGS_L1 : 1;
    uint16_t VGS_H2 : 1;
    uint16_t VGS_L2 : 1;
    uint16_t IDIR : 1;
    uint16_t IDIR_WARN : 1;
    uint16_t PDCHR_WARN : 1;
    uint16_t PCHR_WARN : 1;
    uint16_t STC_WARN_F : 1;
    uint16_t STC_WARN_R : 1;
    uint16_t : 1;
    uint16_t DRVOFF_STAT : 1;
} REG_GD_STAT_R_02h_t;

typedef struct
{
    uint16_t OUT1_HS_OCP : 1;
    uint16_t OUT2_HS_OCP : 1;
    uint16_t OUT3_HS_OCP : 1;
    uint16_t OUT4_HS_OCP : 1;
    uint16_t OUT5_HS_OCP : 1;
    uint16_t OUT6_HS_OCP : 1;
    uint16_t : 2;
    uint16_t OUT1_LS_OCP : 1;
    uint16_t OUT2_LS_OCP : 1;
    uint16_t OUT3_LS_OCP : 1;
    uint16_t OUT4_LS_OCP : 1;
    uint16_t OUT5_LS_OCP : 1;
    uint16_t OUT6_LS_OCP : 1;
    uint16_t : 2;
} REG_HB_STAT1_R_03h_t;

typedef struct
{
    uint16_t OUT1_OLA : 1;
    uint16_t OUT2_OLA : 1;
    uint16_t OUT3_OLA : 1;
    uint16_t OUT4_OLA : 1;
    uint16_t OUT5_OLA : 1;
    uint16_t OUT6_OLA : 1;
    uint16_t : 2;
    uint16_t HB_OLP_STAT : 1;
    uint16_t : 7;
} REG_HB_STAT2_R_04h_t;

typedef struct
{
    uint16_t OUT1_ITRIP_STAT : 1;
    uint16_t OUT2_ITRIP_STAT : 1;
    uint16_t OUT3_ITRIP_STAT : 1;
    uint16_t OUT4_ITRIP_STAT : 1;
    uint16_t OUT5_ITRIP_STAT : 1;
    uint16_t OUT6_ITRIP_STAT : 1;
    uint16_t OUT7_ITRIP_STAT : 1;
    uint16_t OUT7_ITRIP_TO : 1;
    uint16_t HEAT_VDS : 1;
    uint16_t HEAT_OL : 1;
    uint16_t ECFB_OL : 1;
    uint16_t ECFB_OC : 1;
    uint16_t ECFB_LO : 1;
    uint16_t ECFB_HI : 1;
    uint16_t ECFB_OV : 1;
    uint16_t ECFB_UV : 1;
} REG_EC_HEAT_ITRIP_STAT_R_05h_t;

typedef struct
{
    uint16_t OUT7_OCP : 1;
    uint16_t OUT8_OCP : 1;
    uint16_t OUT9_OCP : 1;
    uint16_t OUT10_OCP : 1;
    uint16_t OUT11_OCP : 1;
    uint16_t OUT12_OCP : 1;
    uint16_t : 2;
    uint16_t OUT7_OLA : 1;
    uint16_t OUT8_OLA : 1;
    uint16_t OUT9_OLA : 1;
    uint16_t OUT11_OLA : 1;
    uint16_t OUT10_OLA : 1;
    uint16_t OUT12_OLA : 1;
    uint16_t : 2;
} REG_HS_STAT_R_06h_t;

// typedef struct
// {
//         uint16_t                        : 16;
// } REG_SPARE_STAT1_R_07h_t;

typedef struct
{
    uint16_t DEVICE_ID : 8;
    uint16_t : 8;
} REG_SPARE_STAT2_R_08h_t;

/* **************************************************************** */
/* ************* CONFIG REGISTERS ********************************* */
/* **************************************************************** */
typedef struct
{
    uint16_t EN_SSC : 1;
    uint16_t WD_WIN : 1;
    uint16_t WD_FLT_M : 1;
    uint16_t WD_EN : 1;
    uint16_t PVDD_UV_MODE : 1;
    uint16_t VCP_UV_MODE : 1;
    uint16_t CP_MODE : 2;
    uint16_t VCP_UV_LVL : 1;
    uint16_t PVDD_OV_LVL : 1;
    uint16_t PVDD_OV_DG : 2;
    uint16_t PVDD_OV_MODE : 2;
    uint16_t DIS_CP : 1;
    uint16_t OTSD_MODE : 1;
} REG_IC_CNFG1_RW_09h_t;

typedef struct
{
    uint16_t ZONE1_OTW_L_DIS : 1;
    uint16_t ZONE2_OTW_L_DIS : 1;
    uint16_t ZONE3_OTW_L_DIS : 1;
    uint16_t ZONE4_OTW_L_DIS : 1;
    uint16_t ZONE1_OTW_H_DIS : 1;
    uint16_t ZONE2_OTW_H_DIS : 1;
    uint16_t ZONE3_OTW_H_DIS : 1;
    uint16_t ZONE4_OTW_H_DIS : 1;
    uint16_t : 8;
} REG_IC_CNFG2_RW_0Ah_t;

typedef struct
{
    uint16_t EN_GD : 1;
    uint16_t EN_OLSC : 1;
    uint16_t BRG_MODE : 2;
    uint16_t BRG_FW : 1;
    uint16_t IN1_MODE : 1;
    uint16_t IN2_MODE : 1;
    uint16_t : 1;
    uint16_t PD_SH_2 : 1;
    uint16_t PU_SH_2 : 1;
    uint16_t PD_SH_1 : 1;
    uint16_t PU_SH_1 : 1;
    uint16_t DRV_LO2 : 1;
    uint16_t DRV_LO1 : 1;
    uint16_t : 2;
} REG_GD_CNFG_RW_0Bh_t;

typedef struct
{
    uint16_t IDRVN_2 : 4;
    uint16_t IDRVP_2 : 4;
    uint16_t IDRVN_1 : 4;
    uint16_t IDRVP_1 : 4;
} REG_GD_IDRV_CNFG_RW_0Ch_t;

typedef struct
{
    uint16_t VGS_MODE : 2;
    uint16_t VGS_LVL : 1;
    uint16_t VGS_HS_DIS : 1;
    uint16_t VGS_TDRV : 3;
    uint16_t : 2;
    uint16_t VGS_TDEAD : 2;
    uint16_t VGS_IND : 1;
    uint16_t : 4;
} REG_GD_VGS_CNFG_RW_0Dh_t;

typedef struct
{
    uint16_t VDS_LVL_2 : 4;
    uint16_t VDS_DG : 2;
    uint16_t VDS_MODE : 2;
    uint16_t VDS_LVL_1 : 4;
    uint16_t VDS_IDRVN : 2;
    uint16_t VDS_IND : 1;
    uint16_t : 1;
} REG_GD_VDS_CNFG_RW_0Eh_t;

typedef struct
{
    uint16_t : 1;
    uint16_t CSA_DIV : 1;
    uint16_t CSA_GAIN : 2;
    uint16_t CSA_BLK_SEL : 1;
    uint16_t CSA_BLK : 3;
    uint16_t : 8;
} REG_GD_CSA_CNFG_RW_0Fh_t;

typedef struct
{
    uint16_t EN_PDR : 1;
    uint16_t KP_PDR : 2;
    uint16_t EN_PST_DLY : 1;
    uint16_t KP_PST : 2;
    uint16_t IDIR_MAN : 1;
    uint16_t EN_DCC : 1;
    uint16_t FW_MAX : 1;
    uint16_t SET_AGD : 1;
    uint16_t AGD_THR : 2;
    uint16_t AGD_ISTRONG : 2;
    uint16_t PDR_ERR : 1;
    uint16_t : 1;
} REG_GD_AGD_CNFG_RW_10h_t;

typedef struct
{
    uint16_t PRE_DCHR_INIT : 2;
    uint16_t PRE_CHR_INIT : 2;
    uint16_t T_PRE_DCHR : 2;
    uint16_t T_PRE_CHR : 2;
    uint16_t T_DON_DOFF : 6;
    uint16_t PRE_MAX : 2;
} REG_GD_PDR_CNFG_RW_11h_t;

typedef struct
{
    uint16_t EN_STC : 1;
    uint16_t KP_STC : 2;
    uint16_t STC_ERR : 1;
    uint16_t T_RISE_FALL : 4;
    uint16_t : 8;
} REG_GD_STC_CNFG_RW_12h_t;

// typedef struct
// {
//         uint16_t                        : 16;
// } REG_GD_SPARE_CNFG1_RW_13h_t;

typedef struct
{
    uint16_t OUT1_ITRIP_DG : 2;
    uint16_t OUT2_ITRIP_DG : 2;
    uint16_t OUT3_ITRIP_DG : 2;
    uint16_t OUT4_ITRIP_DG : 2;
    uint16_t OUT5_ITRIP_DG : 2;
    uint16_t OUT6_ITRIP_DG : 2;
    uint16_t : 4;
} REG_HB_ITRIP_DG_RW_14h_t;

typedef struct
{
    uint16_t OUT5_CNFG : 3;
    uint16_t OUT6_CNFG : 3;
    uint16_t : 2;
    uint16_t IPROPI_SH_EN : 1;
    uint16_t NSR_OUT1_DIS : 1;
    uint16_t NSR_OUT2_DIS : 1;
    uint16_t NSR_OUT3_DIS : 1;
    uint16_t NSR_OUT4_DIS : 1;
    uint16_t NSR_OUT5_DIS : 1;
    uint16_t NSR_OUT6_DIS : 1;
    uint16_t : 1;
} REG_HB_OUT_CNFG1_RW_15h_t;

typedef struct
{
    uint16_t OUT1_CNFG : 3;
    uint16_t OUT2_CNFG : 3;
    uint16_t : 2;
    uint16_t OUT3_CNFG : 3;
    uint16_t OUT4_CNFG : 3;
    uint16_t : 2;
} REG_HB_OUT_CNFG2_RW_16h_t;

typedef struct
{
    uint16_t OUT1_OCP_DG : 2;
    uint16_t OUT2_OCP_DG : 2;
    uint16_t OUT3_OCP_DG : 2;
    uint16_t OUT4_OCP_DG : 2;
    uint16_t OUT5_OCP_DG : 2;
    uint16_t OUT6_OCP_DG : 2;
    uint16_t : 4;
} REG_HB_OCP_CNFG_RW_17h_t;

typedef struct
{
    uint16_t OUT1_OLA_EN : 1;
    uint16_t OUT2_OLA_EN : 1;
    uint16_t OUT3_OLA_EN : 1;
    uint16_t OUT4_OLA_EN : 1;
    uint16_t OUT5_OLA_EN : 1;
    uint16_t OUT6_OLA_EN : 1;
    uint16_t : 2;
    uint16_t HB_OLP_SEL : 4;
    uint16_t HB_OLP_CNFG : 2;
    uint16_t : 2;
} REG_HB_OL_CNFG1_RW_18h_t;

typedef struct
{
    uint16_t OUT1_OLA_TH : 1;
    uint16_t OUT2_OLA_TH : 1;
    uint16_t OUT3_OLA_TH : 1;
    uint16_t OUT4_OLA_TH : 1;
    uint16_t OUT5_OLA_TH : 1;
    uint16_t OUT6_OLA_TH : 1;
    uint16_t : 10;
} REG_HB_OL_CNFG2_RW_19h_t;

typedef struct
{
    uint16_t OUT1_SR : 2;
    uint16_t OUT2_SR : 2;
    uint16_t OUT3_SR : 2;
    uint16_t OUT4_SR : 2;
    uint16_t OUT5_SR : 2;
    uint16_t OUT6_SR : 2;
    uint16_t : 4;
} REG_HB_SR_CNFG_RW_1Ah_t;

typedef struct
{
    uint16_t OUT1_ITRIP_LVL : 1;
    uint16_t OUT2_ITRIP_LVL : 1;
    uint16_t OUT3_ITRIP_LVL : 2;
    uint16_t OUT4_ITRIP_LVL : 2;
    uint16_t OUT5_ITRIP_LVL : 2;
    uint16_t OUT6_ITRIP_LVL : 2;
    uint16_t OUT1_ITRIP_EN : 1;
    uint16_t OUT2_ITRIP_EN : 1;
    uint16_t OUT3_ITRIP_EN : 1;
    uint16_t OUT4_ITRIP_EN : 1;
    uint16_t OUT5_ITRIP_EN : 1;
    uint16_t OUT6_ITRIP_EN : 1;
} REG_HB_ITRIP_CNFG_RW_1Bh_t;

typedef struct
{
    uint16_t OUT1_ITRIP_FREQ : 2;
    uint16_t OUT2_ITRIP_FREQ : 2;
    uint16_t OUT3_ITRIP_FREQ : 2;
    uint16_t OUT4_ITRIP_FREQ : 2;
    uint16_t OUT5_ITRIP_FREQ : 2;
    uint16_t OUT6_ITRIP_FREQ : 2;
    uint16_t : 4;
} REG_HB_ITRIP_FREQ_RW_1Ch_t;

typedef struct
{
    uint16_t OUT7_CNFG : 2;
    uint16_t OUT8_CNFG : 2;
    uint16_t OUT9_CNFG : 2;
    uint16_t OUT10_CNFG : 2;
    uint16_t OUT11_CNFG : 2;
    uint16_t OUT12_CNFG : 2;
    uint16_t : 2;
    uint16_t HEAT_OUT_CNFG : 2;
} REG_HS_HEAT_OUT_CNFG_RW_1Dh_t;

typedef struct
{
    uint16_t OUT7_RDSON_MODE : 1;
    uint16_t OUT8_OC_TH : 1;
    uint16_t OUT9_OC_TH : 1;
    uint16_t OUT10_OC_TH : 1;
    uint16_t OUT11_OC_TH : 1;
    uint16_t OUT12_OC_TH : 1;
    uint16_t : 6;
    uint16_t OUT11_EC_MODE : 1;
    uint16_t : 3;
} REG_HS_OC_CNFG_RW_1Eh_t;

typedef struct
{
    uint16_t OUT7_OLA_EN : 1;
    uint16_t OUT8_OLA_EN : 1;
    uint16_t OUT9_OLA_EN : 1;
    uint16_t OUT10_OLA_EN : 1;
    uint16_t OUT11_OLA_EN : 1;
    uint16_t OUT12_OLA_EN : 1;
    uint16_t : 2;
    uint16_t OUT7_OLA_TH : 1;
    uint16_t OUT8_OLA_TH : 1;
    uint16_t OUT9_OLA_TH : 1;
    uint16_t OUT10_OLA_TH : 1;
    uint16_t OUT11_OLA_TH : 1;
    uint16_t OUT12_OLA_TH : 1;
    uint16_t : 2;
} REG_HS_OL_CNFG_RW_1Fh_t;

typedef struct
{
    uint16_t OUT7_ITRIP_DG : 2;
    uint16_t OUT7_ITRIP_FREQ : 2;
    uint16_t OUT7_ITRIP_BLK : 2;
    uint16_t OUT7_ITRIP_CNFG : 2;
    uint16_t ITRIP_TO_SEL : 2;
    uint16_t OUT7_OCP_DIS : 1;
    uint16_t : 5;
} REG_HS_REG_CNFG1_RW_20h_t;

typedef struct
{
    uint16_t OUT7_CCM_EN : 1;
    uint16_t OUT8_CCM_EN : 1;
    uint16_t OUT9_CCM_EN : 1;
    uint16_t OUT10_CCM_EN : 1;
    uint16_t OUT11_CCM_EN : 1;
    uint16_t OUT12_CCM_EN : 1;
    uint16_t : 2;
    uint16_t OUT7_CCM_TO : 1;
    uint16_t OUT8_CCM_TO : 1;
    uint16_t OUT9_CCM_TO : 1;
    uint16_t OUT10_CCM_TO : 1;
    uint16_t OUT11_CCM_TO : 1;
    uint16_t OUT12_CCM_TO : 1;
    uint16_t : 2;
} REG_HS_REG_CNFG2_RW_21h_t;

typedef struct
{
    uint16_t PWM_OUT7_FREQ : 2;
    uint16_t PWM_OUT8_FREQ : 2;
    uint16_t PWM_OUT9_FREQ : 2;
    uint16_t PWM_OUT10_FREQ : 2;
    uint16_t PWM_OUT11_FREQ : 2;
    uint16_t PWM_OUT12_FREQ : 2;
    uint16_t : 4;
} REG_HS_PWM_FREQ_CNFG_RW_22h_t;

typedef struct
{
    uint16_t : 1;
    uint16_t HEAT_OLP_EN : 1;
    uint16_t HEAT_VDS_DG : 2;
    uint16_t HEAT_VDS_BLK : 2;
    uint16_t HEAT_VDS_MODE : 2;
    uint16_t HEAT_VDS_LVL : 4;
    uint16_t : 4;
} REG_HEAT_CNFG_RW_23h_t;

typedef struct
{
    uint16_t ECFB_MAX : 1;
    uint16_t EC_OLEN : 1;
    uint16_t ECFB_LS_PWM : 1;
    uint16_t EC_FLT_MODE : 1;
    uint16_t ECFB_OV_MODE : 2;
    uint16_t ECFB_UV_MODE : 2;
    uint16_t ECFB_OV_DG : 2;
    uint16_t ECFB_UV_DG : 2;
    uint16_t : 2;
    uint16_t ECFB_UV_TH : 1;
    uint16_t ECDRV_OL_EN : 1;
} REG_EC_CNFG_RW_24h_t;

typedef struct
{
    uint16_t OUT7_OCP_DG : 2;
    uint16_t OUT8_OCP_DG : 2;
    uint16_t OUT9_OCP_DG : 2;
    uint16_t OUT10_OCP_DG : 2;
    uint16_t OUT11_OCP_DG : 2;
    uint16_t OUT12_OCP_DG : 2;
    uint16_t : 4;
} REG_HS_OCP_DG_RW_25h_t;

// typedef struct
// {
//         uint16_t                        : 16;
// } REG_SPARE_CNFG2_RW_26h_t;

// typedef struct
// {
//         uint16_t                        : 16;
// } REG_SPARE_CNFG3_RW_27h_t;

// typedef struct
// {
//         uint16_t                        : 16;
// } REG_SPARE_CNFG4_RW_28h_t;

/* **************************************************************** */
/* ************* CONTROL REGISTERS ******************************** */
/* **************************************************************** */
typedef struct
{
    uint16_t CLR_FLT : 1;
    uint16_t WD_RST : 1;
    uint16_t CNFG_LOCK : 3;
    uint16_t CTRL_LOCK : 3;
    uint16_t IPROPI_SEL : 5;
    uint16_t IPROPI_MODE : 1;
    uint16_t : 2;
} REG_IC_CTRL_RW_29h_t;

typedef struct
{
    uint16_t OUT2_CTRL : 2;
    uint16_t OUT1_CTRL : 2;
    uint16_t OUT3_CTRL : 2;
    uint16_t OUT4_CTRL : 2;
    uint16_t OUT5_CTRL : 2;
    uint16_t OUT6_CTRL : 2;
    uint16_t S_IN1 : 1;
    uint16_t S_IN2 : 1;
    uint16_t S_HIZ1 : 1;
    uint16_t S_HIZ2 : 1;
} REG_GD_HB_CTRL_RW_2Ah_t;

typedef struct
{
    uint16_t OUT7_EN : 1;
    uint16_t OUT8_EN : 1;
    uint16_t OUT9_EN : 1;
    uint16_t OUT10_EN : 1;
    uint16_t OUT11_EN : 1;
    uint16_t OUT12_EN : 1;
    uint16_t : 1;
    uint16_t HEAT_EN : 1;
    uint16_t EC_V_TAR : 6;
    uint16_t EC_ON : 1;
    uint16_t ECFB_LS_EN : 1;
} REG_HS_EC_HEAT_CTRL_RW_2Bh_t;

typedef struct
{
    uint16_t OUT7_DC : 10;
    uint16_t : 6;
} REG_OUT7_PWM_DC_RW_2Ch_t;

typedef struct
{
    uint16_t OUT8_DC : 10;
    uint16_t : 6;
} REG_OUT8_PWM_DC_RW_2Dh_t;

typedef struct
{
    uint16_t OUT9_DC : 10;
    uint16_t : 6;
} REG_OUT9_PWM_DC_RW_2Eh_t;

typedef struct
{
    uint16_t OUT10_DC : 10;
    uint16_t : 6;
} REG_OUT10_PWM_DC_RW_2Fh_t;

typedef struct
{
    uint16_t OUT11_DC : 10;
    uint16_t : 6;
} REG_OUT11_PWM_DC_RW_30h_t;

typedef struct
{
    uint16_t OUT12_DC : 10;
    uint16_t : 6;
} REG_OUT12_PWM_DC_RW_31h_t;

/* **********************************************************************/
/* ***               Definition of global types                       ***/
/* **********************************************************************/
typedef union
{
    /* **************************************************************** */
    /* ************* STATUS REGISTERS ********************************* */
    /* **************************************************************** */
    REG_IC_STAT1_R_00h_t Reg_IC_STAT1;
    REG_IC_STAT2_R_01h_t Reg_IC_STAT2;
    REG_GD_STAT_R_02h_t Reg_GD_STAT;
    REG_HB_STAT1_R_03h_t Reg_HB_STAT1;
    REG_HB_STAT2_R_04h_t Reg_HB_STAT2;
    REG_EC_HEAT_ITRIP_STAT_R_05h_t Reg_EC_HEAT_ITRIP_STAT;
    REG_HS_STAT_R_06h_t Reg_HS_STAT;
    // REG_SPARE_STAT1_R_07h_t                  Reg_SPARE_STAT1; // spare reg
    // REG_SPARE_STAT2_R_08h_t                  Reg_DEV_ID;

    /* **************************************************************** */
    /* ************* CONFIG REGISTERS ********************************* */
    /* **************************************************************** */
    REG_IC_CNFG1_RW_09h_t Reg_IC_CNFG1;
    REG_IC_CNFG2_RW_0Ah_t Reg_IC_CNFG2;
    REG_GD_CNFG_RW_0Bh_t Reg_GD_CNFG;
    REG_GD_IDRV_CNFG_RW_0Ch_t Reg_GD_IDRV_CNFG;
    REG_GD_VGS_CNFG_RW_0Dh_t Reg_GD_VGS_CNFG;
    REG_GD_VDS_CNFG_RW_0Eh_t Reg_GD_VDS_CNFG;
    REG_GD_CSA_CNFG_RW_0Fh_t Reg_GD_CSA_CNFG;
    REG_GD_AGD_CNFG_RW_10h_t Reg_GD_AGD_CNFG;
    REG_GD_PDR_CNFG_RW_11h_t Reg_GD_PDR_CNFG;
    REG_GD_STC_CNFG_RW_12h_t Reg_GD_STC_CNFG;
    // REG_GD_SPARE_CNFG1_RW_13h_t              Reg_GD_SPARE_CNFG1; // spare reg
    REG_HB_ITRIP_DG_RW_14h_t Reg_HB_ITRIP_DG;
    REG_HB_OUT_CNFG1_RW_15h_t Reg_HB_OUT_CNFG1;
    REG_HB_OUT_CNFG2_RW_16h_t Reg_HB_OUT_CNFG2;
    REG_HB_OCP_CNFG_RW_17h_t Reg_HB_OCP_CNFG;
    REG_HB_OL_CNFG1_RW_18h_t Reg_HB_OL_CNFG1;
    REG_HB_OL_CNFG2_RW_19h_t Reg_HB_OL_CNFG2;
    REG_HB_SR_CNFG_RW_1Ah_t Reg_HB_SR_CNFG;
    REG_HB_ITRIP_CNFG_RW_1Bh_t Reg_HB_ITRIP_CNFG;
    REG_HB_ITRIP_FREQ_RW_1Ch_t Reg_HB_ITRIP_FREQ;
    REG_HS_HEAT_OUT_CNFG_RW_1Dh_t Reg_HS_HEAT_OUT_CNFG;
    REG_HS_OC_CNFG_RW_1Eh_t Reg_HS_OC_CNFG;
    REG_HS_OL_CNFG_RW_1Fh_t Reg_HS_OL_CNFG;
    REG_HS_REG_CNFG1_RW_20h_t Reg_HS_REG_CNFG1;
    REG_HS_REG_CNFG2_RW_21h_t Reg_HS_REG_CNFG2;
    REG_HS_PWM_FREQ_CNFG_RW_22h_t Reg_HS_PWM_FREQ;
    REG_HEAT_CNFG_RW_23h_t Reg_HEAT_CNFG;
    REG_EC_CNFG_RW_24h_t Reg_EC_CNFG;
    REG_HS_OCP_DG_RW_25h_t Reg_HS_OCP_DG;
    // REG_SPARE_CNFG2_RW_26h_t                 Reg_SPARE_CNFG2; // spare reg
    // REG_SPARE_CNFG3_RW_27h_t                 Reg_SPARE_CNFG3; // spare reg
    // REG_SPARE_CNFG4_RW_28h_t                 Reg_SPARE_CNFG4; // spare reg

    /* **************************************************************** */
    /* ************* CONTROL REGISTERS ******************************** */
    /* **************************************************************** */
    REG_IC_CTRL_RW_29h_t Reg_IC_CTRL;
    REG_GD_HB_CTRL_RW_2Ah_t Reg_GD_HB_CTRL;
    REG_HS_EC_HEAT_CTRL_RW_2Bh_t Reg_HS_EC_HEAT_CTRL;
    REG_OUT7_PWM_DC_RW_2Ch_t Reg_OUT7_PWM_DC;
    REG_OUT8_PWM_DC_RW_2Dh_t Reg_OUT8_PWM_DC;
    REG_OUT9_PWM_DC_RW_2Eh_t Reg_OUT9_PWM_DC;
    REG_OUT10_PWM_DC_RW_2Fh_t Reg_OUT10_PWM_DC;
    REG_OUT11_PWM_DC_RW_30h_t Reg_OUT11_PWM_DC;
    REG_OUT12_PWM_DC_RW_31h_t Reg_OUT12_PWM_DC;

    uint16_t u16_RegWord;
} un_DRV8000_Reg_t;

#endif /* GDU_TI_DRV8000_DRV8000_REG_H_ */
