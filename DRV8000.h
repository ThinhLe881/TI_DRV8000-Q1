/*
 * DRV8000.h
 *
 *  Created on: Mar 3, 2025
 *      Author: thinh
 */

#ifndef GDU_TI_DRV8000_DRV8000_H_
#define GDU_TI_DRV8000_DRV8000_H_

/* **********************************************************************/
/* ***              System and library files included                 ***/
/* **********************************************************************/
#include "DRV8000_Reg.h"


/* **********************************************************************/
/* ***            Definition of global plain CONSTants                ***/
/* **********************************************************************/
#define DRV8000_REG_ADDRESS_MASK								0x3Fu
#define DRV8000_REG_DATA_MASK									0xFFFFu
#define DRV8000_GLOBAL_STATUS_MASK								0x3Fu /* remove the first 2 bits (always 1) */
#define DRV8000_DEVICE_ID_MASK                                  0xFFu
#define DRV8000_WRITE_ACCESS                                    0u
#define DRV8000_READ_ACCESS                                     1u
#define DRV8000_SPI_FRAME_LEN                                   4u /* 24 bits + 8 bits padding = 4 8-bit buffers */
#define DRV8000_SUCCESS_SPI_STATUS                              0x0u /* 0     0    0     0   0    0         */
                                                                     /* FAULT WARN OV_UV DRV OTSD SPI_ERR   */
#define DRV8000_MAX_GEN_PWM_DUTYCYCLE                           0x3FFu /* 10-bit */


/* **********************************************************************/
/* ***               Definition of global types                       ***/
/* **********************************************************************/
typedef union {
    struct {
        uint8_t SPI_ERR         : 1;
        uint8_t OTSD            : 1;
        uint8_t DRV             : 1;
        uint8_t OV_UV			: 1;
        uint8_t WARN 		    : 1;
        uint8_t FAULT		    : 1;
        uint8_t Ignored       	: 2;
    } st_SpiCommStatus;
    uint8_t u8_SpiCommStatus;
} un_DRV8000_SPI_STST_t;

typedef union {
    struct {
        uint32_t DataField	    : 16;
        uint32_t SpiStastus     : 8;
        uint32_t Ignored        : 8;
    } st_SpiCommand;
    uint32_t u32_SpiCommand;
} un_DRV8000_SDO_FRAME_t;

typedef union {
    struct {
        uint32_t DataField	    : 16;
        uint32_t Address	    : 6;
        uint32_t AccessType		: 1;
        uint32_t MSBBit			: 1; /* always 0 for standard frame */
        uint32_t Ignored        : 8;
    } st_SpiCommand;
    uint32_t u32_SpiCommand;
} un_DRV8000_SDI_FRAME_t;

typedef enum {
/* STATUS registers */
    REGID_IC_STAT1                     ,
    REGID_IC_STAT2                     ,
    REGID_GD_STAT                      ,
    REGID_HB_STAT1                     ,
    REGID_HB_STAT2                     ,
    REGID_EC_HEAT_IT_RIP_STAT          ,
    REGID_HS_STAT                      ,
/* CONFIG registers */
    /* REGID_IC_CNFG1                     , */
    /* REGID_IC_CNFG2                     , */
    /* REGID_GD_CNFG                      , */
    /* REGID_GD_IDRV_CNFG                 , */
    /* REGID_GD_VGS_CNFG                  , */
    /* REGID_GD_VDS_CNFG                  , */
    /* REGID_GD_CSA_CNFG                  , */
    /* REGID_GD_AGD_CNFG                  , */
    /* REGID_GD_PDR_CNFG                  , */
    /* REGID_GD_STC_CNFG                  , */
    /* REGID_HB_ITRIP_DG                  , */
    /* REGID_HB_OUT_CNFG1                 , */
    /* REGID_HB_OUT_CNFG2                 , */
    /* REGID_HB_OCP_CNFG                  , */
    /* REGID_HB_OL_CNFG1                  , */
    /* REGID_HB_OL_CNFG2                  , */
    /* REGID_HB_SR_CNFG                   , */
    /* REGID_HB_ITRIP_CNFG                , */
    /* REGID_HB_ITRIP_FREQ                , */
    REGID_HS_HEAT_OUT_CNFG             ,
    /* REGID_HS_OC_CNFG                   , */
    /* REGID_HS_OL_CNFG                   , */
    REGID_HS_REG_CNFG1                 ,
    REGID_HS_REG_CNFG2                 ,
    REGID_HS_PWM_FREQ_CNFG             ,
    /* REGID_HEAT_CNFG                    , */
    /* REGID_EC_CNFG                      , */
    /* REGID_HS_OCP_DG                    , */
/* CONTROL registers */
    REGID_IC_CTRL                      ,
    REGID_GD_HB_CTRL                   ,
    REGID_HS_EC_HEAT_CTRL              ,
    REGID_OUT7_PWM_DC                  ,
    REGID_OUT8_PWM_DC                  ,
    REGID_OUT9_PWM_DC                  ,
    REGID_OUT10_PWM_DC                 ,
    REGID_OUT11_PWM_DC                 ,
    REGID_OUT12_PWM_DC                 ,

    DRV8000_NUM_OF_REGS		           ,
} en_REG_ID_t;

#endif /* GDU_TI_DRV8000_DRV8000_H_ */
