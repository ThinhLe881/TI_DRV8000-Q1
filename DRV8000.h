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
/* ***            Definition of global macros                         ***/
/* **********************************************************************/
#define GDU_PWM_PERIOD_FIXED

#define GDU_GD_USED
#ifdef GDU_GD_USED
#define GDU_GD_IN2_GPIO
#endif /* GDU_GD_USED */

#define GDU_HHB_USED
#ifdef GDU_HHB_USED
#define GDU_PWM2_USED
#endif /* GDU_HHB_USED */

#define GDU_HS_USED


/* **********************************************************************/
/* ***            Definition of global plain CONSTants                ***/
/* **********************************************************************/
#define DRV8000_REG_ADDRESS_MASK								0x3Fu
#define DRV8000_REG_DATA_MASK									0xFFFFu
#define DRV8000_GLOBAL_STATUS_MASK								0x3Fu       /* remove the first 2 bits (always 1) */
#define DRV8000_DEVICE_ID_MASK                                  0xFFu
#define DRV8000_GEN_PWM_DC_MASK                                 0x3FFu      /* 10 bits */
#define DRV8000_EC_V_TAR_MASK                                   0x3Fu       /* 6 bits */

#define DRV8000_WRITE_ACCESS                                    0u
#define DRV8000_READ_ACCESS                                     1u
#define DRV8000_SPI_FRAME_LEN                                   4u          /* 24 bits + 8 bits padding = 4 8-bit buffers */
#define DRV8000_SUCCESS_SPI_STATUS                              0x0u        /* 0     0    0     0   0    0       */
                                                                            /* FAULT WARN OV_UV DRV OTSD SPI_ERR */
#define DRV8000_REG_NOT_CHANGE                                  0x1FFFF     /* Flag for unchange bits, currently only for certain bitfields */


/* **********************************************************************/
/* ***               Definition of global types                       ***/
/* **********************************************************************/
typedef uint8_t (*DRV8000_SPI_Transceive_fptr)(const uint8_t*, uint8_t*, const uint16_t);
typedef uint16_t (*DRV8000_Pwm_Set_Dutycycle_fptr)(uint8_t, uint8_t, uint16_t);
typedef void (*DRV8000_Pwm_Set_Period_fptr)(uint8_t, uint8_t, uint16_t);
typedef uint8_t (*DRV8000_GPIO_Set_fptr)(uint8_t, uint8_t, uint8_t);
typedef void (*DRV8000_Delay_fptr)(uint16_t); /* Delay in us */

typedef struct
{
    DRV8000_SPI_Transceive_fptr         fptr_SpiTransceive;
    DRV8000_Pwm_Set_Period_fptr         fptr_PwmSetPeriod;
    DRV8000_Pwm_Set_Dutycycle_fptr      fptr_PwmSetDutycycle;
    DRV8000_GPIO_Set_fptr               fptr_Gpio;
    DRV8000_Delay_fptr                  fptr_Delay;
    uint16_t pwm_max_period;
	uint8_t pwm1_instance;
	uint8_t pwm1_channel;
#ifdef GDU_HHB_USED
#ifdef GDU_PWM2_USED
	uint8_t pwm2_instance;
	uint8_t pwm2_channel;
#endif /* GDU_PWM2_USED */
#endif /* GDU_HHB_USED */
#ifdef GDU_GD_USED
    uint8_t pwm_gd_in1_instance;
    uint8_t pwm_gd_in1_channel;
#ifdef GDU_GD_IN2_GPIO
        uint8_t gd_in2_port;
        uint8_t gd_in2_pin;
#else /* not defined GDU_GD_IN2_GPIO */
        uint8_t pwm_gd_in2_instance;
        uint8_t pwm_gd_in2_channel;
#endif /* GDU_GD_IN2_GPIO */
    uint8_t drvoff_port;
    uint8_t drvoff_pin;
#endif /* GDU_GD_USED */
    uint8_t nsleep_port;
    uint8_t nsleep_pin;
} st_DRV8000_Interface_t;

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
        uint32_t MSBBit			: 1;  /* always 0 for standard frame */
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
    REGID_IC_CNFG1                     ,
    REGID_IC_CNFG2                     ,
    REGID_GD_CNFG                      ,
    REGID_GD_IDRV_CNFG                 ,
    REGID_GD_VGS_CNFG                  ,
    REGID_GD_VDS_CNFG                  ,
    REGID_GD_CSA_CNFG                  ,
    REGID_GD_AGD_CNFG                  ,
    REGID_GD_PDR_CNFG                  ,
    REGID_GD_STC_CNFG                  ,
    REGID_HB_ITRIP_DG                  ,
    REGID_HB_OUT_CNFG1                 ,
    REGID_HB_OUT_CNFG2                 ,
    REGID_HB_OCP_CNFG                  ,
    REGID_HB_OL_CNFG1                  ,
    REGID_HB_OL_CNFG2                  ,
    REGID_HB_SR_CNFG                   ,
    REGID_HB_ITRIP_CNFG                ,
    REGID_HB_ITRIP_FREQ                ,
    REGID_HS_HEAT_OUT_CNFG             ,
    REGID_HS_OC_CNFG                   ,
    REGID_HS_OL_CNFG                   ,
    REGID_HS_REG_CNFG1                 ,
    REGID_HS_REG_CNFG2                 ,
    REGID_HS_PWM_FREQ_CNFG             ,
    REGID_HEAT_CNFG                    ,
    REGID_EC_CNFG                      ,
    REGID_HS_OCP_DG                    ,
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

typedef enum
{
    UNLOCK_REG_WRITE = 0x3u, /* b011 */
    LOCK_REG_WRITE   = 0x6u, /* b110 */
} en_LOCK_UNLOCK_REG_WRITE_t;

typedef enum
{
    IPROPI_OUTPUT_ADC,
    IPROPI_INPUT_PWM,
} en_IPROPI_MODE_t;

typedef enum
{
    IPROPI_NO_OUT,
    IPROPI_OUT1,
    IPROPI_OUT2,
    IPROPI_OUT3,
    IPROPI_OUT4,
    IPROPI_OUT5,
    IPROPI_OUT6,
    IPROPI_OUT7,
    IPROPI_OUT8,
    IPROPI_OUT9,
    IPROPI_OUT10,
    IPROPI_OUT11,
    IPROPI_OUT12,
    IPROPI_RESERVED,
    IPROPI_PVDD,
    IPROPI_TC1,
    IPROPI_TC2,
    IPROPI_TC3,
    IPROPI_TC4,
} en_IPROPI_SEL_MUX_t;

/* ** High Side Driver Control ** */
typedef enum
{
    HS_OUT_7,
    HS_OUT_8,
    HS_OUT_9,
    HS_OUT_10,
    HS_OUT_11,
    HS_OUT_12,
} en_HS_OUTx_t;

typedef enum
{
    HS_CNFG_DISABLED,
    HS_CNFG_SPI_CONTROL,
    HS_CNFG_PWM_PIN_CONTROL,
    HS_CNFG_PWM_GEN,
    HS_CNFG_REG_NOT_CHANGE = DRV8000_REG_NOT_CHANGE,
} en_HS_CNFG_t;

typedef enum
{
    HS_DISABLE,
    HS_ENABLE,
    HS_EN_REG_NOT_CHANGE = DRV8000_REG_NOT_CHANGE,
} en_HS_EN_t;

typedef enum
{
    HS_GEN_PWM_FREQ_108_Hz,
    HS_GEN_PWM_FREQ_217_Hz,
    HS_GEN_PWM_FREQ_289_Hz,
} en_HS_GEN_PWM_FREQ_t;

typedef enum
{
    HS_CCM_200mA_20ms,
    HS_CCM_390mA_10ms,
} en_HS_CCM_TO_t;

typedef enum
{
    HS_CCM_DISABLE,
    HS_CCM_ENABLE,
} en_HS_CCM_EN_t;

typedef enum
{
    HS_OUT7_OCP_ENABLE,
    HS_OUT7_OCP_DISABLE,
} en_HS_OUT7_OCP_DIS_t;

typedef enum
{
    HS_OUT7_ITRIP_TIMEOUT_100ms,
    HS_OUT7_ITRIP_TIMEOUT_200ms,
    HS_OUT7_ITRIP_TIMEOUT_400ms,
    HS_OUT7_ITRIP_TIMEOUT_800ms,
} en_HS_OUT7_ITRIP_TIMEOUT_t;

typedef enum
{
    HS_OUT7_ITRIP_FAULT_REPORT_ONLY,
    HS_OUT7_ITRIP_REGULATION_TIMEOUT_DRIVER_DISABLE,
    HS_OUT7_ITRIP_REGULATION_ALWAYS,
    HS_OUT7_ITRIP_REGULATION_TIMEOUT_REGULATION_DISABLE,
} en_HS_OUT7_ITRIP_CNFG_t;

typedef enum
{
    HS_OUT7_ITRIP_BLK_0us  = 0x1u,
    HS_OUT7_ITRIP_BLK_20us = 0x2u,
    HS_OUT7_ITRIP_BLK_40us = 0x3u,
} en_HS_OUT7_ITRIP_BLK_t;

typedef enum
{
    HS_OUT7_ITRIP_FREQ_1_7kHz,
    HS_OUT7_ITRIP_FREQ_2_2kHz,
    HS_OUT7_ITRIP_FREQ_3kHz,
    HS_OUT7_ITRIP_FREQ_4_4kHz,
} en_HS_OUT7_ITRIP_FREQ_t;

typedef enum
{
    HS_OUT7_ITRIP_DG_48us,
    HS_OUT7_ITRIP_DG_40us,
    HS_OUT7_ITRIP_DG_32us,
    HS_OUT7_ITRIP_DG_24us,
} en_HS_OUT7_ITRIP_DG_t;

typedef enum
{
    HS_OC_LOW_CURRENT_TH,   /* 250mA or 500mA for OUT7 */
    HS_OC_HIGH_CURRENT_TH,  /* 500mA or 1.5A for OUT7 */
} en_HS_OC_TH_t;

typedef enum
{
    HS_OCP_DG_6us,
    HS_OCP_DG_10us,
    HS_OCP_DG_20us,
    HS_OCP_DG_60us,
} en_HS_OCP_DG_t;

typedef enum
{
    HS_OLA_LOW_TH,
    HS_OLA_HIGH_TH,
} en_HS_OLA_t;

/* *** Heater Driver Control *** */
typedef enum
{
    HEATER_CNFG_DISABLED,
    HEATER_CNFG_SPI_CONTROL,
    HEATER_CNFG_PWM_PIN_CONTROL,
} en_HEATER_CNFG_t;

/* *** Electrochromic Driver Control *** */
typedef enum
{
    EC_ECFB_LS_DISABLE,
    EC_ECFB_LS_ENABLE,
} en_EC_ECFB_LS_EN_t;

typedef enum
{
    EC_OL_CURR_SRC_DIS, /* EC Open-load current source disabled */
    EC_OL_CURR_SRC_EN,  /* EC Open-load current source enabled */
} en_EC_ECDRV_OL_EN_t;

typedef enum
{
    EC_ECFB_UV_TH_100mV,
    EC_ECFB_UV_TH_200mV,
} en_EC_ECFB_UV_TH_t; /* Undervoltage (short to gnd) threshold */

typedef enum
{
    EC_ECFB_UV_OV_DG_20us,
    EC_ECFB_UV_OV_DG_50us,
    EC_ECFB_UV_OV_DG_100us,
    EC_ECFB_UV_OV_DG_200us,
} EC_ECFB_UV_OV_DG_t;

typedef enum
{
    EC_ECFB_UV_OV_NO_ACTION,
    EC_ECFB_UV_OV_REPORT_ONLY,
    EC_ECFB_UV_OV_REPORT_AND_DISABLE,
} EC_ECFB_UV_OV_MODE_t;

typedef enum
{
    EC_FLT_HiZ_EC,
    EC_FLT_RETRY_OUT7_ITRIP,
} EC_FLT_MODE_t;

typedef enum
{
    EC_ECFB_LS_NO_PWM_DISCHARGE, /* Fast discharge */
    EC_ECFB_LS_PWM_DISCHARGE,
} EC_ECFB_LS_PWM_t;

typedef enum
{
    EC_OLEN_DIS,
    EC_OLEN_EN,
} EC_OLEN_t;

typedef enum
{
    ECFB_MAX_1_2V,
    ECFB_MAX_1_5V
} EC_ECFB_MAX_t;

/* *** Half-bridge Control *** */
typedef enum
{
    HHB_OUT_1,
    HHB_OUT_2,
    HHB_OUT_3,
    HHB_OUT_4,
    HHB_OUT_5,
    HHB_OUT_6,
} en_HHB_OUTx_t;

typedef enum
{
    HHB_DISABLED,
    HHB_SPI_EN,
    HHB_PWM1_COMP,
    HHB_PWM1_LS,
    HHB_PWM1_HS,
    HHB_PWM2_COMP,
    HHB_PWM2_LS,
    HHB_PWM2_HS,
    HHB_CNFG_REG_NOT_CHANGE = DRV8000_REG_NOT_CHANGE,
} en_HHB_CNFG_t;

typedef enum
{
    HHB_OFF,
    HHB_HS_ON,
    HHB_LS_ON,
    HHB_EN_REG_NOT_CHANGE = DRV8000_REG_NOT_CHANGE,
} en_HHB_EN_t;

typedef enum
{
    HHB_PASSIVE_FW,
    HHB_ACTIVE_FW,
} en_HHB_FW_t;

typedef enum
{
    HHB_OCP_DG_6us,
    HHB_OCP_DG_10us,
    HHB_OCP_DG_20us,
    HHB_OCP_DG_60us,
} en_HHB_OCP_DG_t;

typedef enum
{
    HHB_OLP_DISABLED,
    HHB_OUTX_PU_OUTY_PD_OUTY_VREF_LOW,  /* OUT X Pull-up enabled, OUT Y pull-down enabled, OUT Y selected, VREF Low */
    HHB_OUTX_PU_OUTY_PD_OUTX_VREF_HIGH, /* OUT X Pull-up enabled, OUT Y pull-down enabled, OUT X selected, VREF High */
    HHB_OUTX_PD_OUTY_PU_OUTY_VREF_LOW,  /* OUT X Pull-down enabled, OUT Y pull-up enabled, OUT Y selected, VREF Low */
} en_HHB_OLP_CNFG_t;

typedef enum
{
    HHB_OLP_SEL_DISABLED,
    HHB_OLP_OUT_1_2,
    HHB_OLP_OUT_1_3,
    HHB_OLP_OUT_1_4,
    HHB_OLP_OUT_1_5,
    HHB_OLP_OUT_1_6,
    HHB_OLP_OUT_2_3,
    HHB_OLP_OUT_2_4,
    HHB_OLP_OUT_2_5,
    HHB_OLP_OUT_2_6,
    HHB_OLP_OUT_3_4,
    HHB_OLP_OUT_3_5,
    HHB_OLP_OUT_3_6,
    HHB_OLP_OUT_4_5,
    HHB_OLP_OUT_4_6,
    HHB_OLP_OUT_5_6,
} en_HHB_OLP_SEL_t;

typedef enum
{
    HHB_OLA_TH_32_cycles,   /* Active open load cycle count threshold */
    HHB_OLA_TH_128_cycles,
} en_HHB_OLA_TH_t;

typedef enum
{
    HHB_SR_1_6V_us,
    HHB_SR_10V_us,
    HHB_SR_20V_us,
} en_HHB_SR_t;

typedef enum
{
    HHB_ITRIP_DG_2us,
    HHB_ITRIP_DG_5us,
    HHB_ITRIP_DG_10us,
    HHB_ITRIP_DG_20us,
} en_HHB_ITRIP_DG_t;

typedef enum
{
    HHB_ITRIP_LVL_O12_0_7A   = 0u,
    HHB_ITRIP_LVL_O12_0_875A = 1u,
} en_HHB_ITRIP_LVL_O12_t;

typedef enum
{
    HHB_ITRIP_LVL_O3456_1_25A = 0u,
    HHB_ITRIP_LVL_O3456_2_5A  = 1u,
    HHB_ITRIP_LVL_O3456_3_5A  = 2u,
} en_HHB_ITRIP_LVL_O3456_t;

typedef enum
{
    HHB_ITRIP_FREQ_20kHz,
    HHB_ITRIP_FREQ_10kHz,
    HHB_ITRIP_FREQ_5kHz,
    HHB_ITRIP_FREQ_2_5kHz,
} en_HHB_ITRIP_FREQ_t;

/* *** H-bridge Control *** */
typedef enum
{
    GD_BRG_MODE_HALF_BRG, /* Half-bridge control mode */
    GD_BRG_MODE_PH_EN,    /* Full-bridge PH/EN control mode */
    GD_BRG_MODE_PWM,      /* Full-bridge PWM control mode */
} en_GD_BRG_MODE_t;

typedef enum
{
    GD_LOW_SIDE_ACTIVE_FW,
    GD_HIGH_SIDE_ACTIVE_FW,
} en_GD_FBRG_FW_MODE_t;

typedef enum
{
    GD_DRIVE_REVERSE,
    GD_DRIVE_FORWARD,
} en_GD_FBRG_DIRECTION_t;

typedef enum
{
    GD_INx_PIN,         // b00
    GD_INx_SPI,         // b01
} en_GD_INx_MODE_t;


/* **********************************************************************/
/* ***               Definition of global variables                   ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***            Declaration of global functions                     ***/
/* **********************************************************************/
/* *** PIN Control *** */
#ifdef GDU_GD_USED
uint8_t drv8000_pin_gd_enable(st_DRV8000_Interface_t* interface);

uint8_t drv8000_pin_gd_disable(st_DRV8000_Interface_t* interface);
#endif /* GDU_GD_USED */

uint8_t drv8000_pin_set_pwm1(st_DRV8000_Interface_t* interface,
                             uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                            );

#ifdef GDU_PWM2_USED
uint8_t drv8000_pin_set_pwm2(st_DRV8000_Interface_t* interface,
                             uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                            );
#endif /* GDU_PWM2_USED */

uint8_t drv8000_pin_set_gd_in1(st_DRV8000_Interface_t* interface,
                               uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                              ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                              );

#ifndef GDU_GD_IN2_GPIO
uint8_t drv8000_pin_set_gd_in2(st_DRV8000_Interface_t* interface,
                               uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                              ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                              );
#endif /* GDU_GD_IN2_GPIO */

uint8_t drv8000_wakeup(st_DRV8000_Interface_t* interface);

uint8_t drv8000_sleep_mode(st_DRV8000_Interface_t* interface);

uint8_t drv8000_reset(st_DRV8000_Interface_t* interface);

/* *** SPI Read *** */
uint8_t drv8000_read_status_registers(st_DRV8000_Interface_t* interface);

uint8_t drv8000_read_config_registers(st_DRV8000_Interface_t* interface);

uint8_t drv8000_read_control_registers(st_DRV8000_Interface_t* interface);

uint8_t drv8000_read_registers(st_DRV8000_Interface_t* interface);

uint8_t drv8000_read_devid(st_DRV8000_Interface_t* interface);

/* *** SPI Control *** */
uint8_t drv8000_clear_fault(st_DRV8000_Interface_t* interface);

uint8_t drv8000_cfg_reg_lock_unlock(st_DRV8000_Interface_t* interface,
                                    en_LOCK_UNLOCK_REG_WRITE_t reg_lock_unlock);

uint8_t drv8000_ctrl_reg_lock_unlock(st_DRV8000_Interface_t* interface,
                                     en_LOCK_UNLOCK_REG_WRITE_t reg_lock_unlock);

uint8_t drv8000_ipropi_mode(st_DRV8000_Interface_t* interface,
                            en_IPROPI_MODE_t ipropi_mode,
                            en_IPROPI_SEL_MUX_t ipropi_sel);

/* ** High Side Driver Control ** */
#ifdef GDU_HS_USED
uint8_t drv8000_hs_driver_cnfg(st_DRV8000_Interface_t* interface,
                               en_HS_CNFG_t hs_out7_cnfg,
                               en_HS_CNFG_t hs_out8_cnfg,
                               en_HS_CNFG_t hs_out9_cnfg,
                               en_HS_CNFG_t hs_out10_cnfg,
                               en_HS_CNFG_t hs_out11_cnfg,
                               en_HS_CNFG_t hs_out12_cnfg);

uint8_t drv8000_hs_driver_spi_enable(st_DRV8000_Interface_t* interface,
                                     en_HS_EN_t hs_out7_en,
                                     en_HS_EN_t hs_out8_en,
                                     en_HS_EN_t hs_out9_en,
                                     en_HS_EN_t hs_out10_en,
                                     en_HS_EN_t hs_out11_en,
                                     en_HS_EN_t hs_out12_en);

uint8_t drv8000_hs_set_gen_pwm_dutycycle(st_DRV8000_Interface_t* interface,
                                         en_HS_OUTx_t hs_outx,
                                         uint16_t dutycycle);

uint8_t drv8000_hs_set_gen_pwm_freq(st_DRV8000_Interface_t* interface,
                                    en_HS_GEN_PWM_FREQ_t hs_out7_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out8_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out9_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out10_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out11_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out12_pwm_freq);

/**
 * @brief Enables or disables constant current mode (CCM) for high-side driver outputs
 * 
 * Enables or disables a constant current for a short duration to the desired high-side outputs.
 * 
 * @param interface             Device control interface from microcontroller. (see `st_DRV8000_Interface_t` struct).
 * @param hs_out7_ccm_en        High-side OUT7 CCM enable bit (see `en_HS_CCM_EN_t` enum).
 * @param hs_out8_ccm_en        High-side OUT8 CCM enable bit (see `en_HS_CCM_EN_t` enum).
 * @param hs_out9_ccm_en        High-side OUT9 CCM enable bit (see `en_HS_CCM_EN_t` enum).
 * @param hs_out10_ccm_en       High-side OUT10 CCM enable bit (see `en_HS_CCM_EN_t` enum).
 * @param hs_out11_ccm_en       High-side OUT11 CCM enable bit (see `en_HS_CCM_EN_t` enum).
 * @param hs_out12_ccm_en       High-side OUT12 CCM enable bit (see `en_HS_CCM_EN_t` enum).
 * @return uint8_t              Status code (0 = Success, non-zero = Error).
 */
uint8_t drv8000_hs_enable_ccm(st_DRV8000_Interface_t* interface,
                              en_HS_CCM_EN_t hs_out7_ccm_en,
                              en_HS_CCM_EN_t hs_out8_ccm_en,
                              en_HS_CCM_EN_t hs_out9_ccm_en,
                              en_HS_CCM_EN_t hs_out10_ccm_en,
                              en_HS_CCM_EN_t hs_out11_ccm_en,
                              en_HS_CCM_EN_t hs_out12_ccm_en);

/* ** Heater Driver Control ** */
uint8_t drv8000_heater_driver_cnfg(st_DRV8000_Interface_t* interface,
                                   en_HEATER_CNFG_t hs_heater_cnfg);

uint8_t drv8000_heater_driver_enable(st_DRV8000_Interface_t* interface,
                                     en_HS_EN_t hs_heater_en);

/* ** Electrochromic Driver Control ** */
uint8_t drv8000_ec_driver_enable(st_DRV8000_Interface_t* interface,
                                 en_HS_EN_t hs_ec_en,
                                 en_EC_ECFB_LS_EN_t es_ecfb_ls_en,
                                 uint8_t ec_v_tar);
#endif /* GDU_HS_USED */

/* *** Half-bridge Control *** */
#ifdef GDU_HHB_USED
/**
 * @brief Configures the control mode of half-bridge outputs OUT1 to OUT4.
 * 
 * Enables or disables control of half-bridge OUT1 to OUT4, and sets control mode between PWM or SPI.
 * 
 * @param interface             Device control interface from microcontroller. (see `st_DRV8000_Interface_t` struct).
 * @param hhb_out1_cnfg         Half-bridge OUT1 configuration (see `en_HHB_CNFG_t` enum).
 * @param hhb_out2_cnfg         Half-bridge OUT2 configuration (see `en_HHB_CNFG_t` enum).
 * @param hhb_out3_cnfg         Half-bridge OUT3 configuration (see `en_HHB_CNFG_t` enum).
 * @param hhb_out4_cnfg         Half-bridge OUT4 configuration (see `en_HHB_CNFG_t` enum).
 * @return uint8_t              Status code (0 = Success, non-zero = Error).
 */
uint8_t drv8000_hhb_out1234_set_mode(st_DRV8000_Interface_t* interface,
                                     en_HHB_CNFG_t hhb_out1_cnfg,
                                     en_HHB_CNFG_t hhb_out2_cnfg,
                                     en_HHB_CNFG_t hhb_out3_cnfg,
                                     en_HHB_CNFG_t hhb_out4_cnfg);

/**
 * @brief Configures the control mode of half-bridge outputs OUT5 and OUT6.
 * 
 * Enables or disables control of half-bridge OUT5 and OUT6, and sets control mode between PWM or SPI.
 * 
 * @param interface             Device control interface from microcontroller. (see `st_DRV8000_Interface_t` struct).
 * @param hhb_out5_cnfg         Half-bridge OUT5 configuration (see `en_HHB_CNFG_t` enum).
 * @param hhb_out6_cnfg         Half-bridge OUT6 configuration (see `en_HHB_CNFG_t` enum).
 * @return uint8_t              Status code (0 = Success, non-zero = Error).
 */
uint8_t drv8000_hhb_out56_set_mode(st_DRV8000_Interface_t* interface,
                                   en_HHB_CNFG_t hhb_out5_cnfg,
                                   en_HHB_CNFG_t hhb_out6_cnfg);

uint8_t drv8000_hhb_spi_enable(st_DRV8000_Interface_t* interface,
                               en_HHB_EN_t hhb_out1_en,
                               en_HHB_EN_t hhb_out2_en,
                               en_HHB_EN_t hhb_out3_en,
                               en_HHB_EN_t hhb_out4_en,
                               en_HHB_EN_t hhb_out5_en,
                               en_HHB_EN_t hhb_out6_en);

/**
 * @brief Configures the freewheeling control mode of half-bridge outputs.
 * 
 * Sets active/passive freewheeling for half-bridges, active freewheeling will disable non-synchronous 
 * rectification during ITRIP regulation.
 * 
 * @param interface             Device control interface from microcontroller. (see `st_DRV8000_Interface_t` struct).
 * @param hhb_out1_fw           Half-bridge OUT1 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out2_fw           Half-bridge OUT2 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out3_fw           Half-bridge OUT3 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out4_fw           Half-bridge OUT4 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out5_fw           Half-bridge OUT5 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out6_fw           Half-bridge OUT6 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @return uint8_t              Status code (0 = Success, non-zero = Error).
 */
uint8_t drv8000_hhb_set_fw(st_DRV8000_Interface_t* interface,
                           en_HHB_FW_t hhb_out1_fw,
                           en_HHB_FW_t hhb_out2_fw,
                           en_HHB_FW_t hhb_out3_fw,
                           en_HHB_FW_t hhb_out4_fw,
                           en_HHB_FW_t hhb_out5_fw,
                           en_HHB_FW_t hhb_out6_fw);
#endif /* GDU_HHB_USED */

/* *** H-bridge Control *** */
#ifdef GDU_GD_USED
uint8_t drv8000_spi_gd_enable(st_DRV8000_Interface_t* interface);

uint8_t drv8000_spi_gd_disable(st_DRV8000_Interface_t* interface);

uint8_t drv8000_gd_hb_set_mode(st_DRV8000_Interface_t* interface,
                               en_GD_BRG_MODE_t brg_mode,
                               en_GD_INx_MODE_t in1_spi_mode,
                               en_GD_INx_MODE_t in2_spi_mode,
                               en_GD_FBRG_FW_MODE_t fw_mode);

uint8_t drv8000_gd_hb_set_direction(st_DRV8000_Interface_t* interface,
                                    en_GD_FBRG_DIRECTION_t direction);
#endif /* GDU_GD_USED */


#endif /* GDU_TI_DRV8000_DRV8000_H_ */
