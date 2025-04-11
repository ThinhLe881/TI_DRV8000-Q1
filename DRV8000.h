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
#define DRV8000_GLOBAL_STATUS_MASK								0xFFu       /* first 2 bits (always 1) + 6-bit fault status */
#define DRV8000_DEVICE_ID_MASK                                  0xFFu
#define DRV8000_GEN_PWM_DC_MASK                                 0x3FFu      /* 10 bits */
#define DRV8000_EC_V_TAR_MASK                                   0x3Fu       /* 6 bits */

#define DRV8000_WRITE_ACCESS                                    0u
#define DRV8000_READ_ACCESS                                     1u
#define DRV8000_SPI_FRAME_LEN                                   4u          /* 24 bits + 8 bits padding = 4 8-bit buffers */
#define DRV8000_SUCCESS_SPI_STATUS                              0xC0u       /* 1 1 0     0    0     0   0    0       */
                                                                            /* 1 1 FAULT WARN OV_UV DRV OTSD SPI_ERR */
#define DRV8000_REG_NOT_CHANGE                                  0x1FFFF     /* Flag for unchange bits, only for certain bitfields */
#define PIN_LOW                                                 0u
#define PIN_HIGH                                                1u


/* **********************************************************************/
/* ***               Definition of global types                       ***/
/* **********************************************************************/
typedef struct
{
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
    void* gd_in2_port;
    uint8_t gd_in2_pin;
#else /* not defined GDU_GD_IN2_GPIO */
    uint8_t pwm_gd_in2_instance;
    uint8_t pwm_gd_in2_channel;
#endif /* GDU_GD_IN2_GPIO */
    void* drvoff_port;
    uint8_t drvoff_pin;
#endif /* GDU_GD_USED */
    void* nsleep_port;
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

typedef enum
{
    STST_SUCCESS,
    
    STST_SPI_ERR,
    STST_GIO_ERR,
    STST_PWM_ERR,
    
    STST_GDU_FAULT,
    STST_GDU_WARN,
    STST_GDU_UNKN_ERR,
    STST_CTRL_REG_LOCKED,
    STST_CNFG_REG_LOCKED,
    
    STST_WRONG_MODE,
    STST_NULL_INTERFACE_PTR,
} en_DRV8000_STST_t;

typedef enum
{
    GATE_DRV_ENABLE,        /* Enable gate driver - gate driver half-bridges are pulled up to PVDD */
    GATE_DRV_DISABLE,       /* Disable gate driver - pull down gate driver half-bridges to Hi-Z */
} en_DRVOFF_t;

typedef enum
{
    SLEEP_MODE,
    OPERATION_MODE,
} en_nSLEEP_t;

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

typedef enum
{
    OTSD_GLOBAL_SHUTDOWN,
    OTSD_AFFECTED_DRV_SHUTDOWN_ONLY,
} en_OTSD_MODE_t;

typedef enum
{
    CHARGE_PUMP_ENABLE,
    CHARGE_PUMP_DISABLE,
} en_DIS_CP_t;

typedef enum
{
    PVDD_OV_MODE_LATCHED_FLT,
    PVDD_OV_MODE_AUT_RECOV,
    PVDD_OV_MODE_WARN_ONLY,
    PVDD_OV_MODE_DISABLE,
} en_PVDD_OV_MODE_t;

typedef enum
{
    PVDD_OV_DG_1us,
    PVDD_OV_DG_2us,
    PVDD_OV_DG_4us,
    PVDD_OV_DG_8us,
} en_PVDD_OV_DG_t;

typedef enum
{
    PVDD_OV_LVL_21_5V,
    PVDD_OV_LVL_28_5V,
} en_PVDD_OV_LVL_t;

typedef enum
{
    VCP_UV_LVL_4_75V,
    VCP_UV_LVL_6_25V,
} en_VCP_UV_LVL_t;

typedef enum
{
    CP_MODE_AUT_SWITCH,
    CP_MODE_DOUBLER,
    CP_MODE_TRIPLER,
} en_CP_MODE_t;

typedef enum
{
    VCP_UV_MODE_LATCHED_FLT,
    VCP_UV_MODE_AUT_RECOV,
} en_VCP_UV_MODE_t;

typedef enum
{
    PVDD_UV_MODE_LATCHED_FLT,
    PVDD_UV_MODE_AUT_RECOV,
} en_PVDD_UV_MODE_t;

typedef enum
{
    WD_FLT_WARN_DRIVER_ON,
    WD_FLT_FAULT_DRIVER_OFF,
} en_WD_FLT_M_t;

typedef enum
{
    WD_WIN_4_to_40ms,
    WD_WIN_10_to_100ms,
} en_WD_WIN_t;

typedef enum
{
    WD_DISABLE,
    WD_ENABLE,
} en_WD_EN_t;

typedef enum
{
    EN_SSC_DISABLE,
    EN_SSC_ENABLE,
} en_EN_SSC_t;

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
    HS_CCM_DISABLE,
    HS_CCM_REG_NOT_CHANGE = DRV8000_REG_NOT_CHANGE,
} en_HS_CCM_t;

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
    HS_OUT7_ITRIP_BLK_0us  = 1u,
    HS_OUT7_ITRIP_BLK_20us = 2u,
    HS_OUT7_ITRIP_BLK_40us = 3u,
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
    HEATER_CNFG_REG_NOT_CHANGE = DRV8000_REG_NOT_CHANGE,
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

    HHB_ITRIP_LVL_O3_1_25A   = 0u,
    HHB_ITRIP_LVL_O3_2_5A    = 1u,
    HHB_ITRIP_LVL_O3_3_5A    = 2u,

    HHB_ITRIP_LVL_O4_1_25A   = 0u,
    HHB_ITRIP_LVL_O4_2_75A   = 1u,
    HHB_ITRIP_LVL_O4_3_5A    = 2u,

    HHB_ITRIP_LVL_O5_2_75A   = 0u,
    HHB_ITRIP_LVL_O5_6_5A    = 1u,
    HHB_ITRIP_LVL_O5_7_5A    = 2u,

    HHB_ITRIP_LVL_O6_2_25A   = 0u,
    HHB_ITRIP_LVL_O6_5_5A    = 1u,
    HHB_ITRIP_LVL_O6_6_25A   = 2u,

    HHB_ITRIP_DISABLE        = 3u,
} en_HHB_ITRIP_t;

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
    GD_INx_PIN,
    GD_INx_SPI,
} en_GD_INx_MODE_t;

typedef enum
{
    GD_EN_DISABLE,      /* Gate driver inputs are ignored and the gate driver passive pull-downs are enabled */
                        /* Unlike DRVOFF, changing EN_GD does not pull down the gate driver half-bridges to Hi-Z */
    GD_EN_ENABLE,       /* Gate driver outputs are enabled and controlled by the digital inputs */
} en_GD_EN_t;


/* **********************************************************************/
/* ***               Definition of global variables                   ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***            Declaration of global functions                     ***/
/* **********************************************************************/
/**
 * @brief Transmits and receives data over SPI for the DRV8000 driver.
 * 
 * This function performs an SPI transaction by sending data from `tx_buffer` and receiving data into `rx_buffer`.
 * 
 * @note The user MUST implement this function to interface with the SPI hardware.
 * @note The implementation of this function must ensure SPI timing requirements of the DRV8000.
 *
 * @param tx_buffer             Pointer to the buffer containing data to be transmitted.
 * @param rx_buffer             Pointer to the buffer where received data will be stored.
 * @param len                   Number of bytes to transmit and receive.
 * @return uint8_t              SPI status code (0 = Success, non-zero = Error).
 * 
 * @warning The buffers must be valid, and `len` should not exceed the maximum SPI transaction size.
 */
extern uint8_t drv8000_spi_transceive(const uint8_t* tx_buffer,
                                        uint8_t* rx_buffer,
                                        const uint16_t len);

/**
 * @brief Set dutycycle for DRV8000 PWM channels
 * 
 * @note The user MUST implement this function to interface with the PWM hardware.
 * 
 * @param instance              PWM instance
 * @param channel               PWM channel
 * @param duty_cycle            PWM dutycycle
 * @return uint8_t              Status code (0 = Success, non-zero = Error).
 */
extern uint8_t drv8000_pwm_set_dc(uint8_t instance,
                                    uint8_t channel,
                                    uint16_t duty_cycle);

#ifndef GDU_PWM_PERIOD_FIXED
/**
 * @brief Set period for DRV8000 PWM channels
 * 
 * @note The user MUST implement this function to interface with the PWM hardware.
 * 
 * @param instance              PWM instance
 * @param channel               PWM channel
 * @param period                PWM period
 * @return uint8_t              Status code (0 = Success, non-zero = Error).
 */
extern uint8_t drv8000_pwm_set_period(uint8_t instance,
                                        uint8_t channel,
                                        uint16_t period);
#endif /* GDU_PWM_PERIOD_FIXED */

/**
 * @brief Control DRV8000 GPIO pins
 * 
 * @note The user MUST implement this function to interface with the GPIO hardware.
 * 
 * @param port                  GPIO port
 * @param pin                   GPIO pin number
 * @param pin_level             GPIO pin level (1 = High, 0 = Low)
 * @return uint8_t              Status code (0 = Success, non-zero = Error).
 */
extern uint8_t drv8000_gpio(void* port,
                            uint8_t pin,
                            uint8_t pin_level);

/**
 * @brief Blocking delay
 * 
 * @note The user MUST implement this function to interface with the hardware.
 * 
 * @param delay_us              Delay period in microseconds
 */
extern void drv8000_delay(uint16_t delay_us);

/**
 * @brief Set the device interface
 * 
 * @param interface             Device control interface from microcontroller. (see `st_DRV8000_Interface_t` struct).
 */
void drv8000_interface_set(st_DRV8000_Interface_t* interface);

/**
 * @brief Get the device SPI IC status
 * 
 * @return un_DRV8000_SPI_STST_t    Device IC status from SPI received frame (see `st_SpiCommStatus` struct).
 */
un_DRV8000_SPI_STST_t drv8000_spi_status_get(void);

/* *** PIN Control *** */
#ifdef GDU_GD_USED
en_DRV8000_STST_t drv8000_pin_gd_enable_disable(en_DRVOFF_t drvoff_en);
#endif /* GDU_GD_USED */

en_DRV8000_STST_t drv8000_pin_set_pwm1(uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                    ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                    );

#ifdef GDU_PWM2_USED
en_DRV8000_STST_t drv8000_pin_set_pwm2(uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                    ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                );
#endif /* GDU_PWM2_USED */

en_DRV8000_STST_t drv8000_pin_set_gd_in1(uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                        ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                    );

#ifndef GDU_GD_IN2_GPIO
en_DRV8000_STST_t drv8000_pin_set_gd_in2(uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                        ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                    );
#endif /* GDU_GD_IN2_GPIO */

en_DRV8000_STST_t drv8000_sleep_wake(en_nSLEEP_t nsleep_en);

en_DRV8000_STST_t drv8000_reset(void);

/* *** SPI Read *** */
en_DRV8000_STST_t drv8000_read_ic_stat1_reg(un_DRV8000_Reg_t* reg_data);

en_DRV8000_STST_t drv8000_read_ic_stat2_reg(un_DRV8000_Reg_t* reg_data);

en_DRV8000_STST_t drv8000_read_hs_stat_reg(un_DRV8000_Reg_t* reg_data);

en_DRV8000_STST_t drv8000_read_ec_heat_itrip_stat_reg(un_DRV8000_Reg_t* reg_data);

en_DRV8000_STST_t drv8000_read_hb_stat1_reg(un_DRV8000_Reg_t* reg_data);

en_DRV8000_STST_t drv8000_read_hb_stat2_reg(un_DRV8000_Reg_t* reg_data);

en_DRV8000_STST_t drv8000_read_gd_stat_reg(un_DRV8000_Reg_t* reg_data);

uint8_t drv8000_read_devid(void);

/* *** SPI Control *** */
/**
 * @brief Clear all fault flags from DRV8000
 * 
 * @return en_DRV8000_STST_t    Status code.
 */
en_DRV8000_STST_t drv8000_clear_fault(void);

en_DRV8000_STST_t drv8000_cfg_reg_lock_unlock(en_LOCK_UNLOCK_REG_WRITE_t reg_lock_unlock);

en_DRV8000_STST_t drv8000_ctrl_reg_lock_unlock(en_LOCK_UNLOCK_REG_WRITE_t reg_lock_unlock);

en_DRV8000_STST_t drv8000_ipropi_mode(en_IPROPI_MODE_t ipropi_mode,
                                        en_IPROPI_SEL_MUX_t ipropi_sel);

en_DRV8000_STST_t drv8000_set_ic_cnfg1(en_OTSD_MODE_t otsd_mode, 
                                        en_DIS_CP_t dis_cp, 
                                        en_PVDD_OV_MODE_t pvdd_ov_mode, 
                                        en_PVDD_OV_DG_t pvdd_ov_dg, 
                                        en_PVDD_OV_LVL_t pvdd_ov_lvl, 
                                        en_VCP_UV_LVL_t vcp_uv_lvl, 
                                        en_CP_MODE_t cp_mode, 
                                        en_VCP_UV_MODE_t vcp_uv_mode, 
                                        en_PVDD_UV_MODE_t pvdd_uv_mode, 
                                        en_WD_FLT_M_t wd_flt_mode, 
                                        en_WD_WIN_t wd_window, 
                                        en_EN_SSC_t en_ssc);

en_DRV8000_STST_t drv8000_wd_enable_disable(en_WD_EN_t wd_en);

/**
 * @brief Watchdog trigger
 * 
 * Trigger watchdog bit to restart the watchdog timer.
 * Clear watchdog fault if occurred and restart timer.
 * 
 * @return en_DRV8000_STST_t    Status code.
 */
en_DRV8000_STST_t drv8000_wd_trig(void);

/**
 * @brief Clear watchdog timer fault and restart watchdog timer
 * 
 * @return en_DRV8000_STST_t    Status code.
 */
en_DRV8000_STST_t drv8000_wd_fault_clear(void);

/* ** High Side Driver and Heater Control ** */
#ifdef GDU_HS_USED
en_DRV8000_STST_t drv8000_hs_driver_cnfg(en_HS_CNFG_t hs_out7_cnfg, 
                                            en_HS_CNFG_t hs_out8_cnfg, 
                                            en_HS_CNFG_t hs_out9_cnfg, 
                                            en_HS_CNFG_t hs_out10_cnfg, 
                                            en_HS_CNFG_t hs_out11_cnfg, 
                                            en_HS_CNFG_t hs_out12_cnfg,
                                            en_HEATER_CNFG_t hs_heater_cnfg);

en_DRV8000_STST_t drv8000_hs_driver_spi_enable(en_HS_EN_t hs_out7_en,
                                                en_HS_EN_t hs_out8_en,
                                                en_HS_EN_t hs_out9_en,
                                                en_HS_EN_t hs_out10_en,
                                                en_HS_EN_t hs_out11_en,
                                                en_HS_EN_t hs_out12_en,
                                                en_HS_EN_t hs_heater_en);

en_DRV8000_STST_t drv8000_hs_set_gen_pwm_dutycycle(en_HS_OUTx_t hs_outx,
                                                    uint16_t dutycycle);

en_DRV8000_STST_t drv8000_hs_set_gen_pwm_freq(en_HS_GEN_PWM_FREQ_t hs_out7_pwm_freq,
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
 * @note Short circuit and over current detection are disabled during constant current mode
 * @note CCM has to be enabled before enabling HS driver
 * 
 * @param hs_out7_ccm_en        High-side OUT7 CCM enable bit (see `en_HS_CCM_t` enum).
 * @param hs_out8_ccm_en        High-side OUT8 CCM enable bit (see `en_HS_CCM_t` enum).
 * @param hs_out9_ccm_en        High-side OUT9 CCM enable bit (see `en_HS_CCM_t` enum).
 * @param hs_out10_ccm_en       High-side OUT10 CCM enable bit (see `en_HS_CCM_t` enum).
 * @param hs_out11_ccm_en       High-side OUT11 CCM enable bit (see `en_HS_CCM_t` enum).
 * @param hs_out12_ccm_en       High-side OUT12 CCM enable bit (see `en_HS_CCM_t` enum).
 * @return en_DRV8000_STST_t    Status code.
 */
en_DRV8000_STST_t drv8000_hs_enable_ccm(en_HS_CCM_t hs_out7_ccm_en,
                                        en_HS_CCM_t hs_out8_ccm_en,
                                        en_HS_CCM_t hs_out9_ccm_en,
                                        en_HS_CCM_t hs_out10_ccm_en,
                                        en_HS_CCM_t hs_out11_ccm_en,
                                        en_HS_CCM_t hs_out12_ccm_en);

/* ** Electrochromic Driver Control ** */
en_DRV8000_STST_t drv8000_ec_driver_enable(en_HS_EN_t hs_ec_en,
                                            en_EC_ECFB_LS_EN_t es_ecfb_ls_en,
                                            uint8_t ec_v_tar);

en_DRV8000_STST_t drv8000_ec_driver_cnfg(EC_ECFB_MAX_t ecfb_max_v,
                                            EC_OLEN_t ec_olen,
                                            EC_ECFB_LS_PWM_t ecfb_ls_pwm,
                                            EC_FLT_MODE_t ec_flt_mode,
                                            EC_ECFB_UV_OV_MODE_t ecfb_ov_mode,
                                            EC_ECFB_UV_OV_MODE_t ecfb_uv_mode,
                                            EC_ECFB_UV_OV_DG_t ecfb_ov_dg,
                                            EC_ECFB_UV_OV_DG_t ecfb_uv_dg,
                                            en_EC_ECFB_UV_TH_t ecfb_uv_th,
                                            en_EC_ECDRV_OL_EN_t ecdrv_ol_en);
#endif /* GDU_HS_USED */

/* *** Half-bridge Control *** */
#ifdef GDU_HHB_USED
/**
 * @brief Configures the control mode of half-bridge outputs OUT1 to OUT4.
 * 
 * Enables or disables control of half-bridge OUT1 to OUT4, and sets control mode between PWM or SPI.
 * 
 * @param hhb_out1_cnfg         Half-bridge OUT1 configuration (see `en_HHB_CNFG_t` enum).
 * @param hhb_out2_cnfg         Half-bridge OUT2 configuration (see `en_HHB_CNFG_t` enum).
 * @param hhb_out3_cnfg         Half-bridge OUT3 configuration (see `en_HHB_CNFG_t` enum).
 * @param hhb_out4_cnfg         Half-bridge OUT4 configuration (see `en_HHB_CNFG_t` enum).
 * @return en_DRV8000_STST_t    Status code.
 */
en_DRV8000_STST_t drv8000_hhb_out1234_set_mode(en_HHB_CNFG_t hhb_out1_cnfg,
                                                en_HHB_CNFG_t hhb_out2_cnfg,
                                                en_HHB_CNFG_t hhb_out3_cnfg,
                                                en_HHB_CNFG_t hhb_out4_cnfg);

/**
 * @brief Configures the control mode of half-bridge outputs OUT5 and OUT6.
 * 
 * Enables or disables control of half-bridge OUT5 and OUT6, and sets control mode between PWM or SPI.
 * 
 * @param hhb_out5_cnfg         Half-bridge OUT5 configuration (see `en_HHB_CNFG_t` enum).
 * @param hhb_out6_cnfg         Half-bridge OUT6 configuration (see `en_HHB_CNFG_t` enum).
 * @return en_DRV8000_STST_t    Status code.
 */
en_DRV8000_STST_t drv8000_hhb_out56_set_mode(en_HHB_CNFG_t hhb_out5_cnfg,
                                                en_HHB_CNFG_t hhb_out6_cnfg);

en_DRV8000_STST_t drv8000_hhb_spi_enable(en_HHB_EN_t hhb_out1_en,
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
 * @param hhb_out1_fw           Half-bridge OUT1 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out2_fw           Half-bridge OUT2 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out3_fw           Half-bridge OUT3 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out4_fw           Half-bridge OUT4 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out5_fw           Half-bridge OUT5 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @param hhb_out6_fw           Half-bridge OUT6 freewheeling configuration (see `en_HHB_FW_t` enum).
 * @return en_DRV8000_STST_t    Status code.
 */
en_DRV8000_STST_t drv8000_hhb_set_fw(en_HHB_FW_t hhb_out1_fw, 
                                        en_HHB_FW_t hhb_out2_fw, 
                                        en_HHB_FW_t hhb_out3_fw, 
                                        en_HHB_FW_t hhb_out4_fw, 
                                        en_HHB_FW_t hhb_out5_fw, 
                                        en_HHB_FW_t hhb_out6_fw);

en_DRV8000_STST_t drv8000_hhb_set_ocp_dg(en_HHB_OCP_DG_t hhb_out1_ocp_dg, 
                                            en_HHB_OCP_DG_t hhb_out2_ocp_dg, 
                                            en_HHB_OCP_DG_t hhb_out3_ocp_dg, 
                                            en_HHB_OCP_DG_t hhb_out4_ocp_dg, 
                                            en_HHB_OCP_DG_t hhb_out5_ocp_dg, 
                                            en_HHB_OCP_DG_t hhb_out6_ocp_dg);

en_DRV8000_STST_t drv8000_hhb_set_itrip_lvl(en_HHB_ITRIP_t hhb_out1_itrip_lvl, 
                                            en_HHB_ITRIP_t hhb_out2_itrip_lvl, 
                                            en_HHB_ITRIP_t hhb_out3_itrip_lvl, 
                                            en_HHB_ITRIP_t hhb_out4_itrip_lvl, 
                                            en_HHB_ITRIP_t hhb_out5_itrip_lvl, 
                                            en_HHB_ITRIP_t hhb_out6_itrip_lvl);
#endif /* GDU_HHB_USED */

/* *** H-bridge Control *** */
#ifdef GDU_GD_USED
en_DRV8000_STST_t drv8000_spi_gd_enable_disable(en_GD_EN_t en_gd);

en_DRV8000_STST_t drv8000_gd_hb_set_mode(en_GD_BRG_MODE_t brg_mode, 
                                            en_GD_INx_MODE_t in1_spi_mode, 
                                            en_GD_INx_MODE_t in2_spi_mode, 
                                            en_GD_FBRG_FW_MODE_t fw_mode);

uinten_CTRL_STST_t8_t drv8000_gd_hb_set_direction(en_GD_FBRG_DIRECTION_t direction);
#endif /* GDU_GD_USED */

en_DRV8000_STST_t drv8000_init(void);

#endif /* GDU_DRV8000_H_ */