/*
 * DRV8000.c
 *
 *  Created on: Mar 3, 2025
 *      Author: thinh
 */

/* **********************************************************************/
/* ***              System and library files included                 ***/
/* **********************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "DRV8000.h"
#include "DRV8000_Reg.h"


/* **********************************************************************/
/* ***            Definition of local plain CONSTants                 ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***               Definition of local types                        ***/
/* **********************************************************************/
typedef enum {
    /* STATUS registers */
    
    /* CONFIG registers */
    REGID_GD_CNFG                      ,
    REGID_HB_OUT_CNFG1                 ,
    REGID_HB_OUT_CNFG2                 ,
    REGID_HS_HEAT_OUT_CNFG             ,
    REGID_HS_PWM_FREQ_CNFG             ,
    /* CONTROL registers */
    REGID_IC_CTRL                      ,
    REGID_GD_HB_CTRL                   ,
    REGID_HS_EC_HEAT_CTRL              ,

    DRV8000_NUM_OF_REGS		           ,
} en_REG_ID_t;


/* **********************************************************************/
/* ***              Definition of local variables                     ***/
/* **********************************************************************/
static st_DRV8000_Interface_t* drv8000_interface = NULL;
/* To avoid RMW for registers that require frequent bit-level modification */
static un_DRV8000_Reg_t drv8000_reg_map[DRV8000_NUM_OF_REGS];
static un_DRV8000_SPI_STST_t drv8000_global_spi_status;


/* **********************************************************************/
/* ***             Declaration of local functions                     ***/
/* **********************************************************************/
static en_DRV8000_STST_t drv8000_spi_status_check(void);

static en_DRV8000_STST_t drv8000_spi_read(const uint8_t reg_addr,
                                            un_DRV8000_Reg_t* reg_val);

static en_DRV8000_STST_t drv8000_spi_write(const uint8_t reg_addr,
                                            const un_DRV8000_Reg_t reg_val);

static en_DRV8000_STST_t drv8000_gpio_set(void* port,
                                            uint8_t pin,
                                            uint8_t pin_level);

static en_DRV8000_STST_t drv8000_pwm_set(uint8_t instance,
                                            uint8_t channel,
                                            uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                            ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                        );


/* **********************************************************************/
/* ***              Definition of global variables                    ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***            Definition of global functions                      ***/
/* **********************************************************************/
/* *** PIN Control *** */
#ifdef GDU_GD_USED
en_DRV8000_STST_t drv8000_pin_gd_enable_disable(en_DRVOFF_t drvoff_en)
{
    /* De-assert DRVOFF does not enable gate driver, 
        use drv8000_enable_gate_driver() to flip gate driver enable bit */
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data;

    if (NULL != drv8000_interface)
    {
        ret = STST_NULL_INTERFACE_PTR;
    } else
    {
        ret = drv8000_gpio_set(drv8000_interface->drvoff_port,
                                drv8000_interface->drvoff_pin,
                                drvoff_en);
    }
    if (STST_SUCCESS == ret)
    {
        drv8000_delay(3000u); /* Wait ~3ms to register a valid DRVOFF command */

        if (GATE_DRV_ENABLE == drvoff_en)
        {
            ret = drv8000_clear_fault(); /* Clear DRVOFF_STAT flag */
        }
        if (STST_SUCCESS == ret)
        {
            ret = drv8000_spi_read(DRV8000_ADDREG_GD_STAT,
                                    &reg_data);

            if ((STST_SUCCESS == ret) && 
                (drvoff_en != reg_data.Reg_GD_STAT.DRVOFF_STAT))
            {
                ret = STST_GDU_UNKN_ERR;
            }
        }
    }
    
    return ret;
}

en_DRV8000_STST_t drv8000_pin_set_gd_in1(uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                        ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                        )
{
    return drv8000_pwm_set(drv8000_interface->pwm_gd_in1_instance,
                            drv8000_interface->pwm_gd_in1_channel,
                            dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,period
#endif /* GDU_PWM_PERIOD_FIXED */
                        );
}

#ifndef GDU_GD_IN2_GPIO
en_DRV8000_STST_t drv8000_pin_set_gd_in2(uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                        ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                    )
{
    return drv8000_pwm_set(drv8000_interface->pwm_gd_in2_instance,
                            drv8000_interface->pwm_gd_in2_channel,
                            dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,period
#endif /* GDU_PWM_PERIOD_FIXED */
                        );
}
#endif /* GDU_GD_IN2_GPIO */
#endif /* GDU_GD_USED */

en_DRV8000_STST_t drv8000_pin_set_pwm1(uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                        ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                    )
{
    return drv8000_pwm_set(drv8000_interface->pwm1_instance,
                            drv8000_interface->pwm1_channel,
                            dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,period
#endif /* GDU_PWM_PERIOD_FIXED */
                        );
}

#ifdef GDU_PWM2_USED
en_DRV8000_STST_t drv8000_pin_set_pwm2(uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                        ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                    )
{
    return drv8000_pwm_set(drv8000_interface->pwm2_instance,
                            drv8000_interface->pwm2_channel,
                            dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,period
#endif /* GDU_PWM_PERIOD_FIXED */
                        );
}
#endif /* GDU_PWM2_USED */

en_DRV8000_STST_t drv8000_sleep_wake(en_nSLEEP_t nsleep_en)
{
    if (NULL != drv8000_interface)
    {
        return STST_NULL_INTERFACE_PTR;
    } else
    {
        return drv8000_gpio_set(drv8000_interface->nsleep_port,
                                drv8000_interface->nsleep_pin,
                                nsleep_en);
    }

}

en_DRV8000_STST_t drv8000_reset(void)
{
    en_DRV8000_STST_t ret;

    /* Sleep DRV8000 */
    ret = drv8000_gpio_set(drv8000_interface->nsleep_port,
                            drv8000_interface->nsleep_pin,
                            PIN_LOW);
    
    if (STST_SUCCESS == ret)
    {
        /* (Optional) wait ~2us for DRV8000 to completely disabled, otherwise register values won't reset */
        drv8000_delay(2u);
        /* Wakeup DRV8000 */
        ret = drv8000_gpio_set(drv8000_interface->nsleep_port,
                                drv8000_interface->nsleep_pin,
                                PIN_HIGH);
        
        if (STST_SUCCESS == ret)
        {
            /* (Optional) wait ~6us for DRV8000 to completely wakeup, otherwise SPI comm won't work */
            drv8000_delay(6u);
        }
    }
    
    return ret;
}

/* *** SPI Read *** */
en_DRV8000_STST_t drv8000_read_ic_stat1_reg(un_DRV8000_Reg_t* reg_data)
{
    return drv8000_spi_read(DRV8000_ADDREG_IC_STAT1, 
                            reg_data);
}

en_DRV8000_STST_t drv8000_read_ic_stat2_reg(un_DRV8000_Reg_t* reg_data)
{
    return drv8000_spi_read(DRV8000_ADDREG_IC_STAT2, 
                            reg_data);
}

en_DRV8000_STST_t drv8000_read_hs_stat_reg(un_DRV8000_Reg_t* reg_data)
{
    return drv8000_spi_read(DRV8000_ADDREG_HS_STAT, 
                            reg_data);
}

en_DRV8000_STST_t drv8000_read_ec_heat_itrip_stat_reg(un_DRV8000_Reg_t* reg_data)
{
    return drv8000_spi_read(DRV8000_ADDREG_EC_HEAT_ITRIP_STAT, 
                            reg_data);
}

en_DRV8000_STST_t drv8000_read_hb_stat1_reg(un_DRV8000_Reg_t* reg_data)
{
    return drv8000_spi_read(DRV8000_ADDREG_HB_STAT1, 
                            reg_data);
}

en_DRV8000_STST_t drv8000_read_hb_stat2_reg(un_DRV8000_Reg_t* reg_data)
{
    return drv8000_spi_read(DRV8000_ADDREG_HB_STAT2, 
                            reg_data);
}

en_DRV8000_STST_t drv8000_read_gd_stat_reg(un_DRV8000_Reg_t* reg_data)
{
    return drv8000_spi_read(DRV8000_ADDREG_GD_STAT, 
                            reg_data);
}

uint8_t drv8000_read_devid(void)
{
    un_DRV8000_Reg_t reg_val;
    
    (void)drv8000_spi_read(DRV8000_ADDREG_DEVICE_ID,
                            &reg_val);
    
    return (uint8_t)(reg_val.u16_RegWord & DRV8000_DEVICE_ID_MASK);
}

/* *** SPI Control *** */
/**
 * @brief Clear all fault flags from DRV8000
 * 
 * @return en_DRV8000_STST_t    Status code.
 */
en_DRV8000_STST_t drv8000_clear_fault(void)
{
    un_DRV8000_Reg_t reg_data;

    reg_data.u16_RegWord = drv8000_reg_map[REGID_IC_CTRL].u16_RegWord;
    reg_data.Reg_IC_CTRL.CLR_FLT = 1u;

    (void)drv8000_spi_write(DRV8000_ADDREG_IC_CTRL,
                            reg_data);
    
    /* Return status code after clear fault bits and sync watchdog bit */
    return drv8000_spi_read(DRV8000_ADDREG_IC_CTRL,
                            &drv8000_reg_map[REGID_IC_CTRL]);
}

en_DRV8000_STST_t drv8000_reg_lock_unlock(en_LOCK_UNLOCK_REG_WRITE_t cnfg_reg_lck_unlck,
                                            en_LOCK_UNLOCK_REG_WRITE_t ctrl_reg_lck_unlck)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data;

    reg_data = drv8000_reg_map[REGID_IC_CTRL];
    reg_data.Reg_IC_CTRL.CNFG_LOCK = cnfg_reg_lck_unlck;
    reg_data.Reg_IC_CTRL.CTRL_LOCK = ctrl_reg_lck_unlck;

    ret = drv8000_spi_write(DRV8000_ADDREG_IC_CTRL,
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_IC_CTRL] = reg_data;
    }

    return ret;
}

en_DRV8000_STST_t drv8000_ipropi_mode(en_IPROPI_MODE_t ipropi_mode,
                                        en_IPROPI_SEL_MUX_t ipropi_sel)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data;

    reg_data = drv8000_reg_map[REGID_IC_CTRL];
    reg_data.Reg_IC_CTRL.IPROPI_MODE = ipropi_mode;
    
    if (IPROPI_INPUT_PWM == ipropi_mode)
    {
        reg_data.Reg_IC_CTRL.IPROPI_SEL = IPROPI_NO_OUT;
    } else
    {
        reg_data.Reg_IC_CTRL.IPROPI_SEL = ipropi_sel;
    }

    ret = drv8000_spi_write(DRV8000_ADDREG_IC_CTRL,
                                reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_IC_CTRL] = reg_data;
    }

    return ret;
}

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
                                        en_EN_SSC_t en_ssc,
                                        en_WD_EN_t wd_en)
{
    un_DRV8000_Reg_t reg_data;
    
    reg_data.Reg_IC_CNFG1.OTSD_MODE = otsd_mode;
    reg_data.Reg_IC_CNFG1.DIS_CP = dis_cp;
    reg_data.Reg_IC_CNFG1.PVDD_OV_MODE = pvdd_ov_mode;
    reg_data.Reg_IC_CNFG1.PVDD_OV_DG = pvdd_ov_dg;
    reg_data.Reg_IC_CNFG1.PVDD_OV_LVL = pvdd_ov_lvl;
    reg_data.Reg_IC_CNFG1.VCP_UV_LVL = vcp_uv_lvl;
    reg_data.Reg_IC_CNFG1.CP_MODE = cp_mode;
    reg_data.Reg_IC_CNFG1.VCP_UV_MODE = vcp_uv_mode;
    reg_data.Reg_IC_CNFG1.PVDD_UV_MODE = pvdd_uv_mode;
    reg_data.Reg_IC_CNFG1.WD_FLT_M = wd_flt_mode;
    reg_data.Reg_IC_CNFG1.WD_WIN = wd_window;
    reg_data.Reg_IC_CNFG1.EN_SSC = en_ssc;
    reg_data.Reg_IC_CNFG1.WD_EN = wd_en;
    
    return drv8000_spi_write(DRV8000_ADDREG_IC_CNFG1,
                                reg_data);
}

/**
 * @brief Watchdog trigger
 * 
 * Trigger watchdog bit to restart the watchdog timer.
 * Clear watchdog fault and POR if occurred and restart timer.
 * 
 * @return en_DRV8000_STST_t    Status code. Not returning watchdog fault.
 */
en_DRV8000_STST_t drv8000_wd_trig(void)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data;

    reg_data = drv8000_reg_map[REGID_IC_CTRL];
    reg_data.Reg_IC_CTRL.WD_RST ^= 1u; /* Invert watchdog bit */

    ret = drv8000_spi_write(DRV8000_ADDREG_IC_CTRL,
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_IC_CTRL] = reg_data;
    }
    {
        if (DRV8000_WATCHDOG_FAULT_STATUS == drv8000_global_spi_status.u8_SpiCommStatus)
        {
            (void)drv8000_spi_read(DRV8000_ADDREG_IC_STAT1,
                                    &reg_data);
            /* Check if it is watchdog timer fault */
            if (0u != reg_data.Reg_IC_STAT1.WD_FLT)
            {
                /* Clear the watchdog fault and sync watchdog bit */
                ret = drv8000_clear_fault();
            } /* Otherwise, it is other fault flag, return error code */
        }
    }

    return ret;
}

/* ** High Side Driver Control ** */
#ifdef GDU_HS_USED
en_DRV8000_STST_t drv8000_hs_driver_cnfg(en_HS_CNFG_t hs_out7_cnfg,
                                            en_HS_CNFG_t hs_out8_cnfg,
                                            en_HS_CNFG_t hs_out9_cnfg,
                                            en_HS_CNFG_t hs_out10_cnfg,
                                            en_HS_CNFG_t hs_out11_cnfg,
                                            en_HS_CNFG_t hs_out12_cnfg,
                                            en_HEATER_CNFG_t hs_heater_cnfg)
{
    en_DRV8000_STST_t ret = STST_SUCCESS;
    un_DRV8000_Reg_t reg_data;

    reg_data = drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG];

    if (BITS_NOT_CHANGE != hs_out7_cnfg)
    {
        reg_data.Reg_HS_HEAT_OUT_CNFG.OUT7_CNFG = hs_out7_cnfg;
    }
    if (BITS_NOT_CHANGE != hs_out8_cnfg)
    {
        reg_data.Reg_HS_HEAT_OUT_CNFG.OUT8_CNFG = hs_out8_cnfg;
    }
    if (BITS_NOT_CHANGE != hs_out9_cnfg)
    {
        reg_data.Reg_HS_HEAT_OUT_CNFG.OUT9_CNFG = hs_out9_cnfg;
    }
    if (BITS_NOT_CHANGE != hs_out10_cnfg)
    {
        reg_data.Reg_HS_HEAT_OUT_CNFG.OUT10_CNFG = hs_out10_cnfg;
    }
    if (BITS_NOT_CHANGE != hs_out11_cnfg)
    {
        reg_data.Reg_HS_HEAT_OUT_CNFG.OUT11_CNFG = hs_out11_cnfg;
    }
    if (BITS_NOT_CHANGE != hs_out12_cnfg)
    {
        reg_data.Reg_HS_HEAT_OUT_CNFG.OUT12_CNFG = hs_out12_cnfg;
    }
    if (BITS_NOT_CHANGE != hs_heater_cnfg)
    {
        reg_data.Reg_HS_HEAT_OUT_CNFG.HEAT_OUT_CNFG = hs_heater_cnfg;
    }

    ret = drv8000_spi_write(DRV8000_ADDREG_HS_HEAT_OUT_CNFG,
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG] = reg_data;
    }

    return ret;
}

en_DRV8000_STST_t drv8000_hs_driver_spi_enable(en_HS_EN_t hs_out7_en,
                                                en_HS_EN_t hs_out8_en,
                                                en_HS_EN_t hs_out9_en,
                                                en_HS_EN_t hs_out10_en,
                                                en_HS_EN_t hs_out11_en,
                                                en_HS_EN_t hs_out12_en,
                                                en_HS_EN_t hs_heater_en)
{
    en_DRV8000_STST_t ret = STST_SUCCESS;
    un_DRV8000_Reg_t reg_data;

    reg_data = drv8000_reg_map[REGID_HS_EC_HEAT_CTRL];

    if (BITS_NOT_CHANGE != hs_out7_en)
    {
        reg_data.Reg_HS_EC_HEAT_CTRL.OUT7_EN = hs_out7_en;
    }
    if (BITS_NOT_CHANGE != hs_out8_en)
    {
        reg_data.Reg_HS_EC_HEAT_CTRL.OUT8_EN = hs_out8_en;
    }
    if (BITS_NOT_CHANGE != hs_out9_en)
    {
        reg_data.Reg_HS_EC_HEAT_CTRL.OUT9_EN = hs_out9_en;
    }
    if (BITS_NOT_CHANGE != hs_out10_en)
    {
        reg_data.Reg_HS_EC_HEAT_CTRL.OUT10_EN = hs_out10_en;
    }
    if (BITS_NOT_CHANGE != hs_out11_en)
    {
        reg_data.Reg_HS_EC_HEAT_CTRL.OUT11_EN = hs_out11_en;
    }
    if (BITS_NOT_CHANGE != hs_out12_en)
    {
        reg_data.Reg_HS_EC_HEAT_CTRL.OUT12_EN = hs_out12_en;
    }
    if (BITS_NOT_CHANGE != hs_heater_en)
    {
        reg_data.Reg_HS_EC_HEAT_CTRL.OUT12_EN = hs_out12_en;
    }

    ret = drv8000_spi_write(DRV8000_ADDREG_HS_EC_HEAT_CTRL,
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_HS_EC_HEAT_CTRL] = reg_data;
    }

    return ret;
}

en_DRV8000_STST_t drv8000_hs_set_gen_pwm_dutycycle(en_HS_OUTx_t hs_outx,
                                                    uint16_t dutycycle)
{
    en_DRV8000_STST_t ret = STST_SUCCESS;
    uint8_t reg_addr;
    un_DRV8000_Reg_t reg_data;

    dutycycle = dutycycle & DRV8000_GEN_PWM_DC_MASK; /* 10 bits only */

    switch (hs_outx)
    {
        case HS_OUT_12:
            reg_addr = DRV8000_ADDREG_OUT12_PWM_DC;
            reg_data.Reg_OUT12_PWM_DC.OUT12_DC = dutycycle;
            break;
        case HS_OUT_11:
            reg_addr = DRV8000_ADDREG_OUT11_PWM_DC;
            reg_data.Reg_OUT11_PWM_DC.OUT11_DC = dutycycle;
            break;
        case HS_OUT_10:
            reg_addr = DRV8000_ADDREG_OUT10_PWM_DC;
            reg_data.Reg_OUT10_PWM_DC.OUT10_DC = dutycycle;
            break;
        case HS_OUT_9:
            reg_addr = DRV8000_ADDREG_OUT9_PWM_DC;
            reg_data.Reg_OUT9_PWM_DC.OUT9_DC = dutycycle;
            break;
        case HS_OUT_8:
            reg_addr = DRV8000_ADDREG_OUT8_PWM_DC;
            reg_data.Reg_OUT8_PWM_DC.OUT8_DC = dutycycle;
            break;
        case HS_OUT_7:
            reg_addr = DRV8000_ADDREG_OUT7_PWM_DC;
            reg_data.Reg_OUT7_PWM_DC.OUT7_DC = dutycycle;
            break;
        default:
            ret = STST_WRONG_MODE;
            break;
    }

    if (STST_SUCCESS == ret)
    {
        ret = drv8000_spi_write(reg_addr,
                                reg_data);
    }

    return ret;
}

en_DRV8000_STST_t drv8000_hs_set_gen_pwm_freq(en_HS_GEN_PWM_FREQ_t hs_out7_pwm_freq,
                                                en_HS_GEN_PWM_FREQ_t hs_out8_pwm_freq,
                                                en_HS_GEN_PWM_FREQ_t hs_out9_pwm_freq,
                                                en_HS_GEN_PWM_FREQ_t hs_out10_pwm_freq,
                                                en_HS_GEN_PWM_FREQ_t hs_out11_pwm_freq,
                                                en_HS_GEN_PWM_FREQ_t hs_out12_pwm_freq)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data;
    
    reg_data = drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG];

    reg_data.Reg_HS_PWM_FREQ.PWM_OUT7_FREQ = hs_out7_pwm_freq;
    reg_data.Reg_HS_PWM_FREQ.PWM_OUT8_FREQ = hs_out8_pwm_freq;
    reg_data.Reg_HS_PWM_FREQ.PWM_OUT9_FREQ = hs_out9_pwm_freq;
    reg_data.Reg_HS_PWM_FREQ.PWM_OUT10_FREQ = hs_out10_pwm_freq;
    reg_data.Reg_HS_PWM_FREQ.PWM_OUT11_FREQ = hs_out11_pwm_freq;
    reg_data.Reg_HS_PWM_FREQ.PWM_OUT12_FREQ = hs_out12_pwm_freq;

    ret = drv8000_spi_write(DRV8000_ADDREG_HS_PWM_FREQ_CNFG,
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG] = reg_data;
    }

    return ret;
}

/**
 * @brief Enables or disables constant current mode (CCM) for high-side driver outputs
 * 
 * Enables or disables a constant current for a short duration to the desired high-side outputs.
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
                                        en_HS_CCM_t hs_out12_ccm_en)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data;
    
    /* HS_REG_CNFG2 changes unexpectedly, requires RMW */
    ret = drv8000_spi_read(DRV8000_ADDREG_HS_REG_CNFG2,
                            &reg_data);
    
    if (STST_SUCCESS != ret)
    {
    	return ret;
    }
    if (BITS_NOT_CHANGE != hs_out7_ccm_en)
    {
        if (HS_CCM_DISABLE == hs_out7_ccm_en)
        {
            reg_data.Reg_HS_REG_CNFG2.OUT7_CCM_TO = 0u;
            reg_data.Reg_HS_REG_CNFG2.OUT7_CCM_EN = 0u;
        } else
        {
            reg_data.Reg_HS_REG_CNFG2.OUT7_CCM_TO = hs_out7_ccm_en;
            reg_data.Reg_HS_REG_CNFG2.OUT7_CCM_EN = 1u;
        }
    }
    if (BITS_NOT_CHANGE != hs_out8_ccm_en)
    {
        if (HS_CCM_DISABLE == hs_out8_ccm_en)
        {
            reg_data.Reg_HS_REG_CNFG2.OUT8_CCM_TO = 0u;
            reg_data.Reg_HS_REG_CNFG2.OUT8_CCM_EN = 0u;
        } else
        {
            reg_data.Reg_HS_REG_CNFG2.OUT8_CCM_TO = hs_out8_ccm_en;
            reg_data.Reg_HS_REG_CNFG2.OUT8_CCM_EN = 1u;
        }
    }
    if (BITS_NOT_CHANGE != hs_out9_ccm_en)
    {
        if (HS_CCM_DISABLE == hs_out9_ccm_en)
        {
            reg_data.Reg_HS_REG_CNFG2.OUT9_CCM_TO = 0u;
            reg_data.Reg_HS_REG_CNFG2.OUT9_CCM_EN = 0u;
        } else
        {
            reg_data.Reg_HS_REG_CNFG2.OUT9_CCM_TO = hs_out9_ccm_en;
            reg_data.Reg_HS_REG_CNFG2.OUT9_CCM_EN = 1u;
        }
    }
    if (BITS_NOT_CHANGE != hs_out10_ccm_en)
    {
        if (HS_CCM_DISABLE == hs_out10_ccm_en)
        {
            reg_data.Reg_HS_REG_CNFG2.OUT10_CCM_TO = 0u;
            reg_data.Reg_HS_REG_CNFG2.OUT10_CCM_EN = 0u;
        } else
        {
            reg_data.Reg_HS_REG_CNFG2.OUT10_CCM_TO = hs_out10_ccm_en;
            reg_data.Reg_HS_REG_CNFG2.OUT10_CCM_EN = 1u;
        }
    }
    if (BITS_NOT_CHANGE != hs_out11_ccm_en)
    {
        if (HS_CCM_DISABLE == hs_out11_ccm_en)
        {
            reg_data.Reg_HS_REG_CNFG2.OUT11_CCM_TO = 0u;
            reg_data.Reg_HS_REG_CNFG2.OUT11_CCM_EN = 0u;
        } else
        {
            reg_data.Reg_HS_REG_CNFG2.OUT11_CCM_TO = hs_out11_ccm_en;
            reg_data.Reg_HS_REG_CNFG2.OUT11_CCM_EN = 1u;
        }
    }
    if (BITS_NOT_CHANGE != hs_out12_ccm_en)
    {
        if (HS_CCM_DISABLE == hs_out12_ccm_en)
        {
            reg_data.Reg_HS_REG_CNFG2.OUT12_CCM_TO = 0u;
            reg_data.Reg_HS_REG_CNFG2.OUT12_CCM_EN = 0u;
        } else
        {
            reg_data.Reg_HS_REG_CNFG2.OUT12_CCM_TO = hs_out12_ccm_en;
            reg_data.Reg_HS_REG_CNFG2.OUT12_CCM_EN = 1u;
        }
    }

    return drv8000_spi_write(DRV8000_ADDREG_HS_REG_CNFG2,
                                reg_data);
}

/* ** Electrochromic Driver Control ** */
en_DRV8000_STST_t drv8000_ec_driver_enable(en_HS_EN_t hs_ec_en,
                                            en_EC_ECFB_LS_EN_t es_ecfb_ls_en,
                                            uint8_t ec_v_tar)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data = drv8000_reg_map[REGID_HS_EC_HEAT_CTRL];

    reg_data.Reg_HS_EC_HEAT_CTRL.EC_ON = hs_ec_en;
    reg_data.Reg_HS_EC_HEAT_CTRL.ECFB_LS_EN = es_ecfb_ls_en;
    reg_data.Reg_HS_EC_HEAT_CTRL.EC_V_TAR = (ec_v_tar & DRV8000_EC_V_TAR_MASK); /* 6 bits */

    ret = drv8000_spi_write(DRV8000_ADDREG_HS_EC_HEAT_CTRL,
                            drv8000_reg_map[REGID_HS_EC_HEAT_CTRL]);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_HS_EC_HEAT_CTRL] = reg_data;
    }

    return ret;
}

en_DRV8000_STST_t drv8000_ec_driver_cnfg(EC_ECFB_MAX_t ecfb_max_v,
                                            EC_OLEN_t ec_olen,
                                            EC_ECFB_LS_PWM_t ecfb_ls_pwm,
                                            EC_FLT_MODE_t ec_flt_mode,
                                            EC_ECFB_UV_OV_MODE_t ecfb_ov_mode,
                                            EC_ECFB_UV_OV_MODE_t ecfb_uv_mode,
                                            EC_ECFB_UV_OV_DG_t ecfb_ov_dg,
                                            EC_ECFB_UV_OV_DG_t ecfb_uv_dg,
                                            en_EC_ECFB_UV_TH_t ecfb_uv_th,
                                            en_EC_ECDRV_OL_EN_t ecdrv_ol_en)
{
    un_DRV8000_Reg_t reg_data;

    reg_data.u16_RegWord = 0u;
    reg_data.Reg_EC_CNFG.ECFB_MAX = ecfb_max_v;
    reg_data.Reg_EC_CNFG.EC_OLEN = ec_olen;
    reg_data.Reg_EC_CNFG.ECFB_LS_PWM = ecfb_ls_pwm;
    reg_data.Reg_EC_CNFG.EC_FLT_MODE = ec_flt_mode;
    reg_data.Reg_EC_CNFG.ECFB_OV_MODE = ecfb_ov_mode;
    reg_data.Reg_EC_CNFG.ECFB_UV_MODE = ecfb_uv_mode;
    reg_data.Reg_EC_CNFG.ECFB_OV_DG = ecfb_ov_dg;
    reg_data.Reg_EC_CNFG.ECFB_UV_DG = ecfb_uv_dg;
    reg_data.Reg_EC_CNFG.ECFB_UV_TH = ecfb_uv_th;
    reg_data.Reg_EC_CNFG.ECDRV_OL_EN = ecdrv_ol_en;
    
    return drv8000_spi_write(DRV8000_ADDREG_EC_CNFG,
                                reg_data);
}
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
                                                en_HHB_CNFG_t hhb_out4_cnfg)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data = drv8000_reg_map[REGID_HB_OUT_CNFG2];

    if (BITS_NOT_CHANGE != hhb_out1_cnfg)
    {
        reg_data.Reg_HB_OUT_CNFG2.OUT1_CNFG = hhb_out1_cnfg;
    }
    if (BITS_NOT_CHANGE != hhb_out2_cnfg)
    {
        reg_data.Reg_HB_OUT_CNFG2.OUT2_CNFG = hhb_out2_cnfg;
    }
    if (BITS_NOT_CHANGE != hhb_out3_cnfg)
    {
        reg_data.Reg_HB_OUT_CNFG2.OUT3_CNFG = hhb_out3_cnfg;
    }
    if (BITS_NOT_CHANGE != hhb_out4_cnfg)
    {
        reg_data.Reg_HB_OUT_CNFG2.OUT4_CNFG = hhb_out4_cnfg;
    }
    
    ret = drv8000_spi_write(DRV8000_ADDREG_HB_OUT_CNFG2,
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG2] = reg_data;
    }

    return ret;
}

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
                                                en_HHB_CNFG_t hhb_out6_cnfg)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data = drv8000_reg_map[REGID_HB_OUT_CNFG1];

    if (BITS_NOT_CHANGE != hhb_out5_cnfg)
    {
        reg_data.Reg_HB_OUT_CNFG1.OUT5_CNFG = hhb_out5_cnfg;
    }
    if (BITS_NOT_CHANGE != hhb_out6_cnfg)
    {
        reg_data.Reg_HB_OUT_CNFG1.OUT6_CNFG = hhb_out6_cnfg;
    }

    ret = drv8000_spi_write(DRV8000_ADDREG_HB_OUT_CNFG1,
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG1] = reg_data;
    }

    return ret;
}

en_DRV8000_STST_t drv8000_hhb_spi_enable(en_HHB_EN_t hhb_out1_en,
                                            en_HHB_EN_t hhb_out2_en,
                                            en_HHB_EN_t hhb_out3_en,
                                            en_HHB_EN_t hhb_out4_en,
                                            en_HHB_EN_t hhb_out5_en,
                                            en_HHB_EN_t hhb_out6_en)
{
    en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data = drv8000_reg_map[REGID_GD_HB_CTRL];

    if (BITS_NOT_CHANGE != hhb_out1_en)
    {
        reg_data.Reg_GD_HB_CTRL.OUT1_CTRL = hhb_out1_en;
    }
    if (BITS_NOT_CHANGE != hhb_out2_en)
    {
        reg_data.Reg_GD_HB_CTRL.OUT2_CTRL = hhb_out2_en;
    }
    if (BITS_NOT_CHANGE != hhb_out3_en)
    {
        reg_data.Reg_GD_HB_CTRL.OUT3_CTRL = hhb_out3_en;
    }
    if (BITS_NOT_CHANGE != hhb_out4_en)
    {
        reg_data.Reg_GD_HB_CTRL.OUT4_CTRL = hhb_out4_en;
    }
    if (BITS_NOT_CHANGE != hhb_out5_en)
    {
        reg_data.Reg_GD_HB_CTRL.OUT5_CTRL = hhb_out5_en;
    }
    if (BITS_NOT_CHANGE != hhb_out6_en)
    {
        reg_data.Reg_GD_HB_CTRL.OUT6_CTRL = hhb_out6_en;
    }

    ret = drv8000_spi_write(DRV8000_ADDREG_GD_HB_CTRL, 
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_GD_HB_CTRL] = reg_data;
    }

    return ret;
}

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
 * @return en_DRV8000_STST_t    Status code
 */
en_DRV8000_STST_t drv8000_hhb_set_fw(en_HHB_FW_t hhb_out1_fw,
                                        en_HHB_FW_t hhb_out2_fw,
                                        en_HHB_FW_t hhb_out3_fw,
                                        en_HHB_FW_t hhb_out4_fw,
                                        en_HHB_FW_t hhb_out5_fw,
                                        en_HHB_FW_t hhb_out6_fw)
{
	en_DRV8000_STST_t ret;
    un_DRV8000_Reg_t reg_data = drv8000_reg_map[REGID_HB_OUT_CNFG1];

    reg_data.Reg_HB_OUT_CNFG1.NSR_OUT1_DIS = hhb_out1_fw;
    reg_data.Reg_HB_OUT_CNFG1.NSR_OUT2_DIS = hhb_out2_fw;
    reg_data.Reg_HB_OUT_CNFG1.NSR_OUT3_DIS = hhb_out3_fw;
    reg_data.Reg_HB_OUT_CNFG1.NSR_OUT4_DIS = hhb_out4_fw;
    reg_data.Reg_HB_OUT_CNFG1.NSR_OUT5_DIS = hhb_out5_fw;
    reg_data.Reg_HB_OUT_CNFG1.NSR_OUT6_DIS = hhb_out6_fw;
    
    ret = drv8000_spi_write(DRV8000_ADDREG_HB_OUT_CNFG1, 
                            reg_data);
    
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG1] = reg_data;
    }
    
    return ret;
}

en_DRV8000_STST_t drv8000_hhb_set_ocp_dg(en_HHB_OCP_DG_t hhb_out1_ocp_dg,
                                            en_HHB_OCP_DG_t hhb_out2_ocp_dg,
                                            en_HHB_OCP_DG_t hhb_out3_ocp_dg,
                                            en_HHB_OCP_DG_t hhb_out4_ocp_dg,
                                            en_HHB_OCP_DG_t hhb_out5_ocp_dg,
                                            en_HHB_OCP_DG_t hhb_out6_ocp_dg)
{
    un_DRV8000_Reg_t reg_data;
    
    reg_data.u16_RegWord = 0u;
    
    reg_data.Reg_HB_OCP_CNFG.OUT1_OCP_DG = hhb_out1_ocp_dg;
    reg_data.Reg_HB_OCP_CNFG.OUT2_OCP_DG = hhb_out2_ocp_dg;
    reg_data.Reg_HB_OCP_CNFG.OUT3_OCP_DG = hhb_out3_ocp_dg;
    reg_data.Reg_HB_OCP_CNFG.OUT4_OCP_DG = hhb_out4_ocp_dg;
    reg_data.Reg_HB_OCP_CNFG.OUT5_OCP_DG = hhb_out5_ocp_dg;
    reg_data.Reg_HB_OCP_CNFG.OUT6_OCP_DG = hhb_out6_ocp_dg;

    return drv8000_spi_write(DRV8000_ADDREG_HB_OCP_CNFG, 
                                reg_data);
}

en_DRV8000_STST_t drv8000_hhb_set_itrip_lvl(en_HHB_ITRIP_t hhb_out1_itrip_lvl,
                                            en_HHB_ITRIP_t hhb_out2_itrip_lvl,
                                            en_HHB_ITRIP_t hhb_out3_itrip_lvl,
                                            en_HHB_ITRIP_t hhb_out4_itrip_lvl,
                                            en_HHB_ITRIP_t hhb_out5_itrip_lvl,
                                            en_HHB_ITRIP_t hhb_out6_itrip_lvl)
{
    un_DRV8000_Reg_t reg_data;
    
    reg_data.u16_RegWord = 0u;
    
    if (HHB_ITRIP_DISABLE != hhb_out1_itrip_lvl)
    {
        reg_data.Reg_HB_ITRIP_CNFG.OUT1_ITRIP_LVL = hhb_out1_itrip_lvl;
        reg_data.Reg_HB_ITRIP_CNFG.OUT1_ITRIP_EN = 1u;
    }
    if (HHB_ITRIP_DISABLE != hhb_out1_itrip_lvl)
    {
        reg_data.Reg_HB_ITRIP_CNFG.OUT2_ITRIP_LVL = hhb_out2_itrip_lvl;
        reg_data.Reg_HB_ITRIP_CNFG.OUT2_ITRIP_EN = 1u;
    }
    if (HHB_ITRIP_DISABLE != hhb_out1_itrip_lvl)
    {
        reg_data.Reg_HB_ITRIP_CNFG.OUT3_ITRIP_LVL = hhb_out3_itrip_lvl;
        reg_data.Reg_HB_ITRIP_CNFG.OUT3_ITRIP_EN = 1u;
    }
    if (HHB_ITRIP_DISABLE != hhb_out1_itrip_lvl)
    {
        reg_data.Reg_HB_ITRIP_CNFG.OUT4_ITRIP_LVL = hhb_out4_itrip_lvl;
        reg_data.Reg_HB_ITRIP_CNFG.OUT4_ITRIP_EN = 1u;
    }
    if (HHB_ITRIP_DISABLE != hhb_out1_itrip_lvl)
    {
        reg_data.Reg_HB_ITRIP_CNFG.OUT5_ITRIP_LVL = hhb_out5_itrip_lvl;
        reg_data.Reg_HB_ITRIP_CNFG.OUT5_ITRIP_EN = 1u;
    }
    if (HHB_ITRIP_DISABLE != hhb_out1_itrip_lvl)
    {
        reg_data.Reg_HB_ITRIP_CNFG.OUT6_ITRIP_LVL = hhb_out6_itrip_lvl;
        reg_data.Reg_HB_ITRIP_CNFG.OUT6_ITRIP_EN = 1u;
    }    
    
    return drv8000_spi_write(DRV8000_ADDREG_HB_ITRIP_CNFG,
                                reg_data);
}
#endif /* GDU_HHB_USED */

/* *** H-bridge Control *** */
#ifdef GDU_GD_USED
en_DRV8000_STST_t drv8000_spi_gd_enable_disable(en_GD_EN_t en_gd)
{
    un_DRV8000_Reg_t reg_data = drv8000_reg_map[REGID_GD_CNFG];
    reg_data.Reg_GD_CNFG.EN_GD = en_gd;

    en_DRV8000_STST_t ret = drv8000_spi_write(DRV8000_ADDREG_GD_CNFG,
                                                reg_data);
    if (STST_SUCCESS == ret)
    {
        drv8000_reg_map[REGID_GD_CNFG] = reg_data;
    }

    return ret;
}

en_DRV8000_STST_t drv8000_gd_hb_set_mode(en_GD_BRG_MODE_t brg_mode,
                                            en_GD_INx_MODE_t in1_spi_mode,
                                            en_GD_INx_MODE_t in2_spi_mode,
                                            en_GD_FBRG_FW_MODE_t fw_mode)
{
    en_DRV8000_STST_t ret = STST_SUCCESS;
    un_DRV8000_Reg_t reg_data = drv8000_reg_map[REGID_GD_CNFG];

    switch (brg_mode)
    {
        case GD_BRG_MODE_HALF_BRG:
            (void)fw_mode;
            reg_data.Reg_GD_CNFG.IN1_MODE = in1_spi_mode;
            reg_data.Reg_GD_CNFG.IN2_MODE = in2_spi_mode;
            break;
#ifdef GDU_GD_IN2_GPIO
        case GD_BRG_MODE_PH_EN:
            /* Cannot have both as SPI pin during this mode */
            if ((GD_INx_SPI == in1_spi_mode) && 
                (GD_INx_SPI == in2_spi_mode))
            {
                ret = STST_WRONG_MODE;
            } else
            {
                reg_data.Reg_GD_CNFG.BRG_FW = fw_mode;
                reg_data.Reg_GD_CNFG.IN1_MODE = in1_spi_mode;
                reg_data.Reg_GD_CNFG.IN2_MODE = in2_spi_mode;
            }

            break;
#else /* not defined GDU_GD_IN2_GPIO */
        case GD_BRG_MODE_PWM:
            (void)in1_spi_mode;
            (void)in2_spi_mode;
            reg_data.Reg_GD_CNFG.BRG_FW = fw_mode;
            break;
#endif /* GDU_GD_IN2_GPIO */
        default:
            ret = STST_WRONG_MODE;
            break;
    }

    if (STST_SUCCESS == ret)
    {
        reg_data.Reg_GD_CNFG.BRG_MODE = brg_mode;

        ret = drv8000_spi_write(DRV8000_ADDREG_GD_CNFG,
                                drv8000_reg_map[REGID_GD_CNFG]);
        
        if (STST_SUCCESS == ret)
        {
            drv8000_reg_map[REGID_GD_CNFG] = reg_data;
        }
    }

    return ret;
}

en_DRV8000_STST_t drv8000_gd_hb_set_direction(en_GD_FBRG_DIRECTION_t direction)
{
    en_DRV8000_STST_t ret = STST_SUCCESS;
    
        /* GD_BRG_MODE_HALF_BRG does not have direction and GD_BRG_MODE_PWM uses PWM to control direction */
    if ((GD_BRG_MODE_PH_EN != drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.BRG_MODE) || 
        /* If IN1 uses SPI, means that IN2 is control direction using PWM */
        (GD_INx_SPI == drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.IN2_MODE))
    {
        ret = STST_WRONG_MODE;
    } else
    {
        if (GD_INx_SPI == drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.IN2_MODE)
        {
            un_DRV8000_Reg_t reg_data = drv8000_reg_map[REGID_GD_HB_CTRL];
            reg_data.Reg_GD_HB_CTRL.S_IN2 = direction;

            ret = drv8000_spi_write(DRV8000_ADDREG_GD_HB_CTRL,
                                    drv8000_reg_map[REGID_GD_HB_CTRL]);
            
            if (STST_SUCCESS == ret)
            {
                drv8000_reg_map[REGID_GD_HB_CTRL] = reg_data;
            }
        } else
        {
#ifdef GDU_GD_IN2_GPIO
            ret = drv8000_gpio_set(drv8000_interface->gd_in2_port,
                                    drv8000_interface->gd_in2_pin,
                                    (uint8_t)direction);
#else /* not defined GDU_GD_IN2_GPIO */
            ret = STST_WRONG_MODE;
#endif /* GDU_GD_IN2_GPIO */
        }
    }

    return ret;
}
#endif /* GDU_GD_USED */

en_DRV8000_STST_t drv8000_init(st_DRV8000_Interface_t* interface)
{
    en_DRV8000_STST_t ret;
    
    drv8000_interface = interface;
    ret = drv8000_reset();
    
    if (STST_SUCCESS == ret)
    {
        if (drv8000_read_devid() != DRV8000_DEFVAL_DEVICE_ID)
        {
            ret = STST_SPI_ERR;
        }
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_clear_fault();
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_regs_periodic_read();
    }
    
    return ret;
}

/**
 * @brief Get the device SPI IC status
 * 
 * @return un_DRV8000_SPI_STST_t    Device IC status from SPI received frame (see `st_SpiCommStatus` struct).
 */
un_DRV8000_SPI_STST_t drv8000_spi_status_get(void)
{
    return drv8000_global_spi_status;
}

en_DRV8000_STST_t drv8000_regs_periodic_read(void)
{
    en_DRV8000_STST_t ret = STST_SUCCESS;

    if (STST_SPI_ERR != ret)
    {
        ret = drv8000_spi_read(DRV8000_ADDREG_GD_CNFG, 
                                &drv8000_reg_map[REGID_GD_CNFG]);
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_spi_read(DRV8000_ADDREG_HB_OUT_CNFG1, 
                                &drv8000_reg_map[REGID_HB_OUT_CNFG1]);
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_spi_read(DRV8000_ADDREG_HB_OUT_CNFG2, 
                                &drv8000_reg_map[REGID_HB_OUT_CNFG2]);
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_spi_read(DRV8000_ADDREG_HS_HEAT_OUT_CNFG, 
                                &drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG]);
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_spi_read(DRV8000_ADDREG_HS_PWM_FREQ_CNFG, 
                                &drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG]);
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_spi_read(DRV8000_ADDREG_IC_CTRL, 
                                &drv8000_reg_map[REGID_IC_CTRL]);
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_spi_read(DRV8000_ADDREG_GD_HB_CTRL, 
                                &drv8000_reg_map[REGID_GD_HB_CTRL]);
    }
    if (STST_SUCCESS == ret)
    {
        ret = drv8000_spi_read(DRV8000_ADDREG_HS_EC_HEAT_CTRL, 
                                &drv8000_reg_map[REGID_HS_EC_HEAT_CTRL]);
    }

    return ret;
}

/* **********************************************************************/
/* ***             Definition of local functions                      ***/
/* **********************************************************************/
static en_DRV8000_STST_t drv8000_spi_status_check(void)
{
    if (DRV8000_SUCCESS_SPI_STATUS != drv8000_global_spi_status.u8_SpiCommStatus)
    {
        if (1u == drv8000_global_spi_status.st_SpiCommStatus.SPI_ERR)
        {
            return STST_SPI_ERR;
        }
        if (1u == drv8000_global_spi_status.st_SpiCommStatus.FAULT)
        {
            return STST_GDU_FAULT;
        }
        if (1u == drv8000_global_spi_status.st_SpiCommStatus.WARN)
        {
            return STST_GDU_WARN;
        }

        return STST_GDU_UNKN_ERR;
    }

    return STST_SUCCESS;
}

static en_DRV8000_STST_t drv8000_spi_read(const uint8_t reg_addr,
                                            un_DRV8000_Reg_t* reg_val)
{
    if (NULL == drv8000_interface)
    {
        return STST_NULL_INTERFACE_PTR;
    }

    en_DRV8000_STST_t ret = STST_SUCCESS;
    un_DRV8000_SDI_FRAME_t spi_transmit;
    un_DRV8000_SDO_FRAME_t spi_receive;

    spi_transmit.st_SpiCommand.Ignored = 0u;
    spi_transmit.st_SpiCommand.MSBBit = 0u;
    spi_transmit.st_SpiCommand.DataField = 0u;
    spi_transmit.st_SpiCommand.AccessType = DRV8000_READ_ACCESS;
    spi_transmit.st_SpiCommand.Address = reg_addr & DRV8000_REG_ADDRESS_MASK;

    ret = (en_DRV8000_STST_t)drv8000_spi_transceive((uint8_t*)&spi_transmit,
                                                    (uint8_t*)&spi_receive,
                                                    (uint16_t)DRV8000_SPI_FRAME_LEN);

    if (STST_SUCCESS == ret)
    {
        drv8000_global_spi_status.u8_SpiCommStatus = spi_receive.st_SpiCommand.SpiStastus & DRV8000_GLOBAL_STATUS_MASK;
        ret = drv8000_spi_status_check();

        /* Update returned register value if SPI ok */
        if (0u == drv8000_global_spi_status.st_SpiCommStatus.SPI_ERR)
        {
            reg_val->u16_RegWord = spi_receive.st_SpiCommand.DataField;
        }
    } else
    {
        ret = STST_SPI_ERR;
    }

    return ret;
}

static en_DRV8000_STST_t drv8000_spi_write(const uint8_t reg_addr, 
                                            const un_DRV8000_Reg_t reg_val)
{
    if (NULL == drv8000_interface)
    {
        return STST_NULL_INTERFACE_PTR;
    }
    if (LOCK_REG_WRITE == drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CTRL_LOCK)
    {
        /* Control register locked, cannot write (except to IC_CTRL) */
        if (reg_addr > DRV8000_ADDREG_IC_CTRL)
        {
            return STST_CTRL_REG_LOCKED;
        }
    }
    if (LOCK_REG_WRITE == drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CNFG_LOCK)
    {
        /* Config register locked, cannot write */
        if (reg_addr < DRV8000_ADDREG_IC_CTRL)
        {
            return STST_CNFG_REG_LOCKED;
        }
    }

    en_DRV8000_STST_t ret = STST_SUCCESS;
    un_DRV8000_SDI_FRAME_t spi_transmit;
    un_DRV8000_SDO_FRAME_t spi_receive;

    spi_transmit.st_SpiCommand.Ignored = 0u;
    spi_transmit.st_SpiCommand.MSBBit = 0u;
    spi_transmit.st_SpiCommand.AccessType = DRV8000_WRITE_ACCESS;
    spi_transmit.st_SpiCommand.Address = reg_addr & DRV8000_REG_ADDRESS_MASK;
    spi_transmit.st_SpiCommand.DataField = reg_val.u16_RegWord & DRV8000_REG_DATA_MASK;

    ret = (en_DRV8000_STST_t)drv8000_spi_transceive((uint8_t*)&spi_transmit,
                                                    (uint8_t*)&spi_receive,
                                                    (uint16_t)DRV8000_SPI_FRAME_LEN);

    if (STST_SUCCESS == ret)
    {
        drv8000_global_spi_status.u8_SpiCommStatus = spi_receive.st_SpiCommand.SpiStastus & DRV8000_GLOBAL_STATUS_MASK;
        ret = drv8000_spi_status_check();
    } else
    {
        ret = STST_SPI_ERR;
    }

    return ret;
}

static en_DRV8000_STST_t drv8000_gpio_set(void* port,
                                            uint8_t pin,
                                            uint8_t pin_level)
{
    en_DRV8000_STST_t ret = (en_DRV8000_STST_t)drv8000_gpio(port, 
                                                            pin, 
                                                            pin_level);
    
    if (STST_SUCCESS == ret)
    {
        return STST_SUCCESS;
    } else
    {
        return STST_GIO_ERR;
    }
}

static en_DRV8000_STST_t drv8000_pwm_set(uint8_t instance,
                                            uint8_t channel,
                                            uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                            ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                                        )
{
#ifndef GDU_PWM_PERIOD_FIXED
    if (dutycycle > period)
    {
        dutycycle = period;
    }

    (void)drv8000_pwm_set_period(instance,
                                    channel,
                                    period);
#endif /* GDU_PWM_PERIOD_FIXED */

    en_DRV8000_STST_t ret = (en_DRV8000_STST_t)drv8000_pwm_set_dc(instance,
                                                                    channel,
                                                                    dutycycle);
    
    if (STST_SUCCESS == ret)
    {
        return STST_SUCCESS;
    } else
    {
        return STST_PWM_ERR;
    }
}