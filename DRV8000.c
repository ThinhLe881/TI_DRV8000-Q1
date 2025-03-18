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


/* **********************************************************************/
/* ***              Definition of local variables                     ***/
/* **********************************************************************/
static un_DRV8000_Reg_t drv8000_reg_map[DRV8000_NUM_OF_REGS];
static un_DRV8000_SPI_STST_t drv8000_global_spi_status;


/* **********************************************************************/
/* ***             Declaration of local functions                     ***/
/* **********************************************************************/
uint8_t drv8000_spi_status_check(void);

un_DRV8000_SPI_STST_t drv8000_spi_status_get(void);

uint8_t drv8000_spi_read(st_DRV8000_Interface_t* interface,
                         const uint8_t reg_addr,
                         un_DRV8000_Reg_t* reg_val);

uint8_t drv8000_spi_write(st_DRV8000_Interface_t* interface,
                          const uint8_t reg_addr,
                          const un_DRV8000_Reg_t reg_val);

uint8_t drv8000_gpio_set(st_DRV8000_Interface_t* interface,
                         uint8_t port,
                         uint8_t pin,
                         uint8_t pin_level);

uint8_t drv8000_pwm_set(st_DRV8000_Interface_t* interface,
                             uint8_t instance,
                             uint8_t channel,
                             uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                            );

uint8_t drv8000_delay_set(st_DRV8000_Interface_t* interface,
                          uint16_t delay_us);


/* **********************************************************************/
/* ***              Definition of global variables                    ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***            Definition of global functions                      ***/
/* **********************************************************************/
/* *** PIN Control *** */
#ifdef GDU_GD_USED
uint8_t drv8000_pin_gd_enable(st_DRV8000_Interface_t* interface)
{
    /* Enable gate driver - gate driver half-bridges are pulled up to PVDD */
    /* De-assert DRVOFF does not enable gate driver, 
        use drv8000_enable_gate_driver() to flip gate driver enable bit */
    uint8_t ret = 0u;
    ret = drv8000_gpio_set(interface,
                           interface->drvoff_port,
                           interface->drvoff_pin,
                           0u);

    if (0u == ret)
    {
        drv8000_delay_set(interface, 3000u); /* Wait ~3ms to register a valid DRVOFF command */
        ret = drv8000_clear_fault(interface); /* Clear DRVOFF_STAT flag */

        if (0u == ret)
        {
            ret = drv8000_spi_read(interface, DRV8000_ADDREG_GD_STAT, &drv8000_reg_map[REGID_GD_STAT]);

            if (0u != ret || 0u != drv8000_reg_map[REGID_GD_STAT].Reg_GD_STAT.DRVOFF_STAT)
            {
                ret = 1u;
            }
        }
    }

    return ret;
}

uint8_t drv8000_pin_gd_disable(st_DRV8000_Interface_t* interface)
{
    /* Disable gate driver - pull down gate driver half-bridges to Hi-Z */
    uint8_t ret = 0u;
    ret = drv8000_gpio_set(interface,
                           interface->drvoff_port,
                           interface->drvoff_pin,
                           1u);

    if (0u == ret)
    {
        drv8000_delay_set(interface, 3000u); /* Wait ~3ms to register a valid DRVOFF command */
        ret = drv8000_spi_read(interface, DRV8000_ADDREG_GD_STAT, &drv8000_reg_map[REGID_GD_STAT]);

        if (0u != ret || 1u != drv8000_reg_map[REGID_GD_STAT].Reg_GD_STAT.DRVOFF_STAT)
        {
            ret = 1u;
        }
    }

    return ret;
}

uint8_t drv8000_pin_set_gd_in1(st_DRV8000_Interface_t* interface,
                               uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                              ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                              )
{
    return drv8000_pwm_set(interface,
                                interface->pwm_gd_in1_instance,
                                interface->pwm_gd_in1_channel,
                                dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                ,period
#endif /* GDU_PWM_PERIOD_FIXED */
                            );
}

#ifndef GDU_GD_IN2_GPIO
uint8_t drv8000_pin_set_gd_in2(st_DRV8000_Interface_t* interface,
                               uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                              ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                              )
{
    return drv8000_pwm_set(interface,
                                interface->pwm_gd_in2_instance,
                                interface->pwm_gd_in2_channel,
                                dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                                ,period
#endif /* GDU_PWM_PERIOD_FIXED */
                            );
}
#endif /* GDU_GD_IN2_GPIO */
#endif /* GDU_GD_USED */

uint8_t drv8000_pin_set_pwm1(st_DRV8000_Interface_t* interface,
                             uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                            )
{
    return drv8000_pwm_set(interface,
                           interface->pwm1_instance,
                           interface->pwm1_channel,
                           dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                          ,period
#endif /* GDU_PWM_PERIOD_FIXED */
                          );
}

#ifdef GDU_PWM2_USED
uint8_t drv8000_pin_set_pwm2(st_DRV8000_Interface_t* interface,
                             uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                            )
{
    return drv8000_pwm_set(interface,
                           interface->pwm2_instance,
                           interface->pwm2_channel,
                           dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                          ,period
#endif /* GDU_PWM_PERIOD_FIXED */
                          );
}
#endif /* GDU_PWM2_USED */

uint8_t drv8000_wakeup(st_DRV8000_Interface_t* interface)
{
    return drv8000_gpio_set(interface,
                            interface->nsleep_port,
                            interface->nsleep_pin,
                            1u);
}

uint8_t drv8000_sleep_mode(st_DRV8000_Interface_t* interface)
{
    return drv8000_gpio_set(interface,
                            interface->nsleep_port,
                            interface->nsleep_pin,
                            0u);
}

uint8_t drv8000_reset(st_DRV8000_Interface_t* interface)
{
    uint8_t ret = 0u;

    if (NULL == interface || NULL == interface->fptr_Gpio || NULL == interface->fptr_Delay)
    {
        ret = 1u;
    } else {
        /* Sleep DRV8000 */
        ret = interface->fptr_Gpio(interface->nsleep_port,
                                   interface->nsleep_pin,
                                   0u);

        if (0u == ret)
        {
            /* (Optional) wait ~2us for DRV8000 to completely disabled, otherwise register values won't reset */
            interface->fptr_Delay(2u);
            /* Wakeup DRV8000 */
            ret = interface->fptr_Gpio(interface->nsleep_port,
                                       interface->nsleep_pin,
                                       1u);
            if (0u == ret)
            {
                /* (Optional) wait ~6us for DRV8000 to completely wakeup, otherwise SPI comm won't work */
                interface->fptr_Delay(6u);
                /* Set register default value */
                drv8000_reg_map[REGID_IC_CTRL].u16_RegWord = DRV8000_DEFVAL_IC_CTRL;
            }
        }
    }

    return ret;
}

/* *** SPI Read *** */
uint8_t drv8000_read_status_registers(st_DRV8000_Interface_t* interface)
{
    uint8_t ret = 0u;

    ret = drv8000_spi_read(interface,
                           DRV8000_ADDREG_IC_STAT1,
                           &drv8000_reg_map[REGID_IC_STAT1]);

    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_IC_STAT2,
                               &drv8000_reg_map[REGID_IC_STAT2]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_STAT, 
                              &drv8000_reg_map[REGID_GD_STAT]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_STAT1,
                               &drv8000_reg_map[REGID_HB_STAT1]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_STAT1,
                               &drv8000_reg_map[REGID_HB_STAT2]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_EC_HEAT_IT_RIP_STAT,
                               &drv8000_reg_map[REGID_EC_HEAT_IT_RIP_STAT]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_STAT,
                               &drv8000_reg_map[REGID_HS_STAT]);
    }

    return ret;
}

uint8_t drv8000_read_config_registers(st_DRV8000_Interface_t* interface)
{
    uint8_t ret = 0u;

    ret = drv8000_spi_read(interface,
                           DRV8000_ADDREG_IC_CNFG1,
                           &drv8000_reg_map[REGID_IC_CNFG1]);

    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_IC_CNFG2,
                               &drv8000_reg_map[REGID_IC_CNFG2]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_CNFG,
                               &drv8000_reg_map[REGID_GD_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_IDRV_CNFG,
                               &drv8000_reg_map[REGID_GD_IDRV_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_VGS_CNFG,
                               &drv8000_reg_map[REGID_GD_VGS_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_VDS_CNFG,
                               &drv8000_reg_map[REGID_GD_VDS_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_CSA_CNFG,
                               &drv8000_reg_map[REGID_GD_CSA_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_AGD_CNFG,
                               &drv8000_reg_map[REGID_GD_AGD_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_PDR_CNFG,
                               &drv8000_reg_map[REGID_GD_PDR_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_STC_CNFG,
                               &drv8000_reg_map[REGID_GD_STC_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_ITRIP_DG,
                               &drv8000_reg_map[REGID_HB_ITRIP_DG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_OUT_CNFG1,
                               &drv8000_reg_map[REGID_HB_OUT_CNFG1]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_OUT_CNFG2,
                               &drv8000_reg_map[REGID_HB_OUT_CNFG2]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_OCP_CNFG,
                               &drv8000_reg_map[REGID_HB_OCP_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_OL_CNFG1,
                               &drv8000_reg_map[REGID_HB_OL_CNFG1]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_OL_CNFG2,
                               &drv8000_reg_map[REGID_HB_OL_CNFG2]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_SR_CNFG,
                               &drv8000_reg_map[REGID_HB_SR_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_ITRIP_CNFG,
                               &drv8000_reg_map[REGID_HB_ITRIP_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HB_ITRIP_FREQ,
                               &drv8000_reg_map[REGID_HB_ITRIP_FREQ]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_HEAT_OUT_CNFG,
                               &drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_OC_CNFG,
                               &drv8000_reg_map[REGID_HS_OC_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_OL_CNFG,
                               &drv8000_reg_map[REGID_HS_OL_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_REG_CNFG1,
                               &drv8000_reg_map[REGID_HS_REG_CNFG1]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_REG_CNFG2,
                               &drv8000_reg_map[REGID_HS_REG_CNFG2]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_PWM_FREQ_CNFG,
                               &drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HEAT_CNFG,
                               &drv8000_reg_map[REGID_HEAT_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_EC_CNFG,
                               &drv8000_reg_map[REGID_EC_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_OCP_DG,
                               &drv8000_reg_map[REGID_HS_OCP_DG]);
    }

    return ret;
}

uint8_t drv8000_read_control_registers(st_DRV8000_Interface_t* interface)
{
    uint8_t ret = 0u;

    ret = drv8000_spi_read(interface,
                           DRV8000_ADDREG_IC_CTRL,
                           &drv8000_reg_map[REGID_IC_CTRL]);

    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_GD_HB_CTRL,
                               &drv8000_reg_map[REGID_GD_HB_CTRL]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_HS_EC_HEAT_CTRL,
                               &drv8000_reg_map[REGID_HS_EC_HEAT_CTRL]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_OUT7_PWM_DC,
                               &drv8000_reg_map[REGID_OUT7_PWM_DC]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_OUT8_PWM_DC,
                               &drv8000_reg_map[REGID_OUT8_PWM_DC]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_OUT9_PWM_DC,
                               &drv8000_reg_map[REGID_OUT9_PWM_DC]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_OUT10_PWM_DC,
                               &drv8000_reg_map[REGID_OUT10_PWM_DC]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_OUT11_PWM_DC,
                               &drv8000_reg_map[REGID_OUT11_PWM_DC]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_read(interface,
                               DRV8000_ADDREG_OUT12_PWM_DC,
                               &drv8000_reg_map[REGID_OUT12_PWM_DC]);
    }

    return ret;
}

uint8_t drv8000_read_registers(st_DRV8000_Interface_t* interface)
{
    uint8_t ret = 0u;

    /* read device registers */
    ret = drv8000_read_status_registers(interface);
    
    if (0u == ret)
    {
        ret = drv8000_read_config_registers(interface);
    }
    if (0u == ret)
    {
        ret = drv8000_read_control_registers(interface);
    }

    return ret;
}

uint8_t drv8000_read_devid(st_DRV8000_Interface_t* interface)
{
    un_DRV8000_Reg_t reg_val;

    (void)drv8000_spi_read(interface,
                           DRV8000_ADDREG_DEVICE_ID,
                           &reg_val);

    return (uint8_t)(reg_val.u16_RegWord & DRV8000_DEVICE_ID_MASK);
}

/* *** SPI Control *** */
uint8_t drv8000_clear_fault(st_DRV8000_Interface_t* interface)
{
    drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CLR_FLT = 1u;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_IC_CTRL,
                             drv8000_reg_map[REGID_IC_CTRL]);
}

uint8_t drv8000_cfg_reg_lock_unlock(st_DRV8000_Interface_t* interface,
                                    en_LOCK_UNLOCK_REG_WRITE_t reg_lock_unlock)
{
    drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CNFG_LOCK = reg_lock_unlock;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_IC_CTRL,
                             drv8000_reg_map[REGID_IC_CTRL]);
    
}

uint8_t drv8000_ctrl_reg_lock_unlock(st_DRV8000_Interface_t* interface,
                                     en_LOCK_UNLOCK_REG_WRITE_t reg_lock_unlock)
{
    drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CTRL_LOCK = reg_lock_unlock;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_IC_CTRL,
                             drv8000_reg_map[REGID_IC_CTRL]);
}

uint8_t drv8000_ipropi_mode(st_DRV8000_Interface_t* interface,
                            en_IPROPI_MODE_t ipropi_mode,
                            en_IPROPI_SEL_MUX_t ipropi_sel)
{
    drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.IPROPI_MODE = ipropi_mode;
    
    if (IPROPI_INPUT_PWM == ipropi_mode)
    {
        drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.IPROPI_SEL = IPROPI_NO_OUT;
    } else
    {
        drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.IPROPI_SEL = ipropi_sel;
    }

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_IC_CTRL,
                             drv8000_reg_map[REGID_IC_CTRL]);
}

/* ** High Side Driver Control ** */
uint8_t drv8000_hs_driver_cnfg(st_DRV8000_Interface_t* interface,
                               en_HS_CNFG_t hs_out7_cnfg,
                               en_HS_CNFG_t hs_out8_cnfg,
                               en_HS_CNFG_t hs_out9_cnfg,
                               en_HS_CNFG_t hs_out10_cnfg,
                               en_HS_CNFG_t hs_out11_cnfg,
                               en_HS_CNFG_t hs_out12_cnfg)
{
    if (DRV8000_REG_NOT_CHANGE != hs_out7_cnfg)
    {
        drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT7_CNFG = hs_out7_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out8_cnfg)
    {
        drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT8_CNFG = hs_out8_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out9_cnfg)
    {
        drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT9_CNFG = hs_out9_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out10_cnfg)
    {
        drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT10_CNFG = hs_out10_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out11_cnfg)
    {
        drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT11_CNFG = hs_out11_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out12_cnfg)
    {
        drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT12_CNFG = hs_out12_cnfg;
    }

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_HEAT_OUT_CNFG,
                             drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG]);
}

uint8_t drv8000_hs_driver_spi_enable(st_DRV8000_Interface_t* interface,
                                     en_HS_EN_t hs_out7_en,
                                     en_HS_EN_t hs_out8_en,
                                     en_HS_EN_t hs_out9_en,
                                     en_HS_EN_t hs_out10_en,
                                     en_HS_EN_t hs_out11_en,
                                     en_HS_EN_t hs_out12_en)
{
    if (DRV8000_REG_NOT_CHANGE != hs_out7_en)
    {
        drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT7_EN = hs_out7_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out8_en)
    {
        drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT8_EN = hs_out8_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out9_en)
    {
        drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT9_EN = hs_out9_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out10_en)
    {
        drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT10_EN = hs_out10_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out11_en)
    {
        drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT11_EN = hs_out11_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hs_out12_en)
    {
        drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT12_EN = hs_out12_en;
    }

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_EC_HEAT_CTRL,
                             drv8000_reg_map[REGID_HS_EC_HEAT_CTRL]);
}

uint8_t drv8000_hs_set_gen_pwm_dutycycle(st_DRV8000_Interface_t* interface,
                                         en_HS_OUTx_t hs_outx,
                                         uint16_t dutycycle)
{
    uint8_t ret = 0u;
    uint8_t reg_id;
    uint8_t reg_addr;

    dutycycle = dutycycle & DRV8000_GEN_PWM_DC_MASK; /* 10 bits only */

    switch (hs_outx)
    {
        case HS_OUT_12:
            reg_id = REGID_OUT12_PWM_DC;
            reg_addr = DRV8000_ADDREG_OUT12_PWM_DC;
            drv8000_reg_map[reg_id].Reg_OUT12_PWM_DC.OUT12_DC = dutycycle;
            break;
        case HS_OUT_11:
            reg_id = REGID_OUT11_PWM_DC;
            reg_addr = DRV8000_ADDREG_OUT11_PWM_DC;
            drv8000_reg_map[reg_id].Reg_OUT11_PWM_DC.OUT11_DC = dutycycle;
            break;
        case HS_OUT_10:
            reg_id = REGID_OUT10_PWM_DC;
            reg_addr = DRV8000_ADDREG_OUT10_PWM_DC;
            drv8000_reg_map[reg_id].Reg_OUT10_PWM_DC.OUT10_DC = dutycycle;
            break;
        case HS_OUT_9:
            reg_id = REGID_OUT9_PWM_DC;
            reg_addr = DRV8000_ADDREG_OUT9_PWM_DC;
            drv8000_reg_map[reg_id].Reg_OUT9_PWM_DC.OUT9_DC = dutycycle;
            break;
        case HS_OUT_8:
            reg_id = REGID_OUT8_PWM_DC;
            reg_addr = DRV8000_ADDREG_OUT8_PWM_DC;
            drv8000_reg_map[reg_id].Reg_OUT8_PWM_DC.OUT8_DC = dutycycle;
            break;
        case HS_OUT_7:
            reg_id = REGID_OUT7_PWM_DC;
            reg_addr = DRV8000_ADDREG_OUT7_PWM_DC;
            drv8000_reg_map[reg_id].Reg_OUT7_PWM_DC.OUT7_DC = dutycycle;
            break;
        default:
            ret = 1u;
            break;
    }

    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                reg_addr,
                                drv8000_reg_map[reg_id]);
    }

    return ret;
}

uint8_t drv8000_hs_set_gen_pwm_freq(st_DRV8000_Interface_t* interface,
                                    en_HS_GEN_PWM_FREQ_t hs_out7_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out8_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out9_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out10_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out11_pwm_freq,
                                    en_HS_GEN_PWM_FREQ_t hs_out12_pwm_freq)
{
    drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT7_FREQ = hs_out7_pwm_freq;
    drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT8_FREQ = hs_out8_pwm_freq;
    drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT9_FREQ = hs_out9_pwm_freq;
    drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT10_FREQ = hs_out10_pwm_freq;
    drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT11_FREQ = hs_out11_pwm_freq;
    drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT12_FREQ = hs_out12_pwm_freq;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_PWM_FREQ_CNFG,
                             drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG]);
}

/**
 * @brief Enables or disables constant current mode (CCM) for high-side driver outputs
 * 
 * Enables or disables a constant current for a short duration to the desired high-side outputs.
 * 
 * @note Short circuit and over current detection are disabled during constant current mode
 * @note CCM has to be enabled before enabling HS driver
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
                              en_HS_CCM_EN_t hs_out12_ccm_en)
{
    drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT7_CCM_EN = hs_out7_ccm_en;
    drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT8_CCM_EN = hs_out8_ccm_en;
    drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT9_CCM_EN = hs_out9_ccm_en;
    drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT10_CCM_EN = hs_out10_ccm_en;
    drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT11_CCM_EN = hs_out11_ccm_en;
    drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT12_CCM_EN = hs_out12_ccm_en;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_REG_CNFG2,
                             drv8000_reg_map[REGID_HS_REG_CNFG2]);
}

/* ** Heater Driver Control ** */
uint8_t drv8000_heater_driver_cnfg(st_DRV8000_Interface_t* interface,
                                   en_HEATER_CNFG_t hs_heater_cnfg)
{
    if (DRV8000_REG_NOT_CHANGE == hs_heater_cnfg)
    {
        return 1u;
    } else
    {
        drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.HEAT_OUT_CNFG = hs_heater_cnfg;

        return drv8000_spi_write(interface,
                                 DRV8000_ADDREG_HS_HEAT_OUT_CNFG,
                                 drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG]);
    }
}

uint8_t drv8000_heater_driver_enable(st_DRV8000_Interface_t* interface,
                                     en_HS_EN_t hs_heater_en)
{
    drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.HEAT_EN = hs_heater_en;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_EC_HEAT_CTRL,
                             drv8000_reg_map[REGID_HS_EC_HEAT_CTRL]);
}

/* ** Electrochromic Driver Control ** */
uint8_t drv8000_ec_driver_enable(st_DRV8000_Interface_t* interface,
                                 en_HS_EN_t hs_ec_en,
                                 en_EC_ECFB_LS_EN_t es_ecfb_ls_en,
                                 uint8_t ec_v_tar)
{
    drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.EC_ON = hs_ec_en;
    drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.ECFB_LS_EN = es_ecfb_ls_en;
    drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.EC_V_TAR = (ec_v_tar & DRV8000_EC_V_TAR_MASK); /* 6 bits */

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_EC_HEAT_CTRL,
                             drv8000_reg_map[REGID_HS_EC_HEAT_CTRL]);
}

/* *** Half-bridge Control *** */
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
                                     en_HHB_CNFG_t hhb_out4_cnfg)
{
    if (DRV8000_REG_NOT_CHANGE != hhb_out1_cnfg)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG2].Reg_HB_OUT_CNFG2.OUT1_CNFG = hhb_out1_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out2_cnfg)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG2].Reg_HB_OUT_CNFG2.OUT2_CNFG = hhb_out2_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out3_cnfg)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG2].Reg_HB_OUT_CNFG2.OUT3_CNFG = hhb_out3_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out4_cnfg)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG2].Reg_HB_OUT_CNFG2.OUT4_CNFG = hhb_out4_cnfg;
    }

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HB_OUT_CNFG2,
                             drv8000_reg_map[REGID_HB_OUT_CNFG2]);
}

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
                                   en_HHB_CNFG_t hhb_out6_cnfg)
{
    if (DRV8000_REG_NOT_CHANGE != hhb_out5_cnfg)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG1].Reg_HB_OUT_CNFG1.OUT5_CNFG = hhb_out5_cnfg;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out6_cnfg)
    {
        drv8000_reg_map[REGID_HB_OUT_CNFG1].Reg_HB_OUT_CNFG1.OUT6_CNFG = hhb_out6_cnfg;
    }

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HB_OUT_CNFG1,
                             drv8000_reg_map[REGID_HB_OUT_CNFG1]);
}

uint8_t drv8000_hhb_spi_enable(st_DRV8000_Interface_t* interface,
                               en_HHB_EN_t hhb_out1_en,
                               en_HHB_EN_t hhb_out2_en,
                               en_HHB_EN_t hhb_out3_en,
                               en_HHB_EN_t hhb_out4_en,
                               en_HHB_EN_t hhb_out5_en,
                               en_HHB_EN_t hhb_out6_en)
{
    if (DRV8000_REG_NOT_CHANGE != hhb_out1_en)
    {
        drv8000_reg_map[REGID_GD_HB_CTRL].Reg_GD_HB_CTRL.OUT1_CTRL = hhb_out1_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out2_en)
    {
        drv8000_reg_map[REGID_GD_HB_CTRL].Reg_GD_HB_CTRL.OUT2_CTRL = hhb_out2_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out3_en)
    {
        drv8000_reg_map[REGID_GD_HB_CTRL].Reg_GD_HB_CTRL.OUT3_CTRL = hhb_out3_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out4_en)
    {
        drv8000_reg_map[REGID_GD_HB_CTRL].Reg_GD_HB_CTRL.OUT4_CTRL = hhb_out4_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out5_en)
    {
        drv8000_reg_map[REGID_GD_HB_CTRL].Reg_GD_HB_CTRL.OUT5_CTRL = hhb_out5_en;
    }
    if (DRV8000_REG_NOT_CHANGE != hhb_out6_en)
    {
        drv8000_reg_map[REGID_GD_HB_CTRL].Reg_GD_HB_CTRL.OUT6_CTRL = hhb_out6_en;
    }

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_GD_HB_CTRL,
                             drv8000_reg_map[REGID_GD_HB_CTRL]);
}

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
                           en_HHB_FW_t hhb_out6_fw)
{
    drv8000_reg_map[REGID_HB_OUT_CNFG1].Reg_HB_OUT_CNFG1.NSR_OUT1_DIS = hhb_out1_fw;
    drv8000_reg_map[REGID_HB_OUT_CNFG1].Reg_HB_OUT_CNFG1.NSR_OUT2_DIS = hhb_out2_fw;
    drv8000_reg_map[REGID_HB_OUT_CNFG1].Reg_HB_OUT_CNFG1.NSR_OUT3_DIS = hhb_out3_fw;
    drv8000_reg_map[REGID_HB_OUT_CNFG1].Reg_HB_OUT_CNFG1.NSR_OUT4_DIS = hhb_out4_fw;
    drv8000_reg_map[REGID_HB_OUT_CNFG1].Reg_HB_OUT_CNFG1.NSR_OUT5_DIS = hhb_out5_fw;
    drv8000_reg_map[REGID_HB_OUT_CNFG1].Reg_HB_OUT_CNFG1.NSR_OUT6_DIS = hhb_out6_fw;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HB_OUT_CNFG1,
                             drv8000_reg_map[REGID_HB_OUT_CNFG1]);
}

/* *** H-bridge Control *** */
#ifdef GDU_GD_USED
uint8_t drv8000_spi_gd_enable(st_DRV8000_Interface_t* interface)
{
    /* Gate driver outputs are enabled and controlled by the digital inputs */
    drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.EN_GD = 1u;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_GD_CNFG,
                             drv8000_reg_map[REGID_GD_CNFG]);
}

uint8_t drv8000_spi_gd_disable(st_DRV8000_Interface_t* interface)
{
    /* Gate driver inputs are ignored and the gate driver passive pull-downs are enabled */
    /* Unlike DRVOFF, changing EN_GD does not pull down the gate driver half-bridges to Hi-Z */
    drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.EN_GD = 0u;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_GD_CNFG,
                             drv8000_reg_map[REGID_GD_CNFG]);
}

uint8_t drv8000_gd_hb_set_mode(st_DRV8000_Interface_t* interface,
                               en_GD_BRG_MODE_t brg_mode,
                               en_GD_INx_MODE_t in1_spi_mode,
                               en_GD_INx_MODE_t in2_spi_mode,
                               en_GD_FBRG_FW_MODE_t fw_mode)
{
    uint8_t ret = 0u;

    switch (brg_mode)
    {
        case GD_BRG_MODE_HALF_BRG:
            (void)fw_mode;
            drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.IN1_MODE = in1_spi_mode;
            drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.IN2_MODE = in2_spi_mode;
            break;
#ifdef GDU_GD_IN2_GPIO
        case GD_BRG_MODE_PH_EN:
            /* Cannot have both as SPI pin during this mode */
            if (GD_INx_SPI == in1_spi_mode && GD_INx_SPI == in2_spi_mode)
            {
                ret = 1u;
            } else
            {
                drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.BRG_FW = fw_mode;
                drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.IN1_MODE = in1_spi_mode;
                drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.IN2_MODE = in2_spi_mode;
            }

            break;
#else /* not defined GDU_GD_IN2_GPIO */
        case GD_BRG_MODE_PWM:
            (void)in1_spi_mode;
            (void)in2_spi_mode;
            drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.BRG_FW = fw_mode;
            break;
#endif /* GDU_GD_IN2_GPIO */
        default:
            ret = 1u;
            break;
    }

    if (0u == ret)
    {
        drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.BRG_MODE = brg_mode;

        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_GD_CNFG,
                                drv8000_reg_map[REGID_GD_CNFG]);
    }

    return ret;
}

uint8_t drv8000_gd_hb_set_direction(st_DRV8000_Interface_t* interface,
                                    en_GD_FBRG_DIRECTION_t direction)
{
    uint8_t ret = 0u;

    /* GD_BRG_MODE_HALF_BRG does not have direction and GD_BRG_MODE_PWM uses PWM to control direction */
    if (GD_BRG_MODE_PH_EN != drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.BRG_MODE
        ||
        /* If IN1 uses SPI, means that IN2 is control direction using PWM */
        GD_INx_SPI == drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.IN2_MODE)
    {
        ret = 1u;
    } else
    {
        if (GD_INx_SPI == drv8000_reg_map[REGID_GD_CNFG].Reg_GD_CNFG.IN2_MODE)
        {
            drv8000_reg_map[REGID_GD_HB_CTRL].Reg_GD_HB_CTRL.S_IN2 = direction;

            ret = drv8000_spi_write(interface,
                                    DRV8000_ADDREG_GD_HB_CTRL,
                                    drv8000_reg_map[REGID_GD_HB_CTRL]);
        } else
        {
#ifdef GDU_GD_IN2_GPIO
            ret = drv8000_gpio_set(interface,
                                   interface->gd_in2_port,
                                   interface->gd_in2_pin,
                                   (uint8_t)direction);
#else /* not defined GDU_GD_IN2_GPIO */
            ret = 1u;
#endif /* GDU_GD_IN2_GPIO */
        }
    }

    return ret;
}
#endif /* GDU_GD_USED */


/* **********************************************************************/
/* ***             Definition of local functions                      ***/
/* **********************************************************************/
uint8_t drv8000_spi_status_check(void)
{
    if (DRV8000_SUCCESS_SPI_STATUS != drv8000_global_spi_status.u8_SpiCommStatus)
    {
        return 1u;
    }

    return 0u;
}

un_DRV8000_SPI_STST_t drv8000_spi_status_get(void)
{
    return drv8000_global_spi_status;
}

uint8_t drv8000_spi_read(st_DRV8000_Interface_t* interface,
                         const uint8_t reg_addr,
                         un_DRV8000_Reg_t* reg_val)
{
    if (NULL == interface || NULL == interface->fptr_SpiTransceive)
    {
        return 1u;
    }

    uint8_t ret = 0u;
    un_DRV8000_SDI_FRAME_t spi_transmit;
    un_DRV8000_SDO_FRAME_t spi_receive;

    spi_transmit.st_SpiCommand.Ignored = 0u;
    spi_transmit.st_SpiCommand.MSBBit = 0u;
    spi_transmit.st_SpiCommand.DataField = 0u;
    spi_transmit.st_SpiCommand.AccessType = DRV8000_READ_ACCESS;
    spi_transmit.st_SpiCommand.Address = reg_addr & DRV8000_REG_ADDRESS_MASK;

    ret = interface->fptr_SpiTransceive((uint8_t*)&spi_transmit,
                                        (uint8_t*)&spi_receive,
                                        (uint16_t)DRV8000_SPI_FRAME_LEN);

    if (0u == ret)
    {
        reg_val->u16_RegWord = spi_receive.st_SpiCommand.DataField;
        drv8000_global_spi_status.u8_SpiCommStatus = spi_receive.st_SpiCommand.SpiStastus & DRV8000_GLOBAL_STATUS_MASK;
        ret = drv8000_spi_status_check();
    } else 
    {
        reg_val->u16_RegWord = 0u;
    }

    return ret;
}

uint8_t drv8000_spi_write(st_DRV8000_Interface_t* interface,
                          const uint8_t reg_addr,
                          const un_DRV8000_Reg_t reg_val)
{
    if (NULL == interface || NULL == interface->fptr_SpiTransceive)
    {
        return 1u;
    }
    if (LOCK_REG_WRITE == drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CTRL_LOCK)
    {
        /* Control register locked, cannot write (except to IC_CTRL) */
        if (reg_addr > DRV8000_ADDREG_IC_CTRL)
        {
            return 1u;
        }
    }
    if (LOCK_REG_WRITE == drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CNFG_LOCK)
    {
        /* Config register locked, cannot write */
        if (reg_addr < DRV8000_ADDREG_IC_CTRL)
        {
            return 1u;
        }
    }

    uint8_t ret = 0u;
    un_DRV8000_SDI_FRAME_t spi_transmit;
    un_DRV8000_SDO_FRAME_t spi_receive;

    spi_transmit.st_SpiCommand.Ignored = 0u;
    spi_transmit.st_SpiCommand.MSBBit = 0u;
    spi_transmit.st_SpiCommand.AccessType = DRV8000_WRITE_ACCESS;
    spi_transmit.st_SpiCommand.Address = reg_addr & DRV8000_REG_ADDRESS_MASK;
    spi_transmit.st_SpiCommand.DataField = reg_val.u16_RegWord & DRV8000_REG_DATA_MASK;

    ret = interface->fptr_SpiTransceive((uint8_t*)&spi_transmit,
                                        (uint8_t*)&spi_receive,
                                        (uint16_t)DRV8000_SPI_FRAME_LEN);

    if (0u == ret)
    {
        drv8000_global_spi_status.u8_SpiCommStatus = spi_receive.st_SpiCommand.SpiStastus & DRV8000_GLOBAL_STATUS_MASK;
        ret = drv8000_spi_status_check();
    }

    return ret;
}

uint8_t drv8000_gpio_set(st_DRV8000_Interface_t* interface,
                         uint8_t port,
                         uint8_t pin,
                         uint8_t pin_level)
{
    if (NULL == interface || NULL == interface->fptr_Gpio)
    {
        return 1u;
    } else
    {
        return interface->fptr_Gpio(port,
                                    pin,
                                    pin_level);
    }
}

uint8_t drv8000_pwm_set(st_DRV8000_Interface_t* interface,
                             uint8_t instance,
                             uint8_t channel,
                             uint16_t dutycycle
#ifndef GDU_PWM_PERIOD_FIXED
                            ,uint16_t period
#endif /* GDU_PWM_PERIOD_FIXED */
                            )
{
    if (NULL == interface || NULL == interface->fptr_PwmSetDutycycle)
    {
        return 1u;
    }
#ifndef GDU_PWM_PERIOD_FIXED
    if (NULL == interface->fptr_PwmSetPeriod)
    {
        return 1u;
    }
    if (period > interface->pwm_max_period)
    {
        period = interface->pwm_max_period;
    }
    if (dutycycle > period)
    {
        dutycycle = period;
    }

    interface->fptr_PwmSetPeriod(instance,
                                 channel,
                                 period);
#else /* defined GDU_PWM_PERIOD_FIXED */
    if (dutycycle > interface->pwm_max_period)
    {
        dutycycle = interface->pwm_max_period;
    }
#endif /* GDU_PWM_PERIOD_FIXED */
    return interface->fptr_PwmSetDutycycle(instance,
                                           channel,
                                           dutycycle);
}

uint8_t drv8000_delay_set(st_DRV8000_Interface_t* interface,
                          uint16_t delay_us)
{
    if (NULL != interface && NULL != interface->fptr_Delay)
    {
        interface->fptr_Delay(delay_us);
        return 0u;
    }

    return 1u;
}