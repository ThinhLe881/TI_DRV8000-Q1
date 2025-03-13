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
#ifdef GDU_PWM_PERIOD_NOT_FIXED
                            ,uint16_t period
#endif
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

uint8_t drv8000_pin_set_pwm1(st_DRV8000_Interface_t* interface,
                             uint16_t dutycycle
#ifdef GDU_PWM_PERIOD_NOT_FIXED
                            ,uint16_t period
#endif
                            )
{
    return drv8000_pwm_set(interface,
                                interface->pwm1_instance,
                                interface->pwm1_channel,
                                dutycycle
#ifdef GDU_PWM_PERIOD_NOT_FIXED
                                ,period
#endif
                            );
}

uint8_t drv8000_pin_set_gd_in1(st_DRV8000_Interface_t* interface,
                               uint16_t dutycycle
#ifdef GDU_PWM_PERIOD_NOT_FIXED
                              ,uint16_t period
#endif
                              )
{
    return drv8000_pwm_set(interface,
                                interface->pwm_gd_in1_instance,
                                interface->pwm_gd_in1_channel,
                                dutycycle
#ifdef GDU_PWM_PERIOD_NOT_FIXED
                                ,period
#endif
                            );
}

#ifndef GDU_GD_IN2_GPIO
uint8_t drv8000_pin_set_gd_in2(st_DRV8000_Interface_t* interface,
                               uint16_t dutycycle
#ifdef GDU_PWM_PERIOD_NOT_FIXED
                              ,uint16_t period
#endif
                              )
{
    return drv8000_pwm_set(interface,
                                interface->pwm_gd_in2_instance,
                                interface->pwm_gd_in2_channel,
                                dutycycle
#ifdef GDU_PWM_PERIOD_NOT_FIXED
                                ,period
#endif
                            );
}
#endif

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

uint8_t drv8000_cfg_reg_lock(st_DRV8000_Interface_t* interface,
                             en_LOCK_REG_WRITE_t reg_lock)
{
    drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CNFG_LOCK = reg_lock;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_IC_CTRL,
                             drv8000_reg_map[REGID_IC_CTRL]);
}

uint8_t drv8000_ctrl_reg_lock(st_DRV8000_Interface_t* interface,
                              en_LOCK_REG_WRITE_t reg_lock)
{
    drv8000_reg_map[REGID_IC_CTRL].Reg_IC_CTRL.CTRL_LOCK = reg_lock;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_IC_CTRL,
                             drv8000_reg_map[REGID_IC_CTRL]);
}

/* ** High Side Driver Control ** */
uint8_t drv8000_hs_driver_cnfg(st_DRV8000_Interface_t* interface,
                               en_HS_OUTx_t hs_outx,
                               en_HS_CNFG_t hs_out_cnfg)
{
    uint8_t ret = 0u;

    switch (hs_outx)
    {
        case HS_OUT_12:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT12_CNFG = hs_out_cnfg;
            break;
        case HS_OUT_11:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT11_CNFG = hs_out_cnfg;
            break;
        case HS_OUT_10:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT10_CNFG = hs_out_cnfg;
            break;
        case HS_OUT_9:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT9_CNFG = hs_out_cnfg;
            break;
        case HS_OUT_8:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT8_CNFG = hs_out_cnfg;
            break;
        case HS_OUT_7:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT7_CNFG = hs_out_cnfg;
            break;
        default:
            ret = 1u;
            break;
    }

    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_HEAT_OUT_CNFG,
                                drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG]);
    }

    return ret;
}

uint8_t drv8000_hs_driver_spi(st_DRV8000_Interface_t* interface,
                              en_HS_OUTx_t hs_outx,
                              en_HS_SPI_EN_t hs_spi_en)
{
    uint8_t ret = 0u;

    switch (hs_outx)
    {
        case HS_OUT_12:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT12_CNFG = HS_CNFG_SPI_CONTROL;
            drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT12_EN = hs_spi_en;
            break;
        case HS_OUT_11:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT11_CNFG = HS_CNFG_SPI_CONTROL;
            drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT11_EN = hs_spi_en;
            break;
        case HS_OUT_10:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT10_CNFG = HS_CNFG_SPI_CONTROL;
            drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT10_EN = hs_spi_en;
            break;
        case HS_OUT_9:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT9_CNFG = HS_CNFG_SPI_CONTROL;
            drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT9_EN = hs_spi_en;
            break;
        case HS_OUT_8:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT8_CNFG = HS_CNFG_SPI_CONTROL;
            drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT8_EN = hs_spi_en;
            break;
        case HS_OUT_7:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT7_CNFG = HS_CNFG_SPI_CONTROL;
            drv8000_reg_map[REGID_HS_EC_HEAT_CTRL].Reg_HS_EC_HEAT_CTRL.OUT7_EN = hs_spi_en;
            break;
        default:
            ret = 1u;
            break;
    }
    
    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_HEAT_OUT_CNFG,
                                drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_EC_HEAT_CTRL,
                                drv8000_reg_map[REGID_HS_EC_HEAT_CTRL]);
    }

    return ret;
}

uint8_t drv8000_hs_driver_pwm_gen(st_DRV8000_Interface_t* interface,
                                  en_HS_OUTx_t hs_outx,
                                  en_HS_GEN_PWM_FREQ_t freq,
                                  uint16_t dutycycle)
{
    uint8_t ret = 0u;
    uint8_t dc_reg_id;
    uint8_t dc_reg_addr;

    dutycycle = dutycycle & DRV8000_MAX_GEN_PWM_DUTYCYCLE; /* 10-bit only */

    switch (hs_outx)
    {
        case HS_OUT_12:
            dc_reg_id = REGID_OUT12_PWM_DC;
            dc_reg_addr = DRV8000_ADDREG_OUT12_PWM_DC;
            drv8000_reg_map[dc_reg_id].Reg_OUT12_PWM_DC.OUT12_DC = dutycycle;
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT12_FREQ = freq;
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT12_CNFG = HS_CNFG_PWM_GEN;
            break;
        case HS_OUT_11:
            dc_reg_id = REGID_OUT11_PWM_DC;
            dc_reg_addr = DRV8000_ADDREG_OUT11_PWM_DC;
            drv8000_reg_map[dc_reg_id].Reg_OUT11_PWM_DC.OUT11_DC = dutycycle;
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT11_FREQ = freq;
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT11_CNFG = HS_CNFG_PWM_GEN;
            break;
        case HS_OUT_10:
            dc_reg_id = REGID_OUT10_PWM_DC;
            dc_reg_addr = DRV8000_ADDREG_OUT10_PWM_DC;
            drv8000_reg_map[dc_reg_id].Reg_OUT10_PWM_DC.OUT10_DC = dutycycle;
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT10_FREQ = freq;
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT10_CNFG = HS_CNFG_PWM_GEN;
            break;
        case HS_OUT_9:
            dc_reg_id = REGID_OUT9_PWM_DC;
            dc_reg_addr = DRV8000_ADDREG_OUT9_PWM_DC;
            drv8000_reg_map[dc_reg_id].Reg_OUT9_PWM_DC.OUT9_DC = dutycycle;
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT9_FREQ = freq;
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT9_CNFG = HS_CNFG_PWM_GEN;
            break;
        case HS_OUT_8:
            dc_reg_id = REGID_OUT8_PWM_DC;
            dc_reg_addr = DRV8000_ADDREG_OUT8_PWM_DC;
            drv8000_reg_map[dc_reg_id].Reg_OUT8_PWM_DC.OUT8_DC = dutycycle;
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT8_FREQ = freq;
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT8_CNFG = HS_CNFG_PWM_GEN;
            break;
        case HS_OUT_7:
            dc_reg_id = REGID_OUT7_PWM_DC;
            dc_reg_addr = DRV8000_ADDREG_OUT7_PWM_DC;
            drv8000_reg_map[dc_reg_id].Reg_OUT7_PWM_DC.OUT7_DC = dutycycle;
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT7_FREQ = freq;
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT7_CNFG = HS_CNFG_PWM_GEN;
            break;
        default:
            ret = 1u;
            break;
    }

    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                dc_reg_addr,
                                drv8000_reg_map[dc_reg_id]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_PWM_FREQ_CNFG,
                                drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_HEAT_OUT_CNFG,
                                drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG]);
    }
    
    return ret;
}

uint8_t drv8000_hs_set_gen_pwm_freq(st_DRV8000_Interface_t* interface,
                                    en_HS_OUTx_t hs_outx,
                                    en_HS_GEN_PWM_FREQ_t freq)
{
    uint8_t ret = 0u;

    switch (hs_outx)
    {
        case HS_OUT_12:
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT12_FREQ = freq;
            break;
        case HS_OUT_11:
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT11_FREQ = freq;
            break;
        case HS_OUT_10:
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT10_FREQ = freq;
            break;
        case HS_OUT_9:
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT9_FREQ = freq;
            break;
        case HS_OUT_8:
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT8_FREQ = freq;
            break;
        case HS_OUT_7:
            drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG].Reg_HS_PWM_FREQ.PWM_OUT7_FREQ = freq;
            break;
        default:
            ret = 1u;
            break;
    }

    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_PWM_FREQ_CNFG,
                                drv8000_reg_map[REGID_HS_PWM_FREQ_CNFG]);
    }

    return ret;
}

uint8_t drv8000_hs_set_gen_pwm_dutycycle(st_DRV8000_Interface_t* interface,
                                         en_HS_OUTx_t hs_outx,
                                         uint16_t dutycycle)
{
    uint8_t ret = 0u;
    uint8_t reg_id;
    uint8_t reg_addr;

    dutycycle = dutycycle & DRV8000_MAX_GEN_PWM_DUTYCYCLE; /* 10-bit only */

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

uint8_t drv8000_hs_set_ccm(st_DRV8000_Interface_t* interface,
                           en_HS_OUTx_t hs_outx,
                           en_HS_CCM_TO_t hs_ccm)
{
    /* Note: Short circuit and over current detection are disabled during constant current mode */
    uint8_t ret = 0u;
    /* CCM has to be configured before enabling HS driver */
    switch (hs_outx)
    {
        case HS_OUT_12:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT12_CNFG = HS_CNFG_DISABLED;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT12_CCM_TO = hs_ccm;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT12_CCM_EN = 1u;
            break;
        case HS_OUT_11:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT11_CNFG = HS_CNFG_DISABLED;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT11_CCM_TO = hs_ccm;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT11_CCM_EN = 1u;
            break;
        case HS_OUT_10:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT10_CNFG = HS_CNFG_DISABLED;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT10_CCM_TO = hs_ccm;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT10_CCM_EN = 1u;
            break;
        case HS_OUT_9:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT9_CNFG = HS_CNFG_DISABLED;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT9_CCM_TO = hs_ccm;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT9_CCM_EN = 1u;
            break;
        case HS_OUT_8:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT8_CNFG = HS_CNFG_DISABLED;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT8_CCM_TO = hs_ccm;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT8_CCM_EN = 1u;
            break;
        case HS_OUT_7:
            drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG].Reg_HS_HEAT_OUT_CNFG.OUT7_CNFG = HS_CNFG_DISABLED;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT7_CCM_TO = hs_ccm;
            drv8000_reg_map[REGID_HS_REG_CNFG2].Reg_HS_REG_CNFG2.OUT7_CCM_EN = 1u;
            break;
        default:
            ret = 1u;
            break;
    }
    /* Disable HS driver before configuring CCM */
    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_HEAT_OUT_CNFG,
                                drv8000_reg_map[REGID_HS_HEAT_OUT_CNFG]);
    }
    /* Configure CCM */
    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_REG_CNFG2,
                                drv8000_reg_map[REGID_HS_REG_CNFG2]);
    }

    return ret;
}

uint8_t drv8000_hs_set_out7_itrip(st_DRV8000_Interface_t* interface,
                                  en_HS_OUT7_ITRIP_CNFG_t cnfg,
                                  en_HS_OUT7_ITRIP_TIMEOUT_t timeout,
                                  en_HS_OUT7_ITRIP_BLK_t blanking_time,
                                  en_HS_OUT7_ITRIP_FREQ_t regulation_freq,
                                  en_HS_OUT7_ITRIP_DG_t deglitch_time)
{
    /* Must disable OUT7 over current protection for ITRIP to work */
    drv8000_reg_map[REGID_HS_REG_CNFG1].Reg_HS_REG_CNFG1.OUT7_OCP_DIS = HS_OUT7_OCP_DISABLE;
    /* Configure ITRIP */
    drv8000_reg_map[REGID_HS_REG_CNFG1].Reg_HS_REG_CNFG1.ITRIP_TO_SEL = timeout;
    drv8000_reg_map[REGID_HS_REG_CNFG1].Reg_HS_REG_CNFG1.OUT7_ITRIP_CNFG = cnfg;
    drv8000_reg_map[REGID_HS_REG_CNFG1].Reg_HS_REG_CNFG1.OUT7_ITRIP_BLK = blanking_time;
    drv8000_reg_map[REGID_HS_REG_CNFG1].Reg_HS_REG_CNFG1.OUT7_ITRIP_FREQ = regulation_freq;
    drv8000_reg_map[REGID_HS_REG_CNFG1].Reg_HS_REG_CNFG1.OUT7_ITRIP_DG = deglitch_time;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_REG_CNFG1,
                             drv8000_reg_map[REGID_HS_REG_CNFG1]);
}

uint8_t drv8000_hs_set_out7_ocp_enable(st_DRV8000_Interface_t* interface)
{
    /* Remove all ITRIP configs and enable OCP */
    drv8000_reg_map[REGID_HS_REG_CNFG1].u16_RegWord = 0u;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_REG_CNFG1,
                             drv8000_reg_map[REGID_HS_REG_CNFG1]);
}

uint8_t drv8000_hs_set_out7_ocp_disable(st_DRV8000_Interface_t* interface)
{
    /* Remove all ITRIP configs and disable OCP */
    drv8000_reg_map[REGID_HS_REG_CNFG1].u16_RegWord = 0u;
    drv8000_reg_map[REGID_HS_REG_CNFG1].Reg_HS_REG_CNFG1.OUT7_OCP_DIS = HS_OUT7_OCP_DISABLE;

    return drv8000_spi_write(interface,
                             DRV8000_ADDREG_HS_REG_CNFG1,
                             drv8000_reg_map[REGID_HS_REG_CNFG1]);
}

uint8_t drv8000_hs_ocp_config(st_DRV8000_Interface_t* interface,
                              en_HS_OUTx_t hs_outx,
                              en_HS_OC_TH_t hs_oc_th,
                              en_HS_OCP_DG_t hs_ocp_dg)
{
    uint8_t ret = 0u;

    switch (hs_outx)
    {
        case HS_OUT_12:
            drv8000_reg_map[REGID_HS_OC_CNFG].Reg_HS_OC_CNFG.OUT12_OC_TH = hs_oc_th;
            drv8000_reg_map[REGID_HS_OCP_DG].Reg_HS_OCP_DG.OUT12_OCP_DG = hs_ocp_dg;
            break;
        case HS_OUT_11:
            drv8000_reg_map[REGID_HS_OC_CNFG].Reg_HS_OC_CNFG.OUT11_OC_TH = hs_oc_th;
            drv8000_reg_map[REGID_HS_OCP_DG].Reg_HS_OCP_DG.OUT11_OCP_DG = hs_ocp_dg;
            break;
        case HS_OUT_10:
            drv8000_reg_map[REGID_HS_OC_CNFG].Reg_HS_OC_CNFG.OUT10_OC_TH = hs_oc_th;
            drv8000_reg_map[REGID_HS_OCP_DG].Reg_HS_OCP_DG.OUT10_OCP_DG = hs_ocp_dg;
            break;
        case HS_OUT_9:
            drv8000_reg_map[REGID_HS_OC_CNFG].Reg_HS_OC_CNFG.OUT9_OC_TH = hs_oc_th;
            drv8000_reg_map[REGID_HS_OCP_DG].Reg_HS_OCP_DG.OUT9_OCP_DG = hs_ocp_dg;
            break;
        case HS_OUT_8:
            drv8000_reg_map[REGID_HS_OC_CNFG].Reg_HS_OC_CNFG.OUT8_OC_TH = hs_oc_th;
            drv8000_reg_map[REGID_HS_OCP_DG].Reg_HS_OCP_DG.OUT8_OCP_DG = hs_ocp_dg;
            break;
        case HS_OUT_7:
            drv8000_reg_map[REGID_HS_OC_CNFG].Reg_HS_OC_CNFG.OUT7_RDSON_MODE = hs_oc_th;
            drv8000_reg_map[REGID_HS_OCP_DG].Reg_HS_OCP_DG.OUT7_OCP_DG = hs_ocp_dg;
            break;
        default:
            ret = 1u;
            break;
    }

    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_OC_CNFG,
                                drv8000_reg_map[REGID_HS_OC_CNFG]);
    }
    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_OCP_DG,
                                drv8000_reg_map[REGID_HS_OCP_DG]);
    }

    return ret;
}

uint8_t drv8000_hs_ola_config(st_DRV8000_Interface_t* interface,
                              en_HS_OUTx_t hs_outx,
                              en_HS_OLA_t hs_ola_th)
{
    uint8_t ret = 0u;

    switch (hs_outx)
    {
        case HS_OUT_12:
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_TH = hs_ola_th;
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_EN = 1u;
            break;
        case HS_OUT_11:
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_TH = hs_ola_th;
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_EN = 1u;
            break;
        case HS_OUT_10:
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_TH = hs_ola_th;
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_EN = 1u;
            break;
        case HS_OUT_9:
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_TH = hs_ola_th;
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_EN = 1u;
            break;
        case HS_OUT_8:
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_TH = hs_ola_th;
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_EN = 1u;
            break;
        case HS_OUT_7:
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_TH = hs_ola_th;
            drv8000_reg_map[REGID_HS_OL_CNFG].Reg_HS_OL_CNFG.OUT12_OLA_EN = 1u;
            break;
        default:
            ret = 1u;
            break;
    }

    if (0u == ret)
    {
        ret = drv8000_spi_write(interface,
                                DRV8000_ADDREG_HS_OL_CNFG,
                                drv8000_reg_map[REGID_HS_OL_CNFG]);
    }

    return ret;
}


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
#ifdef GDU_PWM_PERIOD_NOT_FIXED
                            ,uint16_t period
#endif
                            )
{
    if (NULL == interface || NULL == interface->fptr_PwmSetDutycycle)
    {
        return 1u;
    }
#ifdef GDU_PWM_PERIOD_NOT_FIXED
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
#else
    if (dutycycle > interface->pwm_max_period)
    {
        dutycycle = interface->pwm_max_period;
    }
#endif
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