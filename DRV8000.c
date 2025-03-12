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
uint8_t drv8000_check_spi_status(void);

un_DRV8000_SPI_STST_t drv8000_get_spi_status(void);

uint8_t drv8000_spi_read(st_DRV8000_Interface_t* interface,
                         const uint8_t reg_addr,
                         un_DRV8000_Reg_t* reg_val);

uint8_t drv8000_spi_write(st_DRV8000_Interface_t* interface,
                          const uint8_t reg_addr,
                          const un_DRV8000_Reg_t reg_val);


/* **********************************************************************/
/* ***              Definition of global variables                    ***/
/* **********************************************************************/


/* **********************************************************************/
/* ***            Definition of global functions                      ***/
/* **********************************************************************/
/* *** PIN Control *** */
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

uint8_t drv8000_enable_gate_driver(st_DRV8000_Interface_t* interface)
{
    return drv8000_gpio_set(interface,
                            interface->drvoff_port,
                            interface->drvoff_pin,
                            0u);
}

uint8_t drv8000_disable_gate_driver(st_DRV8000_Interface_t* interface)
{
    return drv8000_gpio_set(interface,
                            interface->drvoff_port,
                            interface->drvoff_pin,
                            1u);
}

uint8_t drv8000_set_pwm_pins(st_DRV8000_Interface_t* interface,
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

uint8_t drv8000_reset(st_DRV8000_Interface_t* interface)
{
    uint8_t ret = 0u;

    if (NULL == interface || NULL == interface->fptr_Gpio || NULL == interface->fptr_Delay)
    {
        ret = 1u;
    } else {
        // /* Disable gate driver - pull DRVOFF to high */
        // ret = interface->fptr_Gpio(interface->drvoff_port,
        //                            interface->drvoff_pin,
        //                            1u);

        if (0u == ret)
        {
            /* Sleep DRV8000 */
            ret = interface->fptr_Gpio(interface->nsleep_port,
                                       interface->nsleep_pin,
                                       0u);
        }
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

/* **********************************************************************/
/* ***             Definition of local functions                      ***/
/* **********************************************************************/
uint8_t drv8000_check_spi_status(void)
{
    if (DRV8000_SUCCESS_SPI_STATUS != drv8000_global_spi_status.u8_SpiCommStatus)
    {
        return 1u;
    }

    return 0u;
}

un_DRV8000_SPI_STST_t drv8000_get_spi_status(void)
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
        ret = drv8000_check_spi_status();
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
        ret = drv8000_check_spi_status();
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