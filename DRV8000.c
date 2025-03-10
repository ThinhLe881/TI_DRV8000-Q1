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


/* **********************************************************************/
/* ***             Definition of local functions                      ***/
/* **********************************************************************/
uint8_t drv8000_check_spi_status(void)
{
    if (DRV8000_SUCCESS_SPI_STATUS != drv8000_global_spi_status.u8_SpiCommStatus)
    {
        return 1;
    }

    return 0;
}

un_DRV8000_SPI_STST_t drv8000_get_spi_status(void)
{
    return drv8000_global_spi_status;
}

uint8_t drv8000_spi_read(st_DRV8000_Interface_t* interface,
                         const uint8_t reg_addr,
                         un_DRV8000_Reg_t* reg_val)
{
    uint8_t ret;

    if (NULL != interface)
    {
        un_DRV8000_SDI_FRAME_t spi_transmit;
        un_DRV8000_SDO_FRAME_t spi_receive;

        spi_transmit.st_SpiCommand.Ignored = 0u;
        spi_transmit.st_SpiCommand.MSBBit = 0u;
        spi_transmit.st_SpiCommand.DataField = 0u;
        spi_transmit.st_SpiCommand.AccessType = DRV8000_READ_ACCESS;
        spi_transmit.st_SpiCommand.Address = reg_addr & DRV8000_REG_ADDRESS_MASK;

        ret = interface->fptr_SpiTransceive((uint8_t*)&spi_transmit, (uint8_t*)&spi_receive, (uint16_t)interface->spi_frame_len);

        if (0u == ret)
        {
            reg_val->u16_RegWord = spi_receive.st_SpiCommand.DataField;
            drv8000_global_spi_status.u8_SpiCommStatus = spi_receive.st_SpiCommand.SpiStastus & DRV8000_GLOBAL_STATUS_MASK;
            ret = drv8000_check_spi_status();
        } else 
        {
            reg_val->u16_RegWord = 0u;
        }

    } else
    {
        ret = 1u;
    }

    return ret;
}

uint8_t drv8000_spi_write(st_DRV8000_Interface_t* interface,
                          const uint8_t reg_addr,
                          const un_DRV8000_Reg_t reg_val)
{
    uint8_t ret;

    if (NULL != interface)
    {
        un_DRV8000_SDI_FRAME_t spi_transmit;
        un_DRV8000_SDO_FRAME_t spi_receive;

        spi_transmit.st_SpiCommand.Ignored = 0u;
        spi_transmit.st_SpiCommand.MSBBit = 0u;
        spi_transmit.st_SpiCommand.AccessType = DRV8000_WRITE_ACCESS;
        spi_transmit.st_SpiCommand.Address = reg_addr & DRV8000_REG_ADDRESS_MASK;
        spi_transmit.st_SpiCommand.DataField = reg_val.u16_RegWord & DRV8000_REG_DATA_MASK;

        ret = interface->fptr_SpiTransceive((uint8_t*)&spi_transmit, (uint8_t*)&spi_receive, (uint16_t)interface->spi_frame_len);

        if (0u == ret)
        {
            drv8000_global_spi_status.u8_SpiCommStatus = spi_receive.st_SpiCommand.SpiStastus & DRV8000_GLOBAL_STATUS_MASK;
            ret = drv8000_check_spi_status();
        }
    } else
    {
        ret = 1u;
    }

    return ret;
}