
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

#ifndef _VL53LX_PLATFORM_H_
#define _VL53LX_PLATFORM_H_

#include "vl53lx_ll_def.h"
#include "vl53lx_platform_log.h"

#define VL53LX_IPP_API
#include "vl53lx_platform_ipp_imports.h"
#include "vl53lx_platform_user_data.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file   vl53lx_platform.h
 *
 * @brief  All end user OS/platform/application porting
 */



/**
 * @brief  Initialise platform comms.
 *
 * @param[in]   pdev            : pointer to device structure (device handle)
 * @param[in]   comms_type      : selects between I2C and SPI
 * @param[in]   comms_speed_khz : unsigned short containing the I2C speed in kHz
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_CommsInitialise(
	VL53LX_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz);


/**
 * @brief  Close platform comms.
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_CommsClose(
	VL53LX_Dev_t *pdev);


/**
 * @brief Writes the supplied byte buffer to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   pdata     : pointer to uint8_t (byte) buffer containing the data to be written
 * @param[in]   count     : number of bytes in the supplied byte buffer
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WriteMulti(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Reads the requested number of bytes from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to the uint8_t (byte) buffer to store read data
 * @param[in]   count     : number of bytes to read
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_ReadMulti(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Writes a single byte to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint8_t data value to write
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WrByte(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t       data);


/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uin16_t data value write
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WrWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint16_t      data);


/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint32_t data value to write
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WrDWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint32_t      data);



/**
 * @brief  Reads a single byte from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index
 * @param[out]  pdata     : pointer to uint8_t data value
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 *
 */

VL53LX_Error VL53LX_RdByte(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata);


/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint16_t data value
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_RdWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint16_t     *pdata);


/**
 * @brief  Reads a single dword (32-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint32_t data value
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_RdDWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint32_t     *pdata);



/**
 * @brief  Implements a programmable wait in us
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_us   : integer wait in micro seconds
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitUs(
		VL53LX_Dev_t *pdev,
		int32_t       wait_us);


/**
 * @brief  Implements a programmable wait in ms
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_ms   : integer wait in milliseconds
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitMs(
		VL53LX_Dev_t *pdev,
		int32_t       wait_ms);


/**
* @brief Get the frequency of the timer used for ranging results time stamps
*
* @param[out] ptimer_freq_hz : pointer for timer frequency
*
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
*/

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz);

/**
* @brief Get the timer value in units of timer_freq_hz (see VL53LX_get_timestamp_frequency())
*
* @param[out] ptimer_count : pointer for timer count value
*
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
*/

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count);


/**
 * @brief Set the mode of a specified GPIO pin
 *
 * @param  pin - an identifier specifying the pin being modified - defined per platform
 *
 * @param  mode - an identifier specifying the requested mode - defined per platform
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode);


/**
 * @brief Set the value of a specified GPIO pin
 *
 * @param  pin - an identifier specifying the pin being modified - defined per platform
 *
 * @param  value - a value to set on the GPIO pin - typically 0 or 1
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value);


/**
 * @brief Get the value of a specified GPIO pin
 *
 * @param  pin - an identifier specifying the pin being modified - defined per platform
 *
 * @param  pvalue - a value retrieved from the GPIO pin - typically 0 or 1
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue);


/**
 * @brief Sets and clears the XShutdown pin on the Ewok
 *
 * @param  value - the value for xshutdown - 0 = in reset, 1 = operational
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value);


/**
 * @brief Sets and clears the Comms Mode pin (NCS) on the Ewok
 *
 * @param  value - the value for comms select - 0 = I2C, 1 = SPI
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value);


/**
 * @brief Enables and disables the power to the Ewok module
 *
 * @param  value - the state of the power supply - 0 = power off, 1 = power on
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value);

/**
 * @brief Enables callbacks to the supplied funtion pointer when Ewok interrupts ocurr
 *
 * @param  function - a function callback supplies by the caller, for interrupt notification
 * @param  edge_type - falling edge or rising edge interrupt detection
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error  VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type);


/**
 * @brief Disables the callback on Ewok interrupts
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error  VL53LX_GpioInterruptDisable(void);


/*
 * @brief Gets current system tick count in [ms]
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @return  time_ms : current time in [ms]
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GetTickCount(
		VL53LX_Dev_t *pdev,
		uint32_t *ptime_ms);


/**
 * @brief Register "wait for value" polling routine
 *
 * Port of the V2WReg Script function  WaitValueMaskEx()
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @param[in]   timeout_ms    : timeout in [ms]
 * @param[in]   index         : uint16_t register index value
 * @param[in]   value         : value to wait for
 * @param[in]   mask          : mask to be applied before comparison with value
 * @param[in]   poll_delay_ms : polling delay been each read transaction in [ms]
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitValueMaskEx(
		VL53LX_Dev_t *pdev,
		uint32_t      timeout_ms,
		uint16_t      index,
		uint8_t       value,
		uint8_t       mask,
		uint32_t      poll_delay_ms);

#ifdef __cplusplus
}
#endif

#endif

