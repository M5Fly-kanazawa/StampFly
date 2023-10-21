
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */



#include <windows.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53lx_platform.h"
#include <vl53lx_platform_log.h>

#define VL53LX_get_register_name(VL53LX_p_007,VL53LX_p_032) VL53LX_COPYSTRING(VL53LX_p_032, "");

#include "ranging_sensor_comms.h"
#include "power_board_defs.h"


const uint32_t _power_board_in_use = 0;


uint32_t _power_board_extended = 0;


uint8_t global_comms_type = 0;

#define  VL53LX_COMMS_CHUNK_SIZE  56
#define  VL53LX_COMMS_BUFFER_SIZE 64

#define GPIO_INTERRUPT          RS_GPIO62
#define GPIO_POWER_ENABLE       RS_GPIO60
#define GPIO_XSHUTDOWN          RS_GPIO61
#define GPIO_SPI_CHIP_SELECT    RS_GPIO51



#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, \
	level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, \
	VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)


VL53LX_Error VL53LX_CommsInitialise(
	VL53LX_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	char comms_error_string[ERROR_TEXT_LENGTH];

	SUPPRESS_UNUSED_WARNING(pdev);
	SUPPRESS_UNUSED_WARNING(comms_speed_khz);

	global_comms_type = comms_type;

	if(global_comms_type == VL53LX_I2C)
	{
		if ((CP_STATUS)RANGING_SENSOR_COMMS_Init_CCI(0, 0, 0) != CP_STATUS_OK)
		{
			RANGING_SENSOR_COMMS_Get_Error_Text(comms_error_string);
			trace_i2c("VL53LX_CommsInitialise: RANGING_SENSOR_COMMS_Init_CCI() failed\n");
			trace_i2c(comms_error_string);
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}
	}
	else if(global_comms_type == VL53LX_SPI)
	{
		if ((CP_STATUS)RANGING_SENSOR_COMMS_Init_SPI_V2W8(0, 0, 0) != CP_STATUS_OK)
		{
			RANGING_SENSOR_COMMS_Get_Error_Text(comms_error_string);
			trace_i2c("VL53LX_CommsInitialise: RANGING_SENSOR_COMMS_Init_SPI_V2W8() failed\n");
			trace_i2c(comms_error_string);
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}
	}
	else
	{
		trace_i2c("VL53LX_CommsInitialise: Comms must be one of VL53LX_I2C or VL53LX_SPI\n");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	return status;
}


VL53LX_Error VL53LX_CommsClose(
	VL53LX_Dev_t *pdev)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	char comms_error_string[ERROR_TEXT_LENGTH];

	SUPPRESS_UNUSED_WARNING(pdev);

	if(global_comms_type == VL53LX_I2C)
	{
		if((CP_STATUS)RANGING_SENSOR_COMMS_Fini_CCI() != CP_STATUS_OK)
		{
			RANGING_SENSOR_COMMS_Get_Error_Text(comms_error_string);
			trace_i2c("VL53LX_CommsClose: RANGING_SENSOR_COMMS_Fini_CCI() failed\n");
			trace_i2c(comms_error_string);
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}
	}
	else if(global_comms_type == VL53LX_SPI)
	{
		if((CP_STATUS)RANGING_SENSOR_COMMS_Fini_SPI_V2W8() != CP_STATUS_OK)
		{
			RANGING_SENSOR_COMMS_Get_Error_Text(comms_error_string);
			trace_i2c("VL53LX_CommsClose: RANGING_SENSOR_COMMS_Fini_SPI_V2W8() failed\n");
			trace_i2c(comms_error_string);
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}
	}
	else
	{
		trace_i2c("VL53LX_CommsClose: Comms must be one of VL53LX_I2C or VL53LX_SPI\n");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	return status;
}



VL53LX_Error VL53LX_WriteMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     position       = 0;
	uint32_t     data_size      = 0;

	char   comms_error_string[ERROR_TEXT_LENGTH];

	_LOG_STRING_BUFFER(register_name);
	_LOG_STRING_BUFFER(value_as_str);

	if(global_comms_type == VL53LX_I2C)
	{
		for(position=0; position<count; position+=VL53LX_COMMS_CHUNK_SIZE)
		{
			if (count > VL53LX_COMMS_CHUNK_SIZE)
			{
				if((position + VL53LX_COMMS_CHUNK_SIZE) > count)
				{
					data_size = count - position;
				}
				else
				{
					data_size = VL53LX_COMMS_CHUNK_SIZE;
				}
			}
			else
			{
				data_size = count;
			}

			if (status == VL53LX_ERROR_NONE)
			{
				if( RANGING_SENSOR_COMMS_Write_CCI(
								pdev->i2c_slave_address,
								0,
								index+position,
								pdata+position,
								data_size) != 0 )
				{
					status = VL53LX_ERROR_CONTROL_INTERFACE;
				}
			}

#ifdef VL53LX_LOG_ENABLE
			if (status == VL53LX_ERROR_NONE) {
				char* pvalue_as_str;
				uint32_t i;





				pvalue_as_str =  value_as_str;
				for(i = 0 ; i < data_size ; i++)
				{
						sprintf(pvalue_as_str, ", 0x%02X", *(pdata+position+i));
						pvalue_as_str = pvalue_as_str + 6;
				}

				register_name[0] = 0;
				VL53LX_get_register_name(
						index+(uint16_t)position,
						register_name);

				trace_i2c(

						"WriteAutoIncrement(%s%s); // %3u bytes\n",
						register_name,
						value_as_str,
						data_size);
			}
#endif
		}

		if(status != VL53LX_ERROR_NONE)
		{
			RANGING_SENSOR_COMMS_Get_Error_Text(comms_error_string);
			trace_i2c("VL53LX_WriteMulti RANGING_SENSOR_COMMS_Write_CCI() failed\n");
			trace_i2c(comms_error_string);
		}
	}
	else if(global_comms_type == VL53LX_SPI)
	{
		if((CP_STATUS)RANGING_SENSOR_COMMS_Write_SPI_16I(0, 0, index, pdata, count) != CP_STATUS_OK)
		{
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}

		if(status != VL53LX_ERROR_NONE)
		{
			RANGING_SENSOR_COMMS_Get_Error_Text(comms_error_string);
			trace_i2c("VL53LX_WriteMulti RANGING_SENSOR_COMMS_Write_SPI_16I() failed\n");
			trace_i2c(comms_error_string);
		}
	}
	else
	{
		trace_i2c("VL53LX_WriteMulti: Comms must be one of VL53LX_I2C or VL53LX_SPI\n");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	return status;
}


VL53LX_Error VL53LX_ReadMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     position       = 0;
	uint32_t     data_size      = 0;

	char   comms_error_string[ERROR_TEXT_LENGTH];

	_LOG_STRING_BUFFER(register_name);
	_LOG_STRING_BUFFER(value_as_str);

	if(global_comms_type == VL53LX_I2C)
	{
		for(position=0; position<count; position+=VL53LX_COMMS_CHUNK_SIZE)
		{
			if(count > VL53LX_COMMS_CHUNK_SIZE)
			{
				if((position + VL53LX_COMMS_CHUNK_SIZE) > count)
				{
					data_size = count - position;
				}
				else
				{
					data_size = VL53LX_COMMS_CHUNK_SIZE;
				}
			}
			else
				data_size = count;

			if(status == VL53LX_ERROR_NONE)
			{
				if( RANGING_SENSOR_COMMS_Read_CCI(
								pdev->i2c_slave_address,
								0,
								index+position,
								pdata+position,
								data_size) != 0 )
				{
					status = VL53LX_ERROR_CONTROL_INTERFACE;
				}
			}
#ifdef VL53LX_LOG_ENABLE
			if(status == VL53LX_ERROR_NONE)
			{
				char* pvalue_as_str;
				uint32_t i;



				pvalue_as_str =  value_as_str;
				for(i = 0 ; i < data_size ; i++) {
					sprintf(pvalue_as_str, "0x%02X", *(pdata+position+i));
					if (i == 0) {
						pvalue_as_str = pvalue_as_str + 4;
					}
					else {
						pvalue_as_str = pvalue_as_str + 6;
					}
				}

				register_name[0] = 0;
				VL53LX_get_register_name(
						index+(uint16_t)position,
						register_name);

				trace_i2c(

						"ReadAutoIncrement(%s,%3u); // = (%s)\n",
						register_name,
						data_size,
						value_as_str);
			}
#endif
		}

		if(status != VL53LX_ERROR_NONE)
		{
			RANGING_SENSOR_COMMS_Get_Error_Text(comms_error_string);
			trace_i2c("VL53LX_ReadMulti: RANGING_SENSOR_COMMS_Read_CCI() failed\n");
			trace_i2c(comms_error_string);
		}
	}
	else if(global_comms_type == VL53LX_SPI)
	{
		if((CP_STATUS)RANGING_SENSOR_COMMS_Read_SPI_16I(0, 0, index, pdata, count) != CP_STATUS_OK)
		{
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}

		if(status != VL53LX_ERROR_NONE)
		{
			RANGING_SENSOR_COMMS_Get_Error_Text(comms_error_string);
			trace_i2c("VL53LX_ReadMulti: RANGING_SENSOR_COMMS_Read_SPI_16I() failed\n");
			trace_i2c(comms_error_string);
		}
	}
	else
	{
		trace_i2c("VL53LX_ReadMulti: Comms must be one of VL53LX_I2C or VL53LX_SPI\n");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	return status;
}


VL53LX_Error VL53LX_WrByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t       VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];


	buffer[0] = (uint8_t)(VL53LX_p_003);

	status = VL53LX_WriteMulti(pdev, index, buffer, 1);

	return status;
}


VL53LX_Error VL53LX_WrWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];


	buffer[0] = (uint8_t)(VL53LX_p_003 >> 8);
	buffer[1] = (uint8_t)(VL53LX_p_003 &  0x00FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);

	return status;
}


VL53LX_Error VL53LX_WrDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];


	buffer[0] = (uint8_t) (VL53LX_p_003 >> 24);
	buffer[1] = (uint8_t)((VL53LX_p_003 &  0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((VL53LX_p_003 &  0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (VL53LX_p_003 &  0x000000FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);

	return status;
}


VL53LX_Error VL53LX_RdByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];

	status = VL53LX_ReadMulti(pdev, index, buffer, 1);

	*pdata = buffer[0];

	return status;
}


VL53LX_Error VL53LX_RdWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];

	status = VL53LX_ReadMulti(
					pdev,
					index,
					buffer,
					VL53LX_BYTES_PER_WORD);

	*pdata = (uint16_t)(((uint16_t)(buffer[0])<<8) + (uint16_t)buffer[1]);

	return status;
}


VL53LX_Error VL53LX_RdDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];

	status = VL53LX_ReadMulti(
					pdev,
					index,
					buffer,
					VL53LX_BYTES_PER_DWORD);

	*pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

	return status;
}



VL53LX_Error VL53LX_WaitUs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_us)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	float wait_ms = (float)wait_us/1000.0f;
	HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);

	SUPPRESS_UNUSED_WARNING(pdev);


	WaitForSingleObject(hEvent, (DWORD)(wait_ms + 0.5f));

	trace_i2c("WaitUs(%6d);\n", wait_us);

	return status;
}


VL53LX_Error VL53LX_WaitMs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_ms)
{
	return VL53LX_WaitUs(pdev, wait_ms * 1000);
}



VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 0;

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GetTimerFrequency: Freq : %dHz\n", *ptimer_freq_hz);
	return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
	*ptimer_count = 0;

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GetTimerValue: Freq : %dHz\n", *ptimer_count);
	return VL53LX_ERROR_NONE;
}




VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Mode((RS_GPIO_Pin)pin, (RS_GPIO_Mode)mode) != CP_STATUS_OK)
	{
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioSetMode: Status %d. Pin %d, Mode %d\n", status, pin, mode);
	return status;
}


VL53LX_Error  VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value((RS_GPIO_Pin)pin, value) != CP_STATUS_OK)
	{
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioSetValue: Status %d. Pin %d, Mode %d\n", status, pin, value);
	return status;

}


VL53LX_Error  VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	DWORD value = 0;

	if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Get_Value((RS_GPIO_Pin)pin, &value) != CP_STATUS_OK)
	{
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}
	else
	{
		*pvalue = (uint8_t)value;
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioGetValue: Status %d. Pin %d, Mode %d\n", status, pin, *pvalue);
	return status;
}



VL53LX_Error  VL53LX_GpioXshutdown(uint8_t value)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if(status == VL53LX_ERROR_NONE)
	{
		status = VL53LX_GpioSetMode((uint8_t)GPIO_XSHUTDOWN, (uint8_t)GPIO_OutputPP);
	}

	if(status == VL53LX_ERROR_NONE)
	{
		if(value)
		{
			if ((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_XSHUTDOWN, (DWORD)Pin_State_High) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
		else
		{
			if ((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_XSHUTDOWN, (DWORD)Pin_State_Low) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioXShutdown: Status %d. Value %d\n", status, value);
	return status;
}


VL53LX_Error  VL53LX_GpioCommsSelect(uint8_t value)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if(status == VL53LX_ERROR_NONE)
	{
		status = VL53LX_GpioSetMode((uint8_t)GPIO_SPI_CHIP_SELECT, (uint8_t)GPIO_OutputPP);
	}

	if(status == VL53LX_ERROR_NONE)
	{
		if(value)
		{
			if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_SPI_CHIP_SELECT, (DWORD)Pin_State_High) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
		else
		{
			if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_SPI_CHIP_SELECT, (DWORD)Pin_State_Low) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioCommsSelect: Status %d. Value %d\n", status, value);
	return status;
}


VL53LX_Error  VL53LX_GpioPowerEnable(uint8_t value)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	POWER_BOARD_CMD power_cmd;

	if(status == VL53LX_ERROR_NONE)
	{
		status = VL53LX_GpioSetMode((uint8_t)GPIO_POWER_ENABLE, (uint8_t)GPIO_OutputPP);
	}

	if(status == VL53LX_ERROR_NONE)
	{
		if(value)
		{
			if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_POWER_ENABLE, (DWORD)Pin_State_High) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
		else
		{
			if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_POWER_ENABLE, (DWORD)Pin_State_Low) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
	}

	if(status == VL53LX_ERROR_NONE && _power_board_in_use == 1 && value)
	{
		memset(&power_cmd, 0, sizeof(POWER_BOARD_CMD));
		power_cmd.command = ENABLE_DUT_POWER;

		if((CP_STATUS)RANGING_SENSOR_COMMS_Write_System_I2C(
			POWER_BOARD_I2C_ADDRESS, sizeof(POWER_BOARD_CMD), (uint8_t*)&power_cmd) != CP_STATUS_OK)
		{
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioPowerEnable: Status %d. Value %d\n", status, value);
	return status;
}


VL53LX_Error  VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(function);
	SUPPRESS_UNUSED_WARNING(edge_type);

	return status;
}


VL53LX_Error  VL53LX_GpioInterruptDisable(void)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	return status;
}


VL53LX_Error VL53LX_GetTickCount(
		VL53LX_Dev_t *pdev,
		uint32_t *ptick_count_ms)
{



	VL53LX_Error status  = VL53LX_ERROR_NONE;
	(void) pdev;

	*ptick_count_ms = timeGetTime();

	trace_print(
	VL53LX_TRACE_LEVEL_DEBUG,
	"VL53LX_GetTickCount() = %5u ms;\n",
	*ptick_count_ms);

	return status;

}


VL53LX_Error VL53LX_WaitValueMaskEx(
	VL53LX_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{


	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53LX_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	_LOG_STRING_BUFFER(register_name);

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53LX_LOG_ENABLE

	VL53LX_get_register_name(
			index,
			register_name);


	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif



	VL53LX_GetTickCount(pdev, &start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;



#ifdef VL53LX_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
	_LOG_SET_TRACE_FUNCTIONS(VL53LX_TRACE_FUNCTION_NONE);



	while ((status == VL53LX_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53LX_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}




		VL53LX_GetTickCount(pdev, &current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}


	_LOG_SET_TRACE_FUNCTIONS(trace_functions);

	if (found == 0 && status == VL53LX_ERROR_NONE)
		status = VL53LX_ERROR_TIME_OUT;

	return status;
}


