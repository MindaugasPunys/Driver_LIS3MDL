/*_____________________________________________________________________________________________________________________
 * Includes */
#include "driver_lis3mdl.h"

#include "stdlib.h"
#include "main.h"
/*_____________________________________________________________________________________________________________________
 * Private definitions and macros */
#define SPI_WRITE 0x00
#define SPI_READ 0x80

#define GAUS_SCALE_4G 4
#define GAUS_SCALE_8G 8
#define GAUS_SCALE_12G 12
#define GAUS_SCALE_16G 16
/*_____________________________________________________________________________________________________________________
 * Private typedef */

typedef enum eDriverLis3mdl_Reg_t {
	eDriverLis3mdlReg_First = 0,

	eDriverLis3mdlReg_OffsetXLm = eDriverLis3mdlReg_First,
	eDriverLis3mdlReg_OffsetXHm,
	eDriverLis3mdlReg_OffsetYLm,
	eDriverLis3mdlReg_OffsetYHm,
	eDriverLis3mdlReg_OffsetZLm,
	eDriverLis3mdlReg_OffsetZHm,

	eDriverLis3mdlReg_WhoAmI,

	eDriverLis3mdlReg_CtrlReg1,
	eDriverLis3mdlReg_CtrlReg2,
	eDriverLis3mdlReg_CtrlReg3,
	eDriverLis3mdlReg_CtrlReg4,
	eDriverLis3mdlReg_CtrlReg5,
	eDriverLis3mdlReg_StatusReg,

	eDriverLis3mdlReg_OutXL,
	eDriverLis3mdlReg_OutXH,
	eDriverLis3mdlReg_OutYL,
	eDriverLis3mdlReg_OutYH,
	eDriverLis3mdlReg_OutZL,
	eDriverLis3mdlReg_OutZH,

	eDriverLis3mdlReg_TempOutL,
	eDriverLis3mdlReg_TempOutH,

	eDriverLis3mdlReg_IntCfg,
	eDriverLis3mdlReg_IntSrc,
	eDriverLis3mdlReg_IntThsL,
	eDriverLis3mdlReg_IntThsH,

	eDriverLis3mdlReg_Last
} eDriverLis3mdl_Reg_t;
typedef struct sDriverLis3mdl_RegLut_t {
	uint8_t address;
	uint8_t reg_data;
} sDriverLis3mdl_RegLut_t;

// @formatter:off
typedef enum eDriverLis3mdl_Mask_t {
	eDriverLis3mdl_Mask_First = 0,

	// CtrlReg1____________________________________________
	// Temperature sensor
	eDriverLis3mdl_Temp_enable =  		0b10000000,
	eDriverLis3mdl_Temp_disable =  		0b00000000,
	// XY sensor performance mode
	eDriverLis3mdl_XYMode_Low = 		0b00000000,
	eDriverLis3mdl_XYMode_Medium = 		0b00100000,
	eDriverLis3mdl_XYMode_High = 		0b01000000,
	eDriverLis3mdl_XYMode_Ultra = 		0b01100000,
	// Output data rate
	eDriverLis3mdl_Odr_0_625 = 			0b00000000,
	eDriverLis3mdl_Odr_1_250 = 			0b00000100,
	eDriverLis3mdl_Odr_2_500 = 			0b00001000,
	eDriverLis3mdl_Odr_0005 = 			0b00001100,
	eDriverLis3mdl_Odr_0010 = 			0b00010000,
	eDriverLis3mdl_Odr_0020 = 			0b00010100,
	eDriverLis3mdl_Odr_0040 = 			0b00011000,
	eDriverLis3mdl_Odr_0080 = 			0b00011100,
	eDriverLis3mdl_Odr_Fast = 			0b00000010,
	// Self Test Mode
	eDriverLis3mdl_SelfTest_enable =	0b00000001,
	eDriverLis3mdl_SelfTest_disable=	0b00000000,

	// CtrlReg2____________________________________________
	// Full scare (Gaus)
	eDriverLis3mdl_Scale_4G =  			0b00000000,
	eDriverLis3mdl_Scale_8G =  			0b00100000,
	eDriverLis3mdl_Scale_12G = 			0b01000000,
	eDriverLis3mdl_Scale_16G = 			0b01100000,
	// Reboot / Software Restart
	eDriverLis3mdl_Reboot =  			0b00001000,
	eDriverLis3mdl_SoftRst =  			0b00000100,

	// CtrlReg3____________________________________________
	eDriverLis3mdl_LowPower =  			0b00100000,
	// 3-4 wire configuration
	eDriverLis3mdl_3WireMode =  		0b00000100,
	eDriverLis3mdl_4WireMode =			0b00000000,
	// Measurement mode
	eDriverLis3mdl_Mode_Continuous =	0b00000000,
	eDriverLis3mdl_Mode_Single = 		0b00000001,
	eDriverLis3mdl_Mode_Idle1 = 		0b00000010,
	eDriverLis3mdl_Mode_Idle2 = 		0b00000011,

	// CtrlReg4____________________________________________
	// Z sensor performance mode
	eDriverLis3mdl_ZMode_Low = 			0b00000000,
	eDriverLis3mdl_ZMode_Medium = 		0b00000100,
	eDriverLis3mdl_ZMode_High = 		0b00001000,
	eDriverLis3mdl_ZMode_Ultra = 		0b00001100,

	eDriverLis3mdl_Endian_Big =			0b00000000,
	eDriverLis3mdl_Endian_Little =		0b00000010,

	// CtrlReg5____________________________________________
	// Read only high part of data to increase efficiency
	eDriverLis3mdl_FastRead_Disable=	0b00000000,
	eDriverLis3mdl_FastRead_Enable =	0b10000000,
	// Block data update for magnetic data
	eDriverLis3mdl_BlockData_Disable=	0b00000000,
	eDriverLis3mdl_BlockData_Enable =	0b01000000,

	eDriverLis3mdl_Mask_Last
} eDriverLis3mdl_Mask_t;
// @formatter:on
typedef struct sDriverLis3mdl_Settings_t {
	// CtrlReg1____________________________________________
	uint8_t temp_sensor;
	uint8_t xy_mode;
	uint8_t odr_mode;
	uint8_t self_test;
	// CtrlReg2____________________________________________
	uint8_t full_scale;
	uint8_t reboot;
	uint8_t soft_reset;
	// CtrlReg3____________________________________________
	uint8_t low_power_mode;
	uint8_t wire_mode;
	uint8_t measure_mode;
	// CtrlReg4____________________________________________
	uint8_t z_mode;
	uint8_t endian_mode;
	// CtrlReg5____________________________________________
	uint8_t fast_read;
	uint8_t block_data;
} sDriverLis3mdl_Settings_t;

typedef enum eDriverLis3mdl_State_t {
	eDriverLis3mdl_State_First = 0,
	eDriverLis3mdl_State_Setup = eDriverLis3mdl_State_First,
	eDriverLis3mdl_State_GetStatus,
	eDriverLis3mdl_State_GetXL,
	eDriverLis3mdl_State_GetXH,
	eDriverLis3mdl_State_GetYL,
	eDriverLis3mdl_State_GetYH,
	eDriverLis3mdl_State_GetZL,
	eDriverLis3mdl_State_GetZH,
	eDriverLis3mdl_State_Last,
} eDriverLis3mdl_State_t;
/*_____________________________________________________________________________________________________________________
 * Private constants */

/*_____________________________________________________________________________________________________________________
 * Private variables */

// @formatter:off
sDriverLis3mdl_Settings_t sensor_config = {
	// CtrlReg1____________________________________________
	.temp_sensor = 	eDriverLis3mdl_Temp_enable, 		// Default = eDriverLis3mdl_Temp_enable
	.xy_mode = 		eDriverLis3mdl_XYMode_Ultra, 		// Default = eDriverLis3mdl_XYMode_Ultra
	.odr_mode = 	eDriverLis3mdl_Odr_Fast,			// Default = eDriverLis3mdl_Odr_Fast
	.self_test = 	eDriverLis3mdl_SelfTest_disable,	// Default = eDriverLis3mdl_SelfTest_disable
	// CtrlReg2____________________________________________
	.full_scale = 	eDriverLis3mdl_Scale_16G,			// Default = eDriverLis3mdl_Scale_12G
	.reboot = 		eDriverLis3mdl_Mask_First,			// Default = 0
	.soft_reset = 	eDriverLis3mdl_Mask_First,			// Default = 0

	// CtrlReg3____________________________________________
	.low_power_mode=eDriverLis3mdl_Mask_First,			// Default = 0
	.wire_mode = 	eDriverLis3mdl_4WireMode,			// Default = eDriverLis3mdl_4WireMode
	.measure_mode = eDriverLis3mdl_Mode_Continuous,		// Default = eDriverLis3mdl_Mode_Continuous
	// CtrlReg4____________________________________________
	.z_mode = 		eDriverLis3mdl_ZMode_Ultra,			// Default = eDriverLis3mdl_ZMode_Ultra
	.endian_mode = 	eDriverLis3mdl_Endian_Big,			// Default = eDriverLis3mdl_Endian_Big
	// CtrlReg5____________________________________________
	.fast_read = 	eDriverLis3mdl_FastRead_Disable,	// Default = eDriverLis3mdl_FastRead_Disable
	.block_data = 	eDriverLis3mdl_BlockData_Disable,	// Default = eDriverLis3mdl_BlockData_Disable
};

static sDriverLis3mdl_RegLut_t lis3mdl_reg_LUT[eDriverLis3mdlReg_Last] = {
		[eDriverLis3mdlReg_OffsetXLm] = { .address = 0x05, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OffsetXHm] = { .address = 0x06, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OffsetYLm] = { .address = 0x07, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OffsetYHm] = { .address = 0x08, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OffsetZLm] = { .address = 0x09, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OffsetZHm] = { .address = 0x0A, .reg_data = 0b00000000 },

		[eDriverLis3mdlReg_WhoAmI] = 	{ .address = 0x0F, .reg_data = 0b00000000 },

		[eDriverLis3mdlReg_CtrlReg1] = 	{ .address = 0x20, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_CtrlReg2] = 	{ .address = 0x21, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_CtrlReg3] = 	{ .address = 0x22, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_CtrlReg4] = 	{ .address = 0x23, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_CtrlReg5] = 	{ .address = 0x24, .reg_data = 0b00000000 },

		[eDriverLis3mdlReg_StatusReg] = { .address = 0x27, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OutXL] = 	{ .address = 0x28, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OutXH] = 	{ .address = 0x29, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OutYL] = 	{ .address = 0x2A, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OutYH] = 	{ .address = 0x2B, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OutZL] = 	{ .address = 0x2C, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_OutZH] = 	{ .address = 0x2D, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_TempOutL] = 	{ .address = 0x2E, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_TempOutH] = 	{ .address = 0x2F, .reg_data = 0b00000000 },

		[eDriverLis3mdlReg_IntCfg] = 	{ .address = 0x30, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_IntSrc] = 	{ .address = 0x31, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_IntThsL] = 	{ .address = 0x32, .reg_data = 0b00000000 },
		[eDriverLis3mdlReg_IntThsH] = 	{ .address = 0x33, .reg_data = 0b00000000 },
};
// @formatter:on
static uint8_t spi_cmd = 0x00;

// Default - Big endian
static uint8_t lsb_index = 0;
static uint8_t msb_index = 1;
static uint8_t reg_buffer[2] = { 0 };
static int16_t output_data = 0;

static volatile bool recieve_flag = false;
static eDriverLis3mdl_State_t driver_state = eDriverLis3mdl_State_Setup;
sDriverLis3mdl_SensorData_t local_sensor_data = { 0 };
HAL_StatusTypeDef status = HAL_OK;

/*_____________________________________________________________________________________________________________________
 * Exported variables and references */

/*_____________________________________________________________________________________________________________________
 * Prototypes of private functions */
static int16_t Utility_TwosCompToDec(uint8_t msb, uint8_t lsb);
static bool Utility_DecToTwosComp(int16_t decimalValue, uint8_t *msb, uint8_t *lsb);
/*_____________________________________________________________________________________________________________________
 * Definitions of private functions */
static bool Driver_Lis3mdl_WriteData(SPI_HandleTypeDef *hspi, eDriverLis3mdl_Reg_t reg, uint8_t data) {
	if (hspi == NULL || reg >= eDriverLis3mdlReg_Last) {
		return false;
	}
	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_RESET);

	spi_cmd = SPI_WRITE | lis3mdl_reg_LUT[reg].address;
	status = HAL_SPI_Transmit(hspi, &spi_cmd, 1, 100);
	status = HAL_SPI_Transmit(hspi, &data, 1, 100);

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	return true;
}
static bool Driver_Lis3mdl_WriteReg(SPI_HandleTypeDef *hspi, eDriverLis3mdl_Reg_t reg) {
	if (hspi == NULL || reg >= eDriverLis3mdlReg_Last) {
		return false;
	}

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_RESET);

	spi_cmd = SPI_WRITE | lis3mdl_reg_LUT[reg].address;
	status = HAL_SPI_Transmit(hspi, &spi_cmd, 1, 100);
	status = HAL_SPI_Transmit(hspi, &lis3mdl_reg_LUT[reg].reg_data, 1, 100);

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	return true;
}

static bool Driver_Lis3mdl_Read(SPI_HandleTypeDef *hspi, eDriverLis3mdl_Reg_t reg, uint8_t *data) {
	if (hspi == NULL || reg >= eDriverLis3mdlReg_Last || data == NULL) {
		return false;
	}

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_RESET);

	spi_cmd = SPI_READ | lis3mdl_reg_LUT[reg].address;
	status = HAL_SPI_Transmit(hspi, &spi_cmd, 1, 100);
	status = HAL_SPI_Receive(hspi, data, 1, 100);

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	return true;
}
static bool Driver_Lis3mdl_Read_IT(SPI_HandleTypeDef *hspi, eDriverLis3mdl_Reg_t reg, uint8_t *data) {
	if (hspi == NULL || reg >= eDriverLis3mdlReg_Last || data == NULL) {
		return false;
	}

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_RESET);

	spi_cmd = SPI_READ | lis3mdl_reg_LUT[reg].address;
	status = HAL_SPI_Transmit_IT(hspi, &spi_cmd, 1);
	status = HAL_SPI_Receive_IT(hspi, data, 1);
	recieve_flag = true;

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	return true;
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	recieve_flag = false;
}

static bool Driver_Lis3mdl_InitCtrlReg(SPI_HandleTypeDef *hspi) {
	if (hspi == NULL) {
		return false;
	}
	// CtrlReg1____________________________________________
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg1].reg_data = sensor_config.temp_sensor | sensor_config.xy_mode
			| sensor_config.odr_mode | sensor_config.self_test;
	// CtrlReg2____________________________________________
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg2].reg_data = sensor_config.full_scale | sensor_config.reboot
			| sensor_config.soft_reset;
	// CtrlReg3____________________________________________
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg3].reg_data = sensor_config.low_power_mode | sensor_config.wire_mode
			| sensor_config.measure_mode;
	// CtrlReg4____________________________________________
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg4].reg_data = sensor_config.z_mode | sensor_config.endian_mode;
	if (sensor_config.endian_mode == eDriverLis3mdl_Endian_Big) {
		lsb_index = 0;
		msb_index = 1;
	} else {
		lsb_index = 1;
		msb_index = 0;
	}
	// CtrlReg5____________________________________________
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg5].reg_data = sensor_config.fast_read | sensor_config.block_data;

	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg2);
	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg1);
	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg4);
	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg3);
	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg5);

	return true;
}
static float Driver_Lis3mdl_ToGausScale(int16_t data) {
	switch (sensor_config.full_scale) {
		case (eDriverLis3mdl_Scale_4G): {
			return (data - INT16_MIN) * (2 * GAUS_SCALE_4G) / (INT16_MAX - INT16_MIN) - GAUS_SCALE_4G;
		}
		case (eDriverLis3mdl_Scale_8G): {
			return (data - INT16_MIN) * (2 * GAUS_SCALE_8G) / (INT16_MAX - INT16_MIN) - GAUS_SCALE_8G;
		}
		case (eDriverLis3mdl_Scale_12G): {
			return (data - INT16_MIN) * (2 * GAUS_SCALE_12G) / (INT16_MAX - INT16_MIN) - GAUS_SCALE_12G;
		}
		case (eDriverLis3mdl_Scale_16G): {
			return (data - INT16_MIN) * (2 * GAUS_SCALE_16G) / (INT16_MAX - INT16_MIN) - GAUS_SCALE_16G;
		}
		default: {
			return 255;
		}
	}
}

static int16_t Utility_TwosCompToDec(uint8_t msb, uint8_t lsb) {
	int16_t result = 0;
	int16_t sign = msb & 0x80 ? -1 : 1; // Check the sign bit of the MSB

// 	If the number is negative, first take the twos complement
	if (sign == -1) {
		msb = ~msb;
		lsb = ~lsb;
		result = -1; // Start with -1 instead of 0 for the carry
	}
// 	Add the MSB shifted left by 8 bits, and the LSB
	result = (result << 8) | msb;
	result = (result << 8) | lsb;

// 	If the number is negative, subtract 1 to get the correct value
	if (sign == -1) {
		result = -result - 1;
	}

	return result;
}
static bool Utility_DecToTwosComp(int16_t decimalValue, uint8_t *msb, uint8_t *lsb) {
	*msb = (uint8_t) ((decimalValue >> 8) & 0xFF);  // Extract high byte
	*lsb = (uint8_t) (decimalValue & 0xFF);  // Extract low byte
	if (decimalValue < 0) {
		// If the input is negative, compute the two's complement
		*msb = ~(*msb);
		*lsb = ~(*lsb) + 1;
		if (*lsb == 0) {
			// If the low byte wrapped around to 0, increment the high byte
			*msb = *msb + 1;
		}
	}
	return true;
}

/*_____________________________________________________________________________________________________________________
 * Definitions of exported functions */
bool Driver_Lis3mdl_Init(SPI_HandleTypeDef *hspi) {
	if (hspi == NULL) {
		return false;
	}

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	Driver_Lis3mdl_InitCtrlReg(hspi);

	driver_state = eDriverLis3mdl_State_GetStatus;
	recieve_flag = false;
	return true;
}
bool Driver_Lis3mdl_Deinit(SPI_HandleTypeDef *hspi) {
	if (hspi == NULL) {
		return false;
	}

	// Default configuration + Software reset and idle
	sensor_config.temp_sensor = eDriverLis3mdl_Temp_enable;
	sensor_config.xy_mode = eDriverLis3mdl_XYMode_Ultra;
	sensor_config.odr_mode = eDriverLis3mdl_Odr_Fast;
	sensor_config.self_test = eDriverLis3mdl_SelfTest_disable;
	sensor_config.full_scale = eDriverLis3mdl_Scale_12G;
	sensor_config.reboot = eDriverLis3mdl_Mask_First;
	sensor_config.soft_reset = eDriverLis3mdl_SoftRst;
	sensor_config.low_power_mode = eDriverLis3mdl_Mask_First;
	sensor_config.wire_mode = eDriverLis3mdl_4WireMode;
	sensor_config.measure_mode = eDriverLis3mdl_Mode_Idle2;
	sensor_config.z_mode = eDriverLis3mdl_ZMode_Ultra;
	sensor_config.endian_mode = eDriverLis3mdl_Endian_Big;
	sensor_config.fast_read = eDriverLis3mdl_FastRead_Disable;
	sensor_config.block_data = eDriverLis3mdl_BlockData_Disable;

	Driver_Lis3mdl_InitCtrlReg(hspi);
	recieve_flag = false;

	driver_state = eDriverLis3mdl_State_Setup;
	return true;
}

bool Driver_Lis3mdl_ReadSensorData(SPI_HandleTypeDef *hspi, sDriverLis3mdl_SensorData_t *sensor_data) {
	if (hspi == NULL || sensor_data == NULL) {
		return false;
	}
	if (driver_state == eDriverLis3mdl_State_Setup) {
		Driver_Lis3mdl_Init(hspi);
	}

	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_StatusReg, &(sensor_data->status));
// 	Status bit3 - new data available
	if ((sensor_data->status & 0b00000100) == 0) {
		return false;
	}

	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutXL, &reg_buffer[0]);
	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutXH, &reg_buffer[1]);
	output_data = Utility_TwosCompToDec(reg_buffer[msb_index], reg_buffer[lsb_index]);
	sensor_data->x = Driver_Lis3mdl_ToGausScale(output_data);

	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutYL, &reg_buffer[0]);
	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutYH, &reg_buffer[1]);
	output_data = Utility_TwosCompToDec(reg_buffer[msb_index], reg_buffer[lsb_index]);
	sensor_data->y = Driver_Lis3mdl_ToGausScale(output_data);

	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutZH, &reg_buffer[1]);
	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutZL, &reg_buffer[0]);
	output_data = Utility_TwosCompToDec(reg_buffer[msb_index], reg_buffer[lsb_index]);
	sensor_data->z = Driver_Lis3mdl_ToGausScale(output_data);

	return true;
}

bool Driver_Lis3mdl_UpdateSensorData_IT(SPI_HandleTypeDef *hspi) {
	if (hspi == NULL) {
		return false;
	}

	if (recieve_flag == true) {
		return false;
	}
	if (driver_state == eDriverLis3mdl_State_Setup) {
		Driver_Lis3mdl_Init(hspi);
	}

	switch (driver_state) {
		case eDriverLis3mdl_State_GetStatus: {
			Driver_Lis3mdl_Read_IT(hspi, eDriverLis3mdlReg_StatusReg, &(local_sensor_data.status));
			driver_state = eDriverLis3mdl_State_GetXH;
			break;
		}
		case eDriverLis3mdl_State_GetXH: {
			if ((local_sensor_data.status & 0b00000100) == 0) {
				driver_state = eDriverLis3mdl_State_GetStatus;
				return false;
			}
			Driver_Lis3mdl_Read_IT(hspi, eDriverLis3mdlReg_OutXH, &reg_buffer[1]);
			driver_state = eDriverLis3mdl_State_GetXL;
			break;
		}
		case eDriverLis3mdl_State_GetXL: {
			Driver_Lis3mdl_Read_IT(hspi, eDriverLis3mdlReg_OutXL, &reg_buffer[0]);
			driver_state = eDriverLis3mdl_State_GetYH;
			break;
		}
		case eDriverLis3mdl_State_GetYH: {
			output_data = Utility_TwosCompToDec(reg_buffer[msb_index], reg_buffer[lsb_index]);
			local_sensor_data.x = Driver_Lis3mdl_ToGausScale(output_data);

			Driver_Lis3mdl_Read_IT(hspi, eDriverLis3mdlReg_OutYH, &reg_buffer[1]);
			driver_state = eDriverLis3mdl_State_GetYL;
			break;
		}
		case eDriverLis3mdl_State_GetYL: {
			Driver_Lis3mdl_Read_IT(hspi, eDriverLis3mdlReg_OutYL, &reg_buffer[0]);
			driver_state = eDriverLis3mdl_State_GetZH;
			break;
		}
		case eDriverLis3mdl_State_GetZH: {
			output_data = Utility_TwosCompToDec(reg_buffer[msb_index], reg_buffer[lsb_index]);
			local_sensor_data.y = Driver_Lis3mdl_ToGausScale(output_data);

			Driver_Lis3mdl_Read_IT(hspi, eDriverLis3mdlReg_OutZH, &reg_buffer[1]);
			driver_state = eDriverLis3mdl_State_GetZL;
			break;
		}
		case eDriverLis3mdl_State_GetZL: {
			Driver_Lis3mdl_Read_IT(hspi, eDriverLis3mdlReg_OutZL, &reg_buffer[0]);
			driver_state = eDriverLis3mdl_State_Last;
			break;
		}
		case eDriverLis3mdl_State_Last: {
			output_data = Utility_TwosCompToDec(reg_buffer[msb_index], reg_buffer[lsb_index]);
			local_sensor_data.z = Driver_Lis3mdl_ToGausScale(output_data);
			driver_state = eDriverLis3mdl_State_GetStatus;
			break;
		}

		default: {
			driver_state = eDriverLis3mdl_State_GetStatus;
			break;
		}
	}
	return true;
}
sDriverLis3mdl_SensorData_t Driver_Lis3mdl_GetSensorData(void) {
	return local_sensor_data;
}

bool Driver_Lis3mdl_ExampleApp(sDriverLis3mdl_SensorData_t *sensor_data) {
	if (sensor_data == NULL) {
		return false;
	}

	if (abs(sensor_data->x) >= abs(sensor_data->y)) {
		if (sensor_data->x >= 0) {
			HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, LED4_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, LED4_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_RESET);
	} else {
		if (sensor_data->y >= 0) {
			HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, LED4_Pin, GPIO_PIN_RESET);
	}
	return true;
}
