/*_____________________________________________________________________________________________________________________
 * Includes */
#include "driver_lis3mdl.h"
#include "stdlib.h"
/*_____________________________________________________________________________________________________________________
 * Private definitions and macros */
#define SPI_WRITE 0x00
#define SPI_READ 0x80
/*_____________________________________________________________________________________________________________________
 * Private typedef */

typedef enum eDriverLis3mdlReg_t {
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
} eDriverLis3mdlReg_t;
typedef struct sDriverLis3mdlLut_t {
	uint8_t address;
	uint8_t reg_data;
} sDriverLis3mdlLut_t;

// @formatter:off
typedef enum eDriverLis3mdlMask_t {
	eLis3mdl_Mask_First = 0,

	// CtrlReg1____________________________________________
	// Temperature sensor
	eLis3mdl_Temp_enable =  	0b10000000,
	eLis3mdl_Temp_disable =  	0b00000000,
	// XY sensor performance mode
	eLis3mdl_XYMode_Low = 		0b00000000,
	eLis3mdl_XYMode_Medium = 	0b00100000,
	eLis3mdl_XYMode_High = 		0b01000000,
	eLis3mdl_XYMode_Ultra = 	0b01100000,
	// Output data rate
	eLis3mdl_Odr_0_625 = 		0b00000000,
	eLis3mdl_Odr_1_250 = 		0b00000100,
	eLis3mdl_Odr_2_500 = 		0b00001000,
	eLis3mdl_Odr_0005 = 		0b00001100,
	eLis3mdl_Odr_0010 = 		0b00010000,
	eLis3mdl_Odr_0020 = 		0b00010100,
	eLis3mdl_Odr_0040 = 		0b00011000,
	eLis3mdl_Odr_0080 = 		0b00011100,
	eLis3mdl_Odr_Fast = 		0b00000010,
	// Self Test Mode
	eLis3mdl_SelfTest_enable =	0b00000001,
	eLis3mdl_SelfTest_disable=	0b00000000,

	// CtrlReg2____________________________________________
	// Full scare (Gaus)
	eLis3mdl_Scale_4G =  		0b00000000,
	eLis3mdl_Scale_8G =  		0b00100000,
	eLis3mdl_Scale_12G = 		0b01000000,
	eLis3mdl_Scale_16G = 		0b01100000,
	// Reboot / Software Restart
	eLis3mdl_Reboot =  			0b00001000,
	eLis3mdl_SoftRst =  		0b00000100,

	// CtrlReg3____________________________________________
	eLis3mdl_LowPower =  		0b00100000,
	// 3-4 wire configuration
	eLis3mdl_3WireMode =  		0b00000100,
	eLis3mdl_4WireMode =		0b00000000,
	// Measurement mode
	eLis3mdl_Mode_Continuous =	0b00000000,
	eLis3mdl_Mode_Single = 		0b00000001,
	eLis3mdl_Mode_Idle1 = 		0b00000010,
	eLis3mdl_Mode_Idle2 = 		0b00000011,

	// CtrlReg4____________________________________________
	// Z sensor performance mode
	eLis3mdl_ZMode_Low = 		0b00000000,
	eLis3mdl_ZMode_Medium = 	0b00000100,
	eLis3mdl_ZMode_High = 		0b00001000,
	eLis3mdl_ZMode_Ultra = 		0b00001100,

	eLis3mdl_Endian_Big =		0b00000000,
	eLis3mdl_Endian_Little =	0b00000010,

	// CtrlReg5____________________________________________
	// Read only high part of data to increase efficiency
	eLis3mdl_FastRead_Disable=	0b00000000,
	eLis3mdl_FastRead_Enable =	0b10000000,
	// Block data update for magnetic data
	eLis3mdl_BlockData_Disable=	0b00000000,
	eLis3mdl_BlockData_Enable =	0b01000000,

	eLis3mdl_Mask_Last
} eDriverLis3mdlMask_t;
// @formatter:on
typedef struct sLis3mdl_DriverSettings_t {
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
} sLis3mdl_DriverSettings_t;

/*_____________________________________________________________________________________________________________________
 * Private constants */

/*_____________________________________________________________________________________________________________________
 * Private variables */

// @formatter:off
sLis3mdl_DriverSettings_t sensor_config = {
	// CtrlReg1____________________________________________
	.temp_sensor = 	eLis3mdl_Temp_enable, 		// Default = eLis3mdl_Temp_enable
	.xy_mode = 		eLis3mdl_XYMode_Ultra, 		// Default = eLis3mdl_XYMode_Ultra
	.odr_mode = 	eLis3mdl_Odr_Fast,			// Default = eLis3mdl_Odr_Fast
	.self_test = 	eLis3mdl_SelfTest_disable,	// Default = eLis3mdl_SelfTest_disable
	// CtrlReg2____________________________________________
	.full_scale = 	eLis3mdl_Scale_12G,			// Default = eLis3mdl_Scale_12G
	.reboot = 		eLis3mdl_Mask_First,		// Default = 0
	.soft_reset = 	eLis3mdl_Mask_First,		// Default = 0

	// CtrlReg3____________________________________________
	.low_power_mode=eLis3mdl_Mask_First,		// Default = 0
	.wire_mode = 	eLis3mdl_4WireMode,			// Default = eLis3mdl_4WireMode
	.measure_mode = eLis3mdl_Mode_Continuous,	// Default = eLis3mdl_Mode_Continuous
	// CtrlReg4____________________________________________
	.z_mode = eLis3mdl_ZMode_Ultra,				// Default = eLis3mdl_ZMode_Ultra
	.endian_mode = eLis3mdl_Endian_Big,			// Default = eLis3mdl_Endian_Big
	// CtrlReg5____________________________________________
	.fast_read = eLis3mdl_FastRead_Disable,		// Default = eLis3mdl_FastRead_Disable
	.block_data = eLis3mdl_BlockData_Disable,	// Default = eLis3mdl_BlockData_Disable
};

static sDriverLis3mdlLut_t lis3mdl_reg_LUT[eDriverLis3mdlReg_Last] = {
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
static uint8_t sensor_id = 0x00;

static uint8_t lsb_index = 0;
static uint8_t msb_index = 1;
static uint8_t raw_sensor_data[2] = { 0 };

HAL_StatusTypeDef status = HAL_OK;

/*_____________________________________________________________________________________________________________________
 * Exported variables and references */

/*_____________________________________________________________________________________________________________________
 * Prototypes of private functions */
static int16_t Utility_TwosCompToDec(uint8_t msb, uint8_t lsb);
static void Utility_DecToTwosComp(int16_t decimalValue, uint8_t *msb, uint8_t *lsb);
/*_____________________________________________________________________________________________________________________
 * Definitions of private functions */
static bool Driver_Lis3mdl_WriteData(SPI_HandleTypeDef *hspi, eDriverLis3mdlReg_t reg, uint8_t data) {
	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_RESET);

	spi_cmd = SPI_WRITE | lis3mdl_reg_LUT[reg].address;
	status = HAL_SPI_Transmit(hspi, &spi_cmd, 1, 100);
	status = HAL_SPI_Transmit(hspi, &data, 1, 100);

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	return true;
}
static bool Driver_Lis3mdl_WriteReg(SPI_HandleTypeDef *hspi, eDriverLis3mdlReg_t reg) {
	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_RESET);

	spi_cmd = SPI_WRITE | lis3mdl_reg_LUT[reg].address;
	status = HAL_SPI_Transmit(hspi, &spi_cmd, 1, 100);
	status = HAL_SPI_Transmit(hspi, &lis3mdl_reg_LUT[reg].reg_data, 1, 100);

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	return true;
}
static bool Driver_Lis3mdl_Read(SPI_HandleTypeDef *hspi, eDriverLis3mdlReg_t reg, uint8_t *data) {
	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_RESET);

	spi_cmd = SPI_READ | lis3mdl_reg_LUT[reg].address;
	status = HAL_SPI_Transmit(hspi, &spi_cmd, 1, 100);
	status = HAL_SPI_Receive(hspi, data, 1, 100);

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	return true;
}
static bool Driver_Lis3mdl_InitCtrlReg(SPI_HandleTypeDef *hspi) {
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg1].reg_data = sensor_config.temp_sensor | sensor_config.xy_mode
			| sensor_config.odr_mode | sensor_config.self_test;
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg2].reg_data = sensor_config.full_scale | sensor_config.reboot
			| sensor_config.soft_reset;
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg3].reg_data = sensor_config.low_power_mode | sensor_config.wire_mode
			| sensor_config.measure_mode;
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg4].reg_data = sensor_config.z_mode | sensor_config.endian_mode;
	lis3mdl_reg_LUT[eDriverLis3mdlReg_CtrlReg5].reg_data = sensor_config.fast_read | sensor_config.block_data;

	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg2);
	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg1);
	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg4);
	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg3);
	Driver_Lis3mdl_WriteReg(hspi, eDriverLis3mdlReg_CtrlReg5);

	return true;
}

static int16_t Utility_TwosCompToDec(uint8_t msb, uint8_t lsb) {
	int16_t result = 0;
	int16_t sign = msb & 0x80 ? -1 : 1; // Check the sign bit of the MSB

	// If the number is negative, first take the two's complement
	if (sign == -1) {
		msb = ~msb;
		lsb = ~lsb;
		result = -1; // Start with -1 instead of 0 for the carry
	}
	// Add the MSB shifted left by 8 bits, and the LSB
	result = (result << 8) | msb;
	result = (result << 8) | lsb;

	// If the number is negative, subtract 1 to get the correct value
	if (sign == -1) {
		result = -result - 1;
	}

	return result;
}
static void Utility_DecToTwosComp(int16_t decimalValue, uint8_t *msb, uint8_t *lsb) {
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
}
//static bool Utility_DecToTwosComp(unit16_t decimal_value, uint8_t msb, uint8_t lsb) {
//
//	return true;
//}

/*_____________________________________________________________________________________________________________________
 * Definitions of exported functions */
bool Driver_Lis3mdl_Init(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
// 	STARTUP SEQUENCE
	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_WhoAmI, &sensor_id);

	Driver_Lis3mdl_InitCtrlReg(hspi);

	HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, GPIO_PIN_SET);
	return true;
}
bool Driver_Lis3mdl_ReadData(SPI_HandleTypeDef *hspi, sLis3mdl_DriverOut_t *sensor_data) {
	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_StatusReg, &(sensor_data->status));
// 	STATUS BIT3 - NEW DATA AVAILABE
	if ((sensor_data->status & 0b00000100) == 0) {
		return false;
	}

	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutXL, &raw_sensor_data[0]);
	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutXH, &raw_sensor_data[1]);
	sensor_data->x = Utility_TwosCompToDec(raw_sensor_data[1], raw_sensor_data[0]);

	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutYL, &raw_sensor_data[0]);
	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutYH, &raw_sensor_data[1]);
	sensor_data->y = Utility_TwosCompToDec(raw_sensor_data[1], raw_sensor_data[0]);

	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutZL, &raw_sensor_data[0]);
	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_OutZH, &raw_sensor_data[1]);
	sensor_data->z = Utility_TwosCompToDec(raw_sensor_data[1], raw_sensor_data[0]);

	Driver_Lis3mdl_Read(hspi, eDriverLis3mdlReg_WhoAmI, &sensor_id);
	return true;
}

bool Driver_Lis3mdl_ExampleApp(SPI_HandleTypeDef *hspi, sLis3mdl_DriverOut_t *sensor_data) {

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

// Little-Big endian conversion
// Interupts
// scale
// add assert
