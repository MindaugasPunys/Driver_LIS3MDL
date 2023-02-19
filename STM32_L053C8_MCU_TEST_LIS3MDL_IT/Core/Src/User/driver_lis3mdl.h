#ifndef USER_DRIVER_LIS3MDL_H_
#define USER_DRIVER_LIS3MDL_H_
/*_____________________________________________________________________________________________________________________
 * Includes */
#include "main.h"
#include "stdbool.h"
#include "stdint.h"

/*_____________________________________________________________________________________________________________________
 * Exported definitions and macros */

/*_____________________________________________________________________________________________________________________
 * Exported types */
//typedef struct sLis3mdl_DriverRawOut_t {
//	uint8_t out_x_l;
//	uint8_t out_x_h;
//	uint8_t out_y_l;
//	uint8_t out_y_h;
//	uint8_t out_z_l;
//	uint8_t out_z_h;
//} sLis3mdl_DriverRawOut_t;

typedef struct sLis3mdl_DriverOut_t {
	uint8_t status;
	int16_t x;
	int16_t y;
	int16_t z;
} sLis3mdl_DriverOut_t;

/*_____________________________________________________________________________________________________________________
 * Exported variables */

/*_____________________________________________________________________________________________________________________
 * Prototypes of exported functions */
bool Driver_Lis3mdl_Init(SPI_HandleTypeDef *hspi);
bool Driver_Lis3mdl_ReadData(SPI_HandleTypeDef *hspi, sLis3mdl_DriverOut_t *sensor_data);

bool Driver_Lis3mdl_ExampleApp(SPI_HandleTypeDef *hspi, sLis3mdl_DriverOut_t *sensor_data);

#endif /* USER_DRIVER_LIS3MDL_H_ */
