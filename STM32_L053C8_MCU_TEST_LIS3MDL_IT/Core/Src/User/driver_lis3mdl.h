#ifndef USER_DRIVER_LIS3MDL_H_
#define USER_DRIVER_LIS3MDL_H_
/*_____________________________________________________________________________________________________________________
 * Includes */
#include "stdbool.h"
#include "stdint.h"
#include "stm32l0xx_hal.h"
/*_____________________________________________________________________________________________________________________
 * Exported definitions and macros */

/*_____________________________________________________________________________________________________________________
 * Exported types */
typedef struct sDriverLis3mdl_SensorData_t {
	uint8_t status;
	float x;
	float y;
	float z;
} sDriverLis3mdl_SensorData_t;

/*_____________________________________________________________________________________________________________________
 * Exported variables */

/*_____________________________________________________________________________________________________________________
 * Prototypes of exported functions */
bool Driver_Lis3mdl_Init(SPI_HandleTypeDef *hspi);
bool Driver_Lis3mdl_ReadSensorData(SPI_HandleTypeDef *hspi, sDriverLis3mdl_SensorData_t *sensor_data);

bool Driver_Lis3mdl_UpdateSensorData_IT(SPI_HandleTypeDef *hspi);
sDriverLis3mdl_SensorData_t Driver_Lis3mdl_GetSensorData(void);

bool Driver_Lis3mdl_ExampleApp(sDriverLis3mdl_SensorData_t *sensor_data);

#endif /* USER_DRIVER_LIS3MDL_H_ */
