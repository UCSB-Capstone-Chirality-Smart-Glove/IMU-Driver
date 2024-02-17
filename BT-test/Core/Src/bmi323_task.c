#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include "cmsis_os.h"
#include "stm32wbxx_hal.h"
#include "bmi323_task.h"
#include "bmi323.h"
#include "main.h"

struct bmi3_dev dev, dev2;
uint8_t bmi323_dev_addr;
struct bmi3_sensor_data sensor_data = { 1 };

uint8_t GTXBuffer[512], GRXBuffer[2048];

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

/******************************************************************************/
/*!               User interface functions                                    */

void bst_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 84; i++){;}
	}
}

void UART_Printf(uint8_t* buff, uint16_t size)
{
    HAL_UART_Transmit(&UART_HANDLE, buff, size, BUS_TIMEOUT);
}

char chBuffer[512];
void PDEBUG(char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    vsnprintf(chBuffer, sizeof(chBuffer), format, ap);
    UART_Printf((uint8_t *)chBuffer,strlen(chBuffer));
    va_end(ap);
}

#define port1 GPIOA
#define pin1 GPIO_PIN_4
#define port2 GPIOA
#define pin2 GPIO_PIN_5

// Either create multiple devices or find a way to change port (first way easier / can change device purpose)
int8_t SensorAPI_SPIx_Read1(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET); // NSS high
    memcpy(reg_data, GRXBuffer+1, length);

    return 0;
}

int8_t SensorAPI_SPIx_Read2(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET); // NSS high
    memcpy(reg_data, GRXBuffer+1, length);

    return 0;
}

int8_t SensorAPI_SPIx_Write1(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET); // NSS high

    return 0;
}

int8_t SensorAPI_SPIx_Write2(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET); // NSS high

    return 0;
}

/*!
 * @brief This API prints the execution status
 */
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI3_OK:

            /*! Do nothing */
            break;

        case BMI3_E_NULL_PTR:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI3_E_COM_FAIL:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI3_E_DEV_NOT_FOUND:
            PDEBUG("%s\t", api_name);
            PDEBUG("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI3_E_INVALID_SENSOR:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INT_PIN:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI3_E_ACC_INVALID_CFG:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x20\r\n",
                rslt);
            break;

        case BMI3_E_GYRO_INVALID_CFG:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x21\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INPUT:
            PDEBUG("%s\t", api_name);
            PDEBUG("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI3_E_INVALID_STATUS:
            PDEBUG("%s\t", api_name);
            PDEBUG("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI3_E_DATA_RDY_INT_FAILED:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_FOC_POSITION:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_ST_SELECTION:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid self-test selection error. It occurs when there is an invalid precondition" "settings such as alternate accelerometer and gyroscope enable bits, accelerometer mode and output data rate\r\n",
                rslt);
            break;

        case BMI3_E_OUT_OF_RANGE:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Out of range error. It occurs when the range exceeds the maximum range for accel while performing FOC\r\n",
                rslt);
            break;

        case BMI3_E_FEATURE_ENGINE_STATUS:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Feature engine status error. It occurs when the feature engine enable mask is not set\r\n",
                rslt);
            break;

        default:
            PDEBUG("%s\t", api_name);
            PDEBUG("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi3_interface_init(struct bmi3_dev *bmi, uint8_t intf)
{
	int8_t rslt = BMI3_OK;

	/* Bus configuration : SPI */
	if (intf == BMI3_SPI_INTF)
	{
		PDEBUG("SPI Interface \n");
		bmi->intf = BMI3_SPI_INTF;
//		bmi->read = (bmi3_read_fptr_t)SensorAPI_SPIx_Read1;
//		bmi->write = (bmi3_write_fptr_t)SensorAPI_SPIx_Write1;
	}


	/* Assign device address to interface pointer */
	bmi->intf_ptr = &bmi323_dev_addr;
	bmi323_dev_addr = 0;

	/* Configure delay in microseconds */
	bmi->delay_us = bst_delay_us;

	/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
	bmi->read_write_len = READ_WRITE_LEN;

	return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

int8_t Open_BMI323_ACC(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_ACCEL;

	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK)
	{
		/* Update all or any of the accelerometer configurations */

		/* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
		config.cfg.acc.bwp = BMI3_ACC_BW_ODR_HALF;

		/* Set number of average samples for accel. */
		config.cfg.acc.avg_num = BMI3_ACC_AVG8;

		/* Enable the accel mode where averaging of samples
		* will be done based on above set bandwidth and ODR.
		* Note : By default accel is disabled. The accel will get enable by selecting the mode.
		*/
		config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

		config.cfg.acc.odr = BMI3_ACC_ODR_1600HZ; //BMI3_ACC_ODR_400HZ;

		/* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
		config.cfg.acc.range     = BMI3_ACC_RANGE_4G;

		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK)
		{
			PDEBUG("Open ACC failed, rslt=%d\r\n", rslt);
		}
		else
		{
			PDEBUG("Open ACC set successfully\r\n");

			/* Get the configuration settings for validation */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			if (rslt == BMI3_OK)
			{
				PDEBUG("Get ACC configuration successful\r\n");
				PDEBUG("acc_mode = %d\r\n", config.cfg.acc.acc_mode);
				PDEBUG("bwp = %d\r\n", config.cfg.acc.bwp);
				PDEBUG("odr = %d\r\n", config.cfg.acc.odr);
				PDEBUG("Range = %d\r\n", config.cfg.acc.range);
				PDEBUG("avg_num = %d\r\n", config.cfg.acc.avg_num);
			}
		}
	}

	return rslt;
}

int8_t Close_BMI323_ACC(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_ACCEL;
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK)
	{
		config.cfg.acc.acc_mode = BMI3_ACC_MODE_DISABLE;

		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK)
		{
			PDEBUG("Close ACC failed, rslt=%d\r\n", rslt);
		}
		else
		{
			PDEBUG("Open ACC successfully\r\n");
		}
	}
	bmi3_error_codes_print_result("Close_BMI323_ACC",rslt);

	return rslt;
}

int8_t Open_BMI323_GYRO(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_GYRO;
	
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK) 
	{
		config.cfg.gyr.odr = BMI3_GYR_ODR_400HZ;
		/* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
		config.cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;

		/*	The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
		*	Value	Name	  Description
		*	  0   odr_half	 BW = gyr_odr/2
		*	  1  odr_quarter BW = gyr_odr/4
		*/
		config.cfg.gyr.bwp = BMI3_GYR_BW_ODR_QUARTER;
		/* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
		config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
		/* Value    Name    Description
		*  000     avg_1   No averaging; pass sample without filtering
		*  001     avg_2   Averaging of 2 samples
		*  010     avg_4   Averaging of 4 samples
		*  011     avg_8   Averaging of 8 samples
		*  100     avg_16  Averaging of 16 samples
		*  101     avg_32  Averaging of 32 samples
		*  110     avg_64  Averaging of 64 samples
		*/
		config.cfg.gyr.avg_num = BMI3_GYR_AVG64;
	
		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK) 
		{
			PDEBUG("Open GYRO failed\r\n");
		} 
		else
		{
			PDEBUG("Open GYRO successfully\r\n");

			/* Get the configuration settings for validation */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			if (rslt == BMI3_OK) 
			{
				PDEBUG("Get BMI323_GYRO Configuration successful\r\n");
				PDEBUG("gyr_mode = %d\r\n", config.cfg.gyr.gyr_mode);
				PDEBUG("ODR = %d\r\n", config.cfg.gyr.odr);
				PDEBUG("Range = %d\r\n", config.cfg.gyr.range);
				PDEBUG("bw = %d\r\n", config.cfg.gyr.bwp);
				PDEBUG("avg_num = %d\r\n", config.cfg.gyr.avg_num);
			}
		}
	}

	return rslt;
}

int8_t Close_BMI323_GYRO(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;

	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_GYRO;
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK) 
	{
		config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_DISABLE;
		
		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK) 
		{
			PDEBUG("Close GYRO failed, rslt=%d\r\n", rslt);
		} 
		else
		{
			PDEBUG("Open GYRO successfully\r\n");
		}
	}
	bmi3_error_codes_print_result("Close_BMI323_GYRO",rslt);

	return rslt;
}

void BMI323_Print_ALLRegs(struct bmi3_dev *dev)
{
	uint8_t value;

	uint8_t reg_addr;

	for(reg_addr = 0; reg_addr < 0x80; reg_addr++)
	{
		bmi3_get_regs(reg_addr, &value, 1, dev);
		PDEBUG("0x%02X, value=0x%02X\r\n", reg_addr, value);
	}
}

int8_t Init_BMI323(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	uint8_t chipid;

	rslt = bmi3_interface_init(dev, BMI3_SPI_INTF);
	bmi3_error_codes_print_result("bmi3_interface_init",rslt);
	osDelay(100);

	/* Initialize bmi323. */
	rslt = bmi323_init(dev);
	bmi3_error_codes_print_result("bmi323_init",rslt);

	if (rslt != BMI3_OK)
	{
		PDEBUG("bmi323_init() failed, error code: %d\r\n", rslt);
		return rslt;
	}
	else
	{
		PDEBUG("BMI323 initialized successfully\r\n");
	}

	rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, &chipid, 1, dev);
	if (rslt != BMI3_OK) 
	{
		PDEBUG("read chip ID failed, error code: %d\r\n", rslt);
		return rslt;
	}

	PDEBUG("Chip ID:0x%02x\r\n", chipid);
	Open_BMI323_ACC(dev);
	Open_BMI323_GYRO(dev);

	return rslt;
}

int16_t gyro_data[3];
int16_t acc_data[3];

uint8_t read_sensor(struct bmi3_dev dev)
{
	uint8_t acc_regs[6];
	uint8_t gyr_regs[6];
	uint8_t rslt;

	float gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z;
//	uint16_t temper;
	rslt = bmi3_get_regs(BMI3_REG_GYR_DATA_X, &gyr_regs, 6, &dev);
	if (rslt != BMI3_OK)
	{
		PDEBUG("read gyro register failed, error code: %d\r\n", rslt);
		return rslt;
	}
	gyro_data[0] = (gyr_regs[0] | (uint16_t)gyr_regs[1] << 8);
	gyro_data[1] = (gyr_regs[2] | (uint16_t)gyr_regs[3] << 8);
	gyro_data[2] = (gyr_regs[4] | (uint16_t)gyr_regs[5] << 8);

//	rslt = bmi3_get_regs(BMI3_REG_TEMP_DATA, &temp, 2, &dev);
//	temper = (temp[0] | (uint16_t)temp[1] << 8);

	rslt = bmi3_get_regs(BMI3_REG_ACC_DATA_X, &acc_regs, 6, &dev);
	if (rslt != BMI3_OK)
	{
		PDEBUG("read acc register failed, error code: %d\r\n", rslt);
		return rslt;
	}
	acc_data[0] = (acc_regs[0] | (uint16_t)acc_regs[1] << 8);
	acc_data[1] = (acc_regs[2] | (uint16_t)acc_regs[3] << 8);
	acc_data[2] = (acc_regs[4] | (uint16_t)acc_regs[5] << 8);

	gyr_x = lsb_to_dps(gyro_data[0], (float)2000, dev.resolution);
	gyr_y = lsb_to_dps(gyro_data[1], (float)2000, dev.resolution);
	gyr_z = lsb_to_dps(gyro_data[2], (float)2000, dev.resolution);

	acc_x = lsb_to_mps2(acc_data[0], 2, dev.resolution);
	acc_y = lsb_to_mps2(acc_data[1], 2, dev.resolution);
	acc_z = lsb_to_mps2(acc_data[2], 2, dev.resolution);

//	PDEBUG("Sensor Idx=%d Data=%X\r\n",index, tmp_regs[0]);
//	PDEBUG("GYRO: X axis: %4d, Y axis: %4d, Z axis: %4d\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
//	PDEBUG("ACC: X axis: %4d, Y axis: %4d, Z axis: %4d\r\n", acc_data[0], acc_data[1], acc_data[2]);
	PDEBUG("GYRO: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", gyr_x, gyr_y, gyr_z);
	PDEBUG("ACC: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", acc_x, acc_y, acc_z);
//	PDEBUG("Temperature = %d\r\n", (temper / 512 + 23));

	return 0;
}



void StartBMI323Task(void const * argument)
{
//	uint32_t notify_value;
	int8_t rslt = BMI3_OK;
	uint8_t flag;

	dev.read = (bmi3_read_fptr_t)SensorAPI_SPIx_Read1;
	dev.write = (bmi3_write_fptr_t)SensorAPI_SPIx_Write1;
	Init_BMI323(&dev);
	osDelay(100);
	dev2.read = (bmi3_read_fptr_t)SensorAPI_SPIx_Read2;
	dev2.write = (bmi3_write_fptr_t)SensorAPI_SPIx_Write2;
	Init_BMI323(&dev2);

//	BMI323_Print_ALLRegs(&dev);
//	BMI323_Print_ALLRegs(&dev2);

	for(;;)
	{
//		if(xTaskNotifyWait(0, 0, &notify_value, portMAX_DELAY))
//		{
			bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev);
			if((flag & 0x40) == 0) continue;
			PDEBUG("1:\n");
			read_sensor(dev);
			bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev2);
			if((flag & 0x40) == 0) continue;
			PDEBUG("2:\n");
			read_sensor(dev2);
			osDelay(500);
//		}
	}
}

