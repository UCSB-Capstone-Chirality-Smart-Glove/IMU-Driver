#ifndef _BMI323_TASK_H
#define _BMI323_TASK_H

#include <stdint.h>
#include "bmi3_defs.h"
#include "timers.h"


/*! Macro that defines read write length */
#define READ_WRITE_LEN     UINT8_C(32)
/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*! common porting */
#define UART_HANDLE (huart1)
#define SPI_HANDLE (hspi1)

#define BUS_TIMEOUT 1000

void bst_delay_us(uint32_t period, void *intf_ptr);
void UART_Printf(uint8_t* buff, uint16_t size);
void PDEBUG(char *format, ...);

int8_t SensorAPI_SPIx_Read1(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Read2(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Write1(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Write2(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
/*! End */

void bmi3_error_codes_print_result(const char api_name[], int8_t rslt);

int8_t Open_BMI323_ACC(struct bmi3_dev *dev);
int8_t Close_BMI323_ACC(struct bmi3_dev *dev);
int8_t Open_BMI323_GYRO(struct bmi3_dev *dev);
int8_t Close_BMI323_GYRO(struct bmi3_dev *dev);


void BMI323_Print_ALLRegs(struct bmi3_dev *dev);
int8_t Init_BMI323(struct bmi3_dev *dev);
void StartBMI323Task(void const * argument);

void BMI323_Timer_Callback(TimerHandle_t xTimer);


uint8_t read_sensor(struct bmi3_dev dev);

#endif
