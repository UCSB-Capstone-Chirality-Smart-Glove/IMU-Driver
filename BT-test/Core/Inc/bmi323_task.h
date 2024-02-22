#ifndef _BMI323_TASK_H
#define _BMI323_TASK_H

#include <stdint.h>
#include "bmi3_defs.h"

#define READ_WRITE_LEN UINT8_C(32)
#define GRAVITY_EARTH (9.80665f)
#define UART_HANDLE (huart1)
#define BUS_TIMEOUT 1000

void bst_delay_us(uint32_t period, void *intf_ptr);

void UART_Printf(uint8_t* buff, uint16_t size);
void PDEBUG(char *format, ...);
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt);

int8_t Open_BMI323_ACC(struct bmi3_dev *dev);
int8_t Close_BMI323_ACC(struct bmi3_dev *dev);
int8_t Open_BMI323_GYRO(struct bmi3_dev *dev);
int8_t Close_BMI323_GYRO(struct bmi3_dev *dev);

void BMI323_Print_ALLRegs(struct bmi3_dev *dev);
int8_t Init_BMI323(struct bmi3_dev *dev);

uint8_t read_sensor(struct bmi3_dev dev, float data[]);

#endif
