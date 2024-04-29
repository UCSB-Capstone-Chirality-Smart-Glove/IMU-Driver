#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include "stm32wbxx_hal.h"
#include "bmi323_task.h"
#include "bmi323.h"
#include "main.h"
#include "SPI_init.h"

extern SPI_HandleTypeDef hspi1;
uint8_t GTXBuffer[512], GRXBuffer[2048];

#define port1 GPIOA
#define pin1 GPIO_PIN_3
#define port2 GPIOA
#define pin2 GPIO_PIN_4
#define port3 GPIOA
#define pin3 GPIO_PIN_9
#define port4 GPIOA
#define pin4 GPIO_PIN_2

//#define port1 GPIOA
//#define pin1 GPIO_PIN_0
//#define port2 GPIOA
//#define pin2 GPIO_PIN_1
//#define port3 GPIOA
//#define pin3 GPIO_PIN_2
//#define port4 GPIOA
//#define pin4 GPIO_PIN_3
//#define port5 GPIOA
//#define pin5 GPIO_PIN_4
//#define port6 GPIOA
//#define pin6 GPIO_PIN_8
//#define port7 GPIOB
//#define pin7 GPIO_PIN_7
//#define port8 GPIOB
//#define pin8 GPIO_PIN_6
//#define port9 GPIOB
//#define pin9 GPIO_PIN_5
//#define port10 GPIOB
//#define pin10 GPIO_PIN_4
//#define port11 GPIOC
//#define pin11 GPIO_PIN_12

int8_t SensorAPI_SPIx_Read1(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET); // NSS low

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

    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET); // NSS high
    memcpy(reg_data, GRXBuffer+1, length);

    return 0;
}

int8_t SensorAPI_SPIx_Read3(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(port3, pin3, GPIO_PIN_RESET); // NSS low

    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port3, pin3, GPIO_PIN_SET); // NSS high
    memcpy(reg_data, GRXBuffer+1, length);

    return 0;
}

int8_t SensorAPI_SPIx_Read4(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(port4, pin4, GPIO_PIN_RESET); // NSS low

    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port4, pin4, GPIO_PIN_SET); // NSS high
    memcpy(reg_data, GRXBuffer+1, length);

    return 0;
}

//int8_t SensorAPI_SPIx_Read5(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr | 0x80;
//
//    HAL_GPIO_WritePin(port5, pin5, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port5, pin5, GPIO_PIN_SET); // NSS high
//    memcpy(reg_data, GRXBuffer+1, length);
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Read6(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr | 0x80;
//
//    HAL_GPIO_WritePin(port6, pin6, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port6, pin6, GPIO_PIN_SET); // NSS high
//    memcpy(reg_data, GRXBuffer+1, length);
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Read7(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr | 0x80;
//
//    HAL_GPIO_WritePin(port7, pin7, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port7, pin7, GPIO_PIN_SET); // NSS high
//    memcpy(reg_data, GRXBuffer+1, length);
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Read8(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr | 0x80;
//
//    HAL_GPIO_WritePin(port8, pin8, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port8, pin8, GPIO_PIN_SET); // NSS high
//    memcpy(reg_data, GRXBuffer+1, length);
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Read9(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr | 0x80;
//
//    HAL_GPIO_WritePin(port9, pin9, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port9, pin9, GPIO_PIN_SET); // NSS high
//    memcpy(reg_data, GRXBuffer+1, length);
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Read10(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr | 0x80;
//
//    HAL_GPIO_WritePin(port10, pin10, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port10, pin10, GPIO_PIN_SET); // NSS high
//    memcpy(reg_data, GRXBuffer+1, length);
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Read11(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr | 0x80;
//
//    HAL_GPIO_WritePin(port11, pin11, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port11, pin11, GPIO_PIN_SET); // NSS high
//    memcpy(reg_data, GRXBuffer+1, length);
//
//    return 0;
//}


int8_t SensorAPI_SPIx_Write1(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET); // NSS low

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

    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET); // NSS high

    return 0;
}

int8_t SensorAPI_SPIx_Write3(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(port3, pin3, GPIO_PIN_RESET); // NSS low

    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port3, pin3, GPIO_PIN_SET); // NSS high

    return 0;
}

int8_t SensorAPI_SPIx_Write4(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(port4, pin4, GPIO_PIN_RESET); // NSS low

    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(port4, pin4, GPIO_PIN_SET); // NSS high

    return 0;
}

//int8_t SensorAPI_SPIx_Write5(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr & 0x7F;
//    memcpy(&GTXBuffer[1], reg_data, length);
//
//    HAL_GPIO_WritePin(port5, pin5, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port5, pin5, GPIO_PIN_SET); // NSS high
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Write6(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr & 0x7F;
//    memcpy(&GTXBuffer[1], reg_data, length);
//
//    HAL_GPIO_WritePin(port6, pin6, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port6, pin6, GPIO_PIN_SET); // NSS high
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Write7(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr & 0x7F;
//    memcpy(&GTXBuffer[1], reg_data, length);
//
//    HAL_GPIO_WritePin(port7, pin7, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port7, pin7, GPIO_PIN_SET); // NSS high
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Write8(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr & 0x7F;
//    memcpy(&GTXBuffer[1], reg_data, length);
//
//    HAL_GPIO_WritePin(port8, pin8, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port8, pin8, GPIO_PIN_SET); // NSS high
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Write9(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr & 0x7F;
//    memcpy(&GTXBuffer[1], reg_data, length);
//
//    HAL_GPIO_WritePin(port9, pin9, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port9, pin9, GPIO_PIN_SET); // NSS high
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Write10(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr & 0x7F;
//    memcpy(&GTXBuffer[1], reg_data, length);
//
//    HAL_GPIO_WritePin(port10, pin10, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port10, pin10, GPIO_PIN_SET); // NSS high
//
//    return 0;
//}
//
//int8_t SensorAPI_SPIx_Write11(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//    GTXBuffer[0] = reg_addr & 0x7F;
//    memcpy(&GTXBuffer[1], reg_data, length);
//
//    HAL_GPIO_WritePin(port11, pin11, GPIO_PIN_RESET); // NSS low
//
//    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
//    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//
//    HAL_GPIO_WritePin(port11, pin11, GPIO_PIN_SET); // NSS high
//
//    return 0;
//}
