#ifndef _INIT_H
#define _INIT_H

#include <stdint.h>
#include "bmi3_defs.h"

#define SPI_HANDLE (hspi1)

int8_t SensorAPI_SPIx_Read1(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Read2(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read3(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read4(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read5(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read6(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read7(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read8(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read9(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read10(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Read11(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

int8_t SensorAPI_SPIx_Write1(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Write2(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write3(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write4(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write5(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write6(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write7(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write8(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write9(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write10(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
//int8_t SensorAPI_SPIx_Write11(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);


#endif
