#include "ADS1115.h"

#define ADS1115_ADDRESS 0x48
unsigned char ADSwrite[6];
int16_t flexRawData[] = {0,0,0,0};
float vi[] = {5000,0,0,0};

extern I2C_HandleTypeDef hi2c1;

void getFlexData(float flexData[]){
	for(int i=0; i< 4; i++){
		ADSwrite[0] = 0x01;

		switch(i){
			case(0):
				ADSwrite[1] = 0b11000101; //11000001
			break;
			case(1):
				ADSwrite[1] = 0b11010101; //11010001
			break;
			case(2):
				ADSwrite[1] = 0b11100101;
			break;
			case(3):
				ADSwrite[1] = 0b11110101;
			break;
		}

		ADSwrite[2] = 0b11110011; //10000011 LSB

		HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);
		ADSwrite[0] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1 , ADSwrite, 1 ,100);
		HAL_Delay(20);

		HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS <<1, ADSwrite, 2, 100);
		flexRawData[i] = (ADSwrite[0] << 8 | ADSwrite[1] );
		flexData[i] = (flexRawData[i] + vi[i]) / (2 * vi[i]);
	}
}
