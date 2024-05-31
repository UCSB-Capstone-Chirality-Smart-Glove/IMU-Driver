/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi323_task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* smartGlove */
  uint8_t               F1_Notification_Status;
  uint8_t               F2_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOTIFICATION_INTERVAL_MS 100
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
static uint32_t lastNotificationTime = 0;
uint8_t fingerPacket[14];
uint8_t handPacket[13];
uint8_t UpdateCharData2[12]; //Put Second Char data here, Put first finger packet info in UpdateCharData[]
uint8_t UpdateCharData3[4];
uint8_t UpdateCharData4[4];
uint8_t UpdateCharData5[4];

uint16_t charValue2 = 0;
uint16_t charValue3 = 0;
uint16_t charValue4 = 0;
uint16_t charValue5 = 0;

// IMU data
float data1[] = {0,0,0,0,0,0};
float data2[] = {0,0,0,0,0,0};
float data3[] = {0,0,0,0,0,0};
float data4[] = {0,0,0,0,0,0};
float data5[] = {0,0,0,0,0,0};
float data6[] = {0,0,0,0,0,0};
float data7[] = {0,0,0,0,0,0};
float data8[] = {0,0,0,0,0,0};
float data9[] = {0,0,0,0,0,0};
float data10[] = {0,0,0,0,0,0};
float data11[] = {0,0,0,0,0,0};
float data[11][6];

uint16_t dataI1[] = {0,0,0,0,0,0};
uint16_t dataI2[] = {0,0,0,0,0,0};
uint16_t dataI3[] = {0,0,0,0,0,0};
uint16_t dataI4[] = {0,0,0,0,0,0};
uint16_t dataI5[] = {0,0,0,0,0,0};
uint16_t dataI6[] = {0,0,0,0,0,0};
uint16_t dataI7[] = {0,0,0,0,0,0};
uint16_t dataI8[] = {0,0,0,0,0,0};
uint16_t dataI9[] = {0,0,0,0,0,0};
uint16_t dataI10[] = {0,0,0,0,0,0};
uint16_t dataI11[] = {0,0,0,0,0,0};
uint16_t dataI[11][6];

extern struct bmi3_dev dev[11];

// Flex Sensor Data
float flexData[] = {0,0,0,0};

// Finger Data Calculation
extern Finger finger;
extern Hand hand;
extern vec3 hand_basis[3];
extern rotation_vec3 rotation_data;
extern IMUData hand_sensor_data;
extern FingerSensorData finger_sensor_data[5];
rotation_vec3 hand_rotation_data;
const int ACTIVE_FINGERS = 1;
extern ADC_HandleTypeDef hadc1;

const unsigned long CYCLES_PER_MS = 64000;
unsigned long t1;
unsigned long t2;
unsigned long diff;
int frequency = 1000/(NOTIFICATION_INTERVAL_MS);
uint8_t flag;
uint16_t raw;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* smartGlove */
static void Custom_F1_Update_Char(void);
static void Custom_F1_Send_Notification(void);
static void Custom_F2_Update_Char(void);
static void Custom_F2_Send_Notification(void);

/* USER CODE BEGIN PFP */
void myTask(void)
{
	/* On board ADC data */
	//	uint16_t raw;
	//	HAL_ADC_Start(&hadc1);
	//	HAL_ADC_PollForConversion(&hadc1, 10);
	//	raw = HAL_ADC_GetValue(&hadc1);
	//	float raw1 = (float) (raw + 750) / 1500 * 180;
	//	PDEBUG("ADC: %f ", raw1);

	/* IMU grab data */
//    bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[0]);
//   	while((flag & 0x40) == 0) break;
//   	read_sensor(dev[0], data1, dataI1);

//    bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[1]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[1], data2, dataI2);

// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[2]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[2], data3, dataI3);
//
// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[3]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[3], data4, dataI4);

// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[4]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[4], data5, dataI5);

// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[5]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[5], data6, dataI6);

// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[6]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[6], data7, dataI7);

// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[7]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[7], data8, dataI8);

// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[8]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[8], data9, dataI9);

// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[9]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[9], data10, dataI10);

// 		bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[10]);
// 		while((flag & 0x40) == 0) break;
// 		read_sensor(dev[10], data11, dataI11);
//	  HAL_Delay(10);

	/* BLE transfer */
//	rotation_vec3 angles = matrix_to_euler(finger.basis);

//	charValue2 = finger.bend;
//	charValue3 = finger.bend;
//	charValue4 = finger.bend;
//	charValue5 = finger.bend;

//	PDEBUG("Bend: %d\n", (int)finger.bend);
//	PDEBUG("Curl: %d\n", (int)finger.curl);

//	charValue2 = dataI2[0];
//	charValue3 = dataI2[1];
//	charValue4 = dataI2[2];
//	charValue5 = dataI2[3];

    uint32_t currentTick = HAL_GetTick();

    if (currentTick - lastNotificationTime >= NOTIFICATION_INTERVAL_MS) {
    	bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev[3]);
    	while((flag & 0x40) == 0) break;
//    	read_sensor(dev[0], data1, dataI1);
//    	read_sensor(dev[1], data2, dataI2);
//    	read_sensor(dev[2], data3, dataI3);
    	for (int i = 0; i < 11; i++) {
    		read_sensor(dev[i], data[i], dataI[i]);
    	}


    	/* Flex Sensor data */
//    	getFlexData(flexData);
//    	PDEBUG("A0: %f, A1: %f, A2: %f, A3: %f\r\n", flexData[0], flexData[1], flexData[2], flexData[3]);
//    	HAL_Delay(100);

    	for (int i = 0; i < 10; i+=2) {
        	finger_sensor_data[i/2] = (FingerSensorData){
    			.base = (IMUData){
    				.gyro = (rotation_vec3) {
    					.pitch = data[i][0],
    					.roll = data[i][1],
    					.yaw = data[i][2]
    				},
    				.accel = (vec3) {
    					.x = data[i][3],
    					.y = data[i][4],
    					.z = data[i][5]
    				}
    			},
    			.tip = (IMUData) {
    				.gyro = (rotation_vec3) {
    					.pitch = data[i+1][0],
    					.roll = data[i+1][1],
    					.yaw = data[i+1][2]
    				},
    				.accel = (vec3) {
    					.x = data[i+1][3],
    					.y = data[i+1][4],
    					.z = data[i+1][5]
    				}
    			}
        	};
    	}

    	hand_sensor_data.gyro.pitch = data[10][0];
    	hand_sensor_data.gyro.roll = data[10][1];
    	hand_sensor_data.gyro.yaw = data[10][2];

    	hand_sensor_data.accel.x = data[10][3];
    	hand_sensor_data.accel.y = data[10][4];
    	hand_sensor_data.accel.z = data[10][5];

    	charValue2 = (int16_t)finger.bend;
    	charValue3 = (int16_t)finger.curl;
    	charValue4 = 0;
    	charValue5 = 0;

    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, 10);
        raw = HAL_ADC_GetValue(&hadc1);

        float data = ((float)raw/1500)*180;
        PDEBUG("ADC: %f \r\n", data);

//    	update_finger(&finger, &finger_sensor_data, frequency, hand_rotation_data);
//    	PDEBUG("finger_sensor_data.x: %f\n", finger_sensor_data[0].base.accel.x);
    	update_hand(&hand, &hand_sensor_data, frequency, finger_sensor_data, flexData);
    	hand_rotation_data = matrix_to_euler(hand.basis);
    	PDEBUG("Bend: %d\n", (int)hand.finger[0]->bend);
    	PDEBUG("Curl: %d\n", (int)hand.finger[0]->curl);
    	PDEBUG("Roll: %d\n", (int)hand_rotation_data.roll);
    	PDEBUG("Pitch: %d\n", (int)hand_rotation_data.pitch);
    	PDEBUG("Yaw: %d\n", (int)hand_rotation_data.yaw);

    	populateFingerPacket(fingerPacket,
    			hand.finger[0]->curl, hand.finger[0]->bend,
				hand.finger[1]->curl, hand.finger[1]->bend,
				hand.finger[2]->curl, hand.finger[2]->bend,
				hand.finger[3]->curl, hand.finger[3]->bend,
				hand.finger[4]->curl, hand.finger[4]->bend);
    	populateHandPacket(handPacket, 0, 0, 0, 0,
    			hand.finger[0]->wag, hand.finger[1]->wag,
				hand.finger[2]->wag, hand.finger[3]->wag,
				hand.finger[4]->wag, 0);

        if (UpdateCharData[0] == 180) {
            UpdateCharData[0] = 0;
        } else {
            UpdateCharData[0] += 1;
        }
        Custom_F1_Update_Char();

        UpdateCharData2[0] = (charValue2 >> 8) & 0xFF;
        UpdateCharData2[1] = charValue2 & 0xFF;
        UpdateCharData2[2] = (charValue3 >> 8) & 0xFF;
        UpdateCharData2[3] = charValue3 & 0xFF;
        Custom_F2_Update_Char();

        UpdateCharData3[0] = (charValue3 >> 8) & 0xFF;
        UpdateCharData3[1] = charValue3 & 0xFF;

        UpdateCharData4[0] = (charValue4 >> 8) & 0xFF;
        UpdateCharData4[1] = charValue4 & 0xFF;


        UpdateCharData5[0] = (charValue5 >> 8) & 0xFF;
        UpdateCharData5[1] = charValue5 & 0xFF;


        lastNotificationTime = currentTick;
    }

    /* Data monitor */
//  PDEBUG("1:\n");
//	PDEBUG("GYRO: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", data1[0], data1[1], data1[2]);
//	PDEBUG("ACC: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", data1[3], data1[4], data1[5]);
//  PDEBUG("GYRO: X axis: %#16x, Y axis: %#16x, Z axis: %#16x\r\n", dataI1[0], dataI1[1], dataI1[2]);
//  PDEBUG("ACC: X axis: %#16x, Y axis: %#16x, Z axis: %#16x\r\n", dataI1[3], dataI1[4], dataI1[5]);

//	PDEBUG("2:\n");
//	PDEBUG("GYRO: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", data2[0], data2[1], data2[2]);
//	PDEBUG("ACC: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", data2[3], data2[4], data2[5]);
//	PDEBUG("GYRO: X axis: %#16x, Y axis: %#16x, Z axis: %#16x\r\n", dataI2[0], dataI2[1], dataI2[2]);
//	PDEBUG("ACC: X axis: %#16x, Y axis: %#16x, Z axis: %#16x\r\n", dataI2[3], dataI2[4], dataI2[5]);

//	PDEBUG("3:\n");
//	PDEBUG("GYRO: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", data3[0], data3[1], data3[2]);
//	PDEBUG("ACC: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", data3[3], data3[4], data3[5]);
//	PDEBUG("GYRO: X axis: %#16x, Y axis: %#16x, Z axis: %#16x\r\n", dataI3[0], dataI3[1], dataI3[2]);
//	PDEBUG("ACC: X axis: %#16x, Y axis: %#16x, Z axis: %#16x\r\n", dataI3[3], dataI3[4], dataI3[5]);

//	PDEBUG("4:\n");
//	PDEBUG("GYRO: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", data4[0], data4[1], data4[2]);
//	PDEBUG("ACC: X axis: %4.2f, Y axis: %4.2f, Z axis: %4.2f\r\n", data4[3], data4[4], data4[5]);
//	PDEBUG("GYRO: X axis: %#16x, Y axis: %#16x, Z axis: %#16x\r\n", dataI4[0], dataI4[1], dataI4[2]);
//	PDEBUG("ACC: X axis: %#16x, Y axis: %#16x, Z axis: %#16x\r\n", dataI4[3], dataI4[4], dataI4[5]);

//    HAL_Delay(1000);
    UTIL_SEQ_SetTask(1<<CFG_TASK_SEND_NOTIF, CFG_SCH_PRIO_0);
}

/* Helper Functions for Populating Bluetooth packets */
void populateFingerPacket(uint8_t *buffer, uint16_t finger1Curl, uint8_t finger1Bend,
                          uint16_t finger2Curl, uint8_t finger2Bend, uint16_t finger3Curl,
                          uint8_t finger3Bend, uint16_t finger4Curl, uint8_t finger4Bend,
                          uint16_t thumbCurl, uint8_t thumbBend) {
    buffer[0] = (finger1Curl >> 8) & 0xFF;
    buffer[1] = finger1Curl & 0xFF;
    buffer[2] = finger1Bend;
    buffer[3] = (finger2Curl >> 8) & 0xFF;
    buffer[4] = finger2Curl & 0xFF;
    buffer[5] = finger2Bend;
    buffer[6] = (finger3Curl >> 8) & 0xFF;
    buffer[7] = finger3Curl & 0xFF;
    buffer[8] = finger3Bend;
    buffer[9] = (finger4Curl >> 8) & 0xFF;
    buffer[10] = finger4Curl & 0xFF;
    buffer[11] = finger4Bend;
    buffer[12] = (thumbCurl >> 8) & 0xFF;
    buffer[13] = thumbCurl & 0xFF;
    buffer[14] = thumbBend;
}

void populateHandPacket(uint8_t *buffer, uint16_t basisVectorX,
						uint16_t basisVectorY, uint16_t basisVectorZ, uint8_t flexSensorPalm,
						uint8_t wag1, uint8_t wag2, uint8_t wag3,
						uint8_t wag4, uint8_t wag5, uint8_t moreData) {
	buffer[0] = (basisVectorX >> 8) & 0xFF;
	buffer[1] = basisVectorX & 0xFF;
	buffer[2] = (basisVectorY >> 8) & 0xFF;
	buffer[3] = basisVectorY & 0xFF;
	buffer[4] = (basisVectorZ >> 8) & 0xFF;
	buffer[5] = basisVectorZ & 0xFF;
	buffer[6] = flexSensorPalm;
	buffer[7] = wag1;
	buffer[8] = wag2;
	buffer[9] = wag3;
	buffer[10] = wag4;
	buffer[11] = wag5;
}

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* smartGlove */
    case CUSTOM_STM_F1_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_F1_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_F1_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_F1_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_F1_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_F1_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_F2_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_F2_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_F2_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_F2_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_F2_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_F2_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* smartGlove */
void Custom_F1_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F1_UC_1*/
  updateflag = 1;
  /* USER CODE END F1_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_F1, (uint8_t *)fingerPacket);
  }

  /* USER CODE BEGIN F1_UC_Last*/
  // Buffer Above is the one to be populated ______________^^^^^ this buffer
  // Change after every update to IOC
  /* USER CODE END F1_UC_Last*/
  return;
}

void Custom_F1_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F1_NS_1*/

  /* USER CODE END F1_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_F1, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN F1_NS_Last*/

  /* USER CODE END F1_NS_Last*/

  return;
}

void Custom_F2_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F2_UC_1*/
  updateflag = 1;
  /* USER CODE END F2_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_F2, (uint8_t *)handPacket);
  }

  /* USER CODE BEGIN F2_UC_Last*/
  // Buffer Above is the one to be populated ______________^^^^^ this buffer
  // Change after every update to IOC
  /* USER CODE END F2_UC_Last*/
  return;
}

void Custom_F2_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F2_NS_1*/

  /* USER CODE END F2_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_F2, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN F2_NS_Last*/

  /* USER CODE END F2_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
