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
uint8_t handPacket[12];
uint8_t UpdateCharData2[4];
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

uint16_t dataI1[] = {0,0,0,0,0,0};
uint16_t dataI2[] = {0,0,0,0,0,0};
uint16_t dataI3[] = {0,0,0,0,0,0};
uint16_t dataI4[] = {0,0,0,0,0,0};

extern struct bmi3_dev dev, dev2, dev3, dev4; // dev5, dev6, dev7, dev8, dev9, dev10, dev11;

// Flex Sensor Data
float flexData[] = {0,0,0,0};

// Finger Data Calculation
extern Finger finger;
extern vec3 hand_basis[3];
extern rotation_vec3 rotation_data;
extern rotation_vec3 hand_rotation_data;
extern FingerSensorData finger_sensor_data[4];
const int ACTIVE_FINGERS = 1;

const unsigned long CYCLES_PER_MS = 64000;
unsigned long t1;
unsigned long t2;
unsigned long diff;
int frequency = 1000/(NOTIFICATION_INTERVAL_MS);
uint8_t flag;
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

	/* Flex Sensor data */
	getFlexData(flexData);
	PDEBUG("A0: %f, A1: %f, A2: %f, A3: %f\r\n", flexData[0], flexData[1], flexData[2], flexData[3]);
	HAL_Delay(100);

	/* IMU grab data */
//	bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev);
//	while((flag & 0x40) == 0) break;
//	read_sensor(dev, data1, dataI1);
//
//	bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev3);
//	while((flag & 0x40) == 0) break;
//	read_sensor(dev3, data3, dataI3);
//
//	bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev4);
//	while((flag & 0x40) == 0) break;
//	read_sensor(dev4, data4, dataI4);
//	HAL_Delay(10);

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
    	bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev2);
    	while((flag & 0x40) == 0) break;
    	read_sensor(dev, data1, dataI1);
    	read_sensor(dev2, data2, dataI2);

    	for (int i = 0; i < ACTIVE_FINGERS; i++) {

    	}
    	finger_sensor_data.base.roll = data1[0];
    	finger_sensor_data.base.pitch = data1[1];
    	finger_sensor_data.base.yaw = data1[2];

    	finger_sensor_data.tip.roll = data2[0];
    	finger_sensor_data.tip.pitch = data2[1];
    	finger_sensor_data.tip.yaw = data2[2];

    	hand_rotation_data.roll = 0;
    	hand_rotation_data.pitch = 0;
    	hand_rotation_data.yaw = 0;

    	charValue2 = (int16_t)finger.bend;
    	charValue3 = (int16_t)finger.curl;
    	charValue4 = 0;
    	charValue5 = 0;

    	update_finger(&finger, &finger_sensor_data, frequency, hand_rotation_data);
    	PDEBUG("Bend: %d\n", (int)finger.bend);
    	PDEBUG("Curl: %d\n", (int)finger.curl);

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

void populateHandPacket(uint8_t *buffer, uint16_t basisVectorX, uint16_t basisVectorY,
                        uint16_t basisVectorZ, uint8_t flexSensorPalm, uint8_t flexSensorThumbWeb) {
    buffer[0] = (basisVectorX >> 8) & 0xFF;
    buffer[1] = basisVectorX & 0xFF;
    buffer[2] = (basisVectorY >> 8) & 0xFF;
    buffer[3] = basisVectorY & 0xFF;
    buffer[4] = (basisVectorZ >> 8) & 0xFF;
    buffer[5] = basisVectorZ & 0xFF;
    buffer[6] = flexSensorPalm;
    buffer[7] = flexSensorThumbWeb;
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
    Custom_STM_App_Update_Char(CUSTOM_STM_F1, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN F1_UC_Last*/

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
    Custom_STM_App_Update_Char(CUSTOM_STM_F2, (uint8_t *)UpdateCharData2);
  }

  /* USER CODE BEGIN F2_UC_Last*/

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
