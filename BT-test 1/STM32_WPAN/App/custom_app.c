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
  uint8_t               F3_Notification_Status;
  uint8_t               F4_Notification_Status;
  uint8_t               Tdata_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOTIFICATION_INTERVAL_MS 1000
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
uint8_t UpdateCharData2[4];
uint8_t UpdateCharData3[4];
uint8_t UpdateCharData4[4];
uint8_t UpdateCharData5[4];

uint16_t charValue2 = 0;
uint16_t charValue3 = 0;
uint16_t charValue4 = 0;
uint16_t charValue5 = 0;

float data1[] = {0,0,0,0,0,0};
float data2[] = {0,0,0,0,0,0};
float data3[] = {0,0,0,0,0,0};
float data4[] = {0,0,0,0,0,0};

uint16_t dataI1[] = {0,0,0,0,0,0};
uint16_t dataI2[] = {0,0,0,0,0,0};
uint16_t dataI3[] = {0,0,0,0,0,0};
uint16_t dataI4[] = {0,0,0,0,0,0};

extern struct bmi3_dev dev, dev2, dev3, dev4; // dev5, dev6, dev7, dev8, dev9, dev10, dev11;

int8_t rslt = BMI3_OK;
uint8_t flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* smartGlove */
static void Custom_F1_Update_Char(void);
static void Custom_F1_Send_Notification(void);
static void Custom_F2_Update_Char(void);
static void Custom_F2_Send_Notification(void);
static void Custom_F3_Update_Char(void);
static void Custom_F3_Send_Notification(void);
static void Custom_F4_Update_Char(void);
static void Custom_F4_Send_Notification(void);
static void Custom_Tdata_Update_Char(void);
static void Custom_Tdata_Send_Notification(void);

/* USER CODE BEGIN PFP */
void myTask(void)
{
	/* IMU grab data */
//	bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev);
//	while((flag & 0x40) == 0) break;
//	read_sensor(dev, data1, dataI1);
//
	bmi3_get_regs(BMI3_REG_STATUS, &flag, 1, &dev2);
	while((flag & 0x40) == 0) break;
	read_sensor(dev2, data2, dataI2);
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
	charValue2 = dataI2[0];
	charValue3 = dataI2[1];
	charValue4 = dataI2[2];
	charValue5 = dataI2[3];

    uint32_t currentTick = HAL_GetTick();

    if (currentTick - lastNotificationTime >= NOTIFICATION_INTERVAL_MS) {
        if (UpdateCharData[0] == 180) {
            UpdateCharData[0] = 0;
        } else {
            UpdateCharData[0] += 1;
        }
        Custom_F1_Update_Char();

        UpdateCharData2[0] = (charValue2 >> 8) & 0xFF;
        UpdateCharData2[1] = charValue2 & 0xFF;
        Custom_F2_Update_Char();

        UpdateCharData3[0] = (charValue3 >> 8) & 0xFF;
        UpdateCharData3[1] = charValue3 & 0xFF;
        Custom_F3_Update_Char();

        UpdateCharData4[0] = (charValue4 >> 8) & 0xFF;
        UpdateCharData4[1] = charValue4 & 0xFF;
        Custom_F4_Update_Char();

        UpdateCharData5[0] = (charValue5 >> 8) & 0xFF;
        UpdateCharData5[1] = charValue5 & 0xFF;
        Custom_Tdata_Update_Char();

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

/*void myTask2(void)
{
    uint32_t currentTick = HAL_GetTick();

    // Check if enough time has elapsed for the next notification
    if (currentTick - lastNotificationTime >= NOTIFICATION_INTERVAL_MS) {
        // Toggle between 90 and 180
        if (UpdateCharData2[0] == 0) {
            UpdateCharData2[0] = 100;
            UpdateCharData2[1] = 100;
        } else {
        	UpdateCharData2[0] = UpdateCharData2[0] + 1;
        	UpdateCharData2[1] = UpdateCharData2[1] + 1;
        }

        Custom_Fpos_Update_Char();
        lastNotificationTime = currentTick;
    }

    UTIL_SEQ_SetTask(1<<CFG_TASK_SEND_NOTIF2, CFG_SCH_PRIO_0);
}
void myTask3(void)
{
    uint32_t currentTick = HAL_GetTick();

    // Check if enough time has elapsed for the next notification
    if (currentTick - lastNotificationTime >= NOTIFICATION_INTERVAL_MS) {
        // Toggle between 90 and 180
        if (UpdateCharData3[0] == 180) {
            UpdateCharData3[0] = 0;
            UpdateCharData3[1] = 0;
        } else {
        	UpdateCharData3[0] = UpdateCharData3[0] + 5;
        	UpdateCharData3[1] = UpdateCharData3[0] + 5;
        }

        Custom_Fbend2_Update_Char();
        lastNotificationTime = currentTick;
    }


    UTIL_SEQ_SetTask(1<<CFG_TASK_SEND_NOTIF3, CFG_SCH_PRIO_0);
}

void myTask4(void)
{
    uint32_t currentTick = HAL_GetTick();

    // Check if enough time has elapsed for the next notification
    if (currentTick - lastNotificationTime >= NOTIFICATION_INTERVAL_MS) {
        // Toggle between 90 and 180
        if (UpdateCharData4[0] == 0) {
            UpdateCharData4[0] = 100;
            UpdateCharData4[1] = 100;
        } else {
        	UpdateCharData4[0] = UpdateCharData4[0] + 1;
        	UpdateCharData4[1] = UpdateCharData4[1] + 1;
        }

        Custom_Fpos2_Update_Char();
        lastNotificationTime = currentTick;
    }

    UTIL_SEQ_SetTask(1<<CFG_TASK_SEND_NOTIF4, CFG_SCH_PRIO_0);
}

void myTask5(void)
{
    uint32_t currentTick = HAL_GetTick();

    // Check if enough time has elapsed for the next notification
    if (currentTick - lastNotificationTime >= NOTIFICATION_INTERVAL_MS) {
        // Toggle between 90 and 180
        if (UpdateCharData5[0] == 180) {
            UpdateCharData5[0] = 0;
            UpdateCharData5[1] = 0;
        } else {
        	UpdateCharData5[0] = UpdateCharData5[0] + 5;
        	UpdateCharData5[1] = UpdateCharData5[0] + 5;
        }

        Custom_Tdata_Update_Char();
        lastNotificationTime = currentTick;
    }


    UTIL_SEQ_SetTask(1<<CFG_TASK_SEND_NOTIF5, CFG_SCH_PRIO_0);
}
 */
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

    case CUSTOM_STM_F3_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_F3_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_F3_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_F3_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_F3_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_F3_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_F4_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_F4_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_F4_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_F4_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_F4_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_F4_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_TDATA_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TDATA_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_TDATA_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_TDATA_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TDATA_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_TDATA_NOTIFY_DISABLED_EVT */
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
    Custom_STM_App_Update_Char(CUSTOM_STM_F2, (uint8_t *)UpdateCharData);
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

void Custom_F3_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F3_UC_1*/
  updateflag = 1;
  /* USER CODE END F3_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_F3, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN F3_UC_Last*/

  /* USER CODE END F3_UC_Last*/
  return;
}

void Custom_F3_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F3_NS_1*/

  /* USER CODE END F3_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_F3, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN F3_NS_Last*/

  /* USER CODE END F3_NS_Last*/

  return;
}

void Custom_F4_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F4_UC_1*/
  updateflag = 1;
  /* USER CODE END F4_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_F4, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN F4_UC_Last*/

  /* USER CODE END F4_UC_Last*/
  return;
}

void Custom_F4_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F4_NS_1*/

  /* USER CODE END F4_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_F4, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN F4_NS_Last*/

  /* USER CODE END F4_NS_Last*/

  return;
}

void Custom_Tdata_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Tdata_UC_1*/
  updateflag = 1;
  /* USER CODE END Tdata_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_TDATA, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Tdata_UC_Last*/

  /* USER CODE END Tdata_UC_Last*/
  return;
}

void Custom_Tdata_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Tdata_NS_1*/

  /* USER CODE END Tdata_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_TDATA, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Tdata_NS_Last*/

  /* USER CODE END Tdata_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
