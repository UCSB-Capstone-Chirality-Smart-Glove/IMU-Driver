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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* handData */
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
#define NOTIFICATION_INTERVAL_MS 10
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* handData */
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
    static uint32_t charValue2 = 0;
    static uint32_t charValue3 = 0;
    static uint32_t charValue4 = 0;
    static uint32_t charValue5 = 0;

    uint32_t currentTick = HAL_GetTick();

    if (currentTick - lastNotificationTime >= NOTIFICATION_INTERVAL_MS) {
        if (UpdateCharData[0] == 180) {
            UpdateCharData[0] = 0;
        } else {
            UpdateCharData[0] += 5;
        }
        Custom_F1_Update_Char();

        charValue2 += 1;
        UpdateCharData2[0] = (charValue2 >> 24) & 0xFF;
        UpdateCharData2[1] = (charValue2 >> 16) & 0xFF;
        UpdateCharData2[2] = (charValue2 >> 8) & 0xFF;
        UpdateCharData2[3] = charValue2 & 0xFF;
        Custom_F2_Update_Char();

        charValue3 += 2;
        UpdateCharData3[0] = (charValue3 >> 24) & 0xFF;
        UpdateCharData3[1] = (charValue3 >> 16) & 0xFF;
        UpdateCharData3[2] = (charValue3 >> 8) & 0xFF;
        UpdateCharData3[3] = charValue3 & 0xFF;
        Custom_F3_Update_Char();

        charValue4 += 3;
        UpdateCharData4[0] = (charValue4 >> 24) & 0xFF;
        UpdateCharData4[1] = (charValue4 >> 16) & 0xFF;
        UpdateCharData4[2] = (charValue4 >> 8) & 0xFF;
        UpdateCharData4[3] = charValue4 & 0xFF;
        Custom_F4_Update_Char();

        charValue5 += 4;
        UpdateCharData5[0] = (charValue5 >> 24) & 0xFF;
        UpdateCharData5[1] = (charValue5 >> 16) & 0xFF;
        UpdateCharData5[2] = (charValue5 >> 8) & 0xFF;
        UpdateCharData5[3] = charValue5 & 0xFF;
        Custom_Tdata_Update_Char();

        lastNotificationTime = currentTick;
    }

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

    /* handData */
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

/* handData */
void Custom_F1_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN F1_UC_1*/

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
