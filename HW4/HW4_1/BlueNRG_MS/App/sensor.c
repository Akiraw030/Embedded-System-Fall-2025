/**
  ******************************************************************************
  * @file    App/sensor.c
  * @author  SRA Application Team
  * @brief   Sensor init and sensor state machines
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sensor.h"
#include "gatt_db.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "hci_le.h"
#include "hci_const.h"
#include "bluenrg_aci_const.h"
#include "bluenrg_gatt_aci.h"
#include "app_bluenrg_ms.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  ADV_INTERVAL_MIN_MS  1000
#define  ADV_INTERVAL_MAX_MS  1200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t bdaddr[BDADDR_SIZE];
extern uint8_t bnrg_expansion_board;
__IO uint8_t set_connectable = 1;
__IO uint16_t connection_handle = 0;
__IO uint8_t  notification_enabled = FALSE;
__IO uint32_t connected = FALSE;

extern uint16_t EnvironmentalCharHandle;
extern uint16_t AccGyroMagCharHandle;

extern volatile uint8_t g_is_connected;
extern volatile uint16_t g_sampling_period_ms;
extern uint16_t Acc_Freq_Char_Handle;
extern uint16_t Acc_Data_Char_Handle; // <-- ADD THIS LINE

volatile uint8_t request_free_fall_notify = FALSE;

AxesRaw_t x_axes = {0, 0, 0};
AxesRaw_t g_axes = {0, 0, 0};
AxesRaw_t m_axes = {0, 0, 0};
AxesRaw_t q_axes[SEND_N_QUATERNIONS] = {{0, 0, 0}};

/* Private function prototypes -----------------------------------------------*/
void GAP_DisconnectionComplete_CB(void);
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : Set_DeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_DeviceConnectable(void)
{
  uint8_t ret;
  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,SENSOR_DEMO_NAME};

  uint8_t manuf_data[26] = {
    2,0x0A,0x00, /* 0 dBm */  // Transmission Power
    8,0x09,SENSOR_DEMO_NAME,  // Complete Name
    13,0xFF,0x01, /* SKD version */
    0x80,
    0x00,
    0xF4, /* ACC+Gyro+Mag 0xE0 | 0x04 Temp | 0x10 Pressure */
    0x00, /*  */
    0x00, /*  */
    bdaddr[5], /* BLE MAC start -MSB first- */
    bdaddr[4],
    bdaddr[3],
    bdaddr[2],
    bdaddr[1],
    bdaddr[0]  /* BLE MAC stop */
  };

  manuf_data[18] |= 0x01; /* Sensor Fusion */

  hci_le_set_scan_resp_data(0, NULL);

  printf("Set General Discoverable Mode.\n");

  ret = aci_gap_set_discoverable(ADV_DATA_TYPE,
                                (ADV_INTERVAL_MIN_MS*1000)/625,(ADV_INTERVAL_MAX_MS*1000)/625,
                                 STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);

  aci_gap_update_adv_data(26, manuf_data);

  if(ret != BLE_STATUS_SUCCESS)
  {
	  printf("aci_gap_set_discoverable() failed: 0x%02x\r\n", ret);
  }
  else
	  printf("aci_gap_set_discoverable() --> SUCCESS\r\n");
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 * parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 * parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void user_notify(void * pData)
{
  hci_uart_pckt *hci_pckt = pData;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data; // Base pointer

  if(hci_pckt->type != HCI_EVENT_PKT)
    return;

  // Print EVERY event code received
  PRINTF("user_notify: Received HCI_EVENT_PKT, Event Code: 0x%02X\r\n", event_pckt->evt);

  switch(event_pckt->evt){

  case EVT_DISCONN_COMPLETE: // Event Code 0x05
    {
      PRINTF("user_notify: Processing EVT_DISCONN_COMPLETE\r\n"); // Add Print
      GAP_DisconnectionComplete_CB();
    }
    break;

  case EVT_LE_META_EVENT: // Event Code 0x3E
    {
      PRINTF("user_notify: Processing EVT_LE_META_EVENT\r\n"); // Add Print
      // Correct casting: points to data *after* plen field
      evt_le_meta_event *evt = (void *)(event_pckt->data);
      PRINTF("user_notify: LE Meta Subevent Code: 0x%02X\r\n", evt->subevent); // Add Print

      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE: // Subevent Code 0x01
        {
          PRINTF("user_notify: Processing EVT_LE_CONN_COMPLETE subevent\r\n"); // Add Print
          // Correct casting: points to data *after* subevent field
          evt_le_connection_complete *cc = (void *)(evt->data);
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle); // Call the function to set flag
        }
        break;
      // Add other subevent cases if needed for debugging
      default:
        PRINTF("user_notify: Unhandled LE Meta Subevent: 0x%02X\r\n", evt->subevent); // Add Print
        break;
      }
    }
    break;

  case EVT_VENDOR: // Event Code 0xFF
    {
      PRINTF("user_notify: Processing EVT_VENDOR\r\n"); // Add Print
      // Correct casting: points to data *after* plen field
      evt_blue_aci *blue_evt = (void*)(event_pckt->data);
      PRINTF("user_notify: Vendor Specific Event Code: 0x%04X\r\n", blue_evt->ecode); // Add Print

      switch(blue_evt->ecode){

      case EVT_BLUE_GATT_READ_PERMIT_REQ: // Vendor Code 0x0C14
        {
          PRINTF("user_notify: Processing EVT_BLUE_GATT_READ_PERMIT_REQ\r\n"); // Add Print
          // Correct casting: points to data *after* ecode field
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data;
          Read_Request_CB(pr->attr_handle);
        }
        break;

      // --- START: FINAL CORRECTED BLOCK ---
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED: // Vendor Code 0x0C01
        {
          PRINTF("user_notify: Processing EVT_BLUE_GATT_ATTRIBUTE_MODIFIED\r\n"); // Add Print
          // CORRECT STRUCT TYPE from bluenrg_gatt_aci.h
          evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;

          // Check if the handle that was written to is our Frequency Characteristic
          // Use the correct member name 'attr_handle'
          if (evt->attr_handle == Acc_Freq_Char_Handle + 1) // Should be 37 based on previous logs
          {
            PRINTF("user_notify: Attribute modified is Acc_Freq_Char_Handle+1 (%d)\r\n", evt->attr_handle);
            // Use the correct member name 'data_length'
            if (evt->data_length == 2)
            {
               // Use the correct member name 'att_data'
               uint16_t new_period = (uint16_t)evt->att_data[0] | ((uint16_t)evt->att_data[1] << 8);
               if (new_period < 50) new_period = 50;
               if (new_period > 5000) new_period = 5000;
               g_sampling_period_ms = new_period;
               PRINTF("user_notify: Updated g_sampling_period_ms to %d\r\n", g_sampling_period_ms);
            } else {
               PRINTF("user_notify: Incorrect data length (%d) for Freq Char write\r\n", evt->data_length);
            }
          }
          // Check for Data Characteristic CCCD Write
          // Use the correct member name 'attr_handle'
          else if (evt->attr_handle == Acc_Data_Char_Handle + 2) // Should be 35 based on previous logs
          {
              PRINTF("user_notify: Attribute modified is Acc_Data_Char CCCD! (Handle %d)\r\n", evt->attr_handle);
              // Use the correct member name 'data_length' and 'att_data'
              if (evt->data_length == 2 && evt->att_data[0] == 0x01 && evt->att_data[1] == 0x00) {
                  PRINTF("user_notify: ==> Notifications ENABLED by client!\r\n");
                  notification_enabled = TRUE; // Optional: use this flag if needed elsewhere
              } else if (evt->data_length == 2 && evt->att_data[0] == 0x00 && evt->att_data[1] == 0x00) {
                  PRINTF("user_notify: ==> Notifications DISABLED by client!\r\n");
                  notification_enabled = FALSE;
              } else {
                  PRINTF("user_notify: Unexpected value written to CCCD (Len=%d, Val=0x%02X%02X)\r\n", evt->data_length, evt->att_data[1], evt->att_data[0]);
              }
          }
          else {
             // Use the correct member name 'attr_handle'
             PRINTF("user_notify: Attribute modified handle %d is NOT Acc_Freq_Char+1 (%d) or Acc_Data_Char CCCD (%d)\r\n",
                    evt->attr_handle, Acc_Freq_Char_Handle + 1, Acc_Data_Char_Handle + 2);
             // Potentially call original demo's handler if needed
             // Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data); // Example name
          }
        }
        break;
      // --- END: FINAL CORRECTED BLOCK ---

      case EVT_BLUE_GATT_TX_POOL_AVAILABLE: // Vendor Code 0x0C16
              {
                  // You might need the specific struct evt_gatt_tx_pool_available if defined
                  // evt_gatt_tx_pool_available *tx_evt = (evt_gatt_tx_pool_available*)blue_evt->data;
                  // PRINTF("user_notify: TX_POOL_AVAILABLE! ConnHandle=%d, Buffers=%d\r\n", tx_evt->conn_handle, tx_evt->available_buffers);

                  // Simple version:
                  PRINTF("user_notify: Received EVT_BLUE_GATT_TX_POOL_AVAILABLE (Buffers are free!)\r\n");
              }
      break;
      // Add other vendor event cases if needed for debugging
      default:
         PRINTF("user_notify: Unhandled Vendor Specific Event: 0x%04X\r\n", blue_evt->ecode); // Add Print
         break;

      } // end switch(blue_evt->ecode)

    } // end case EVT_VENDOR
    break;

  // Add other standard event cases if needed for debugging
  default:
    PRINTF("user_notify: Unhandled Standard Event: 0x%02X\r\n", event_pckt->evt); // Add Print
    break;
  }
}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  printf("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;

  // === ADD THESE LINES ===
  g_is_connected = 0;
  g_sampling_period_ms = 1000; // Reset period to default
  // =======================
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
  connected = TRUE;
  connection_handle = handle;

  // === ADD THIS LINE ===
  g_is_connected = 1;
  printf("GAP_ConnectionComplete_CB: g_is_connected SET to 1\n"); // <-- ADD Confirmation
  // =======================

  printf("Connected to device:");
  for(uint32_t i = 5; i > 0; i--){
	  printf("%02X-", addr[i]);
  }
  printf("%02X\n", addr[0]);
}
