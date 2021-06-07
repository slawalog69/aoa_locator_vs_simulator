/***************************************************************************//**
 * @file
 * @brief Connection oriented AoA locator application.
 *
 * AoA locator application for connection oriented implementation.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include "system.h"
#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"
#include "app_log.h"
#include "app_assert.h"

#include "conn.h"

#include "aoa.h"
#include "app.h"
#include "aoa_util.h"
#include "app_config.h"

// connection parameters
#define CONN_INTERVAL_MIN             80   //100ms
#define CONN_INTERVAL_MAX             80   //100ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  100  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff

#define CTE_TYPE_AOA                  0

// UUIDs defined by Bluetooth SIG
static const uint8_t cte_service[SERVICE_UUID_LEN] = { 0x50, 0x69, 0x96, 0x81,
                                                       0xb7, 0xa8, 0xad, 0x07,
                                                       0x96, 0xf2, 0x3f, 0x07,
                                                       0x64, 0x36, 0xd0, 0x0e };
static const uint8_t cte_enable_char[CHAR_UUID_LEN] = { 0xdd, 0xc4, 0xfb, 0xc9,
                                                        0xa0, 0x14, 0xd6, 0xcd,
                                                        0x1c, 0x10, 0xd6, 0x57,
                                                        0x72, 0x0b, 0x6a, 0x0d };

// Antenna switching pattern
static const uint8_t antenna_array[AOA_NUM_ARRAY_ELEMENTS] = SWITCHING_PATTERN;

/**************************************************************************//**
 * Connection specific Bluetooth event handler.
 *****************************************************************************/
void app_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  conn_properties_t* conn;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Set passive scanning on 1Mb PHY
      sc = sl_bt_scanner_set_mode(gap_1m_phy, SCAN_PASSIVE);

      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to set scanner mode\n",
                 (int)sc);

      // Set scan interval and scan window
      sc = sl_bt_scanner_set_timing(gap_1m_phy, SCAN_INTERVAL, SCAN_WINDOW);
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to set scanner timing\n",
                 (int)sc);

      // Set the default connection parameters for subsequent connections
      sc = sl_bt_connection_set_default_parameters(CONN_INTERVAL_MIN,
                                                   CONN_INTERVAL_MAX,
                                                   CONN_SLAVE_LATENCY,
                                                   CONN_TIMEOUT,
                                                   CONN_MIN_CE_LENGTH,
                                                   CONN_MAX_CE_LENGTH);
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to set parameters\n",
                 (int)sc);
      // Start scanning - looking for tags
      sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
      app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_INVALID_STATE,
                 "[E: 0x%04x] Failed to start scanner\n",
                 (int)sc);

      app_log("Start scanning...\n");

      break;

    case sl_bt_evt_scanner_scan_report_id:
      // Check if the tag is whitelisted
    {
      if (SL_STATUS_NOT_FOUND == aoa_whitelist_find(evt->data.evt_scanner_scan_report.address.addr)) {
        if (verbose_level > 0 ) {
          app_log("Tag is not on the whitelist, ignoring.\n");
        }

        break;
      }
      // Check for connectable advertising type
      if ((evt->data.evt_scanner_scan_report.packet_type & 0x06) != 0x0) {
        break;
      }
      // If a CTE service is found...
      if (find_service_in_advertisement(&(evt->data.evt_scanner_scan_report.data.data[0]),
                                        evt->data.evt_scanner_scan_report.data.len,
                                        (uint8_t *) cte_service) != 0) {
        conn = get_connection_by_address(&evt->data.evt_scanner_scan_report.address);
        if (!is_connection_list_full() && conn == NULL) {
          uint8_t conn_handle;
          sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                     evt->data.evt_scanner_scan_report.address_type,
                                     gap_1m_phy, &conn_handle);
          app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_INVALID_STATE,
                     "[E: 0x%04x] Failed to open connection\n",
                     (int)sc);
        }
      }
      break;
    }
    case sl_bt_evt_connection_opened_id:
      // Add connection to the connection_properties array
      add_connection((uint16_t)evt->data.evt_connection_opened.connection,
                     &evt->data.evt_connection_opened.address,
                     evt->data.evt_connection_opened.address_type);
      // Discover CTE service on the slave device
      sc = sl_bt_gatt_discover_primary_services_by_uuid(evt->data.evt_connection_opened.connection,
                                                        SERVICE_UUID_LEN,
                                                        cte_service);
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to discover primary services\n",
                 (int)sc);
      app_log("Connected to tag. Discovering services...\r\n");

      app_log("Tag's Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
              evt->data.evt_connection_opened.address_type ? "static random" : "public device",
              evt->data.evt_connection_opened.address.addr[5],
              evt->data.evt_connection_opened.address.addr[4],
              evt->data.evt_connection_opened.address.addr[3],
              evt->data.evt_connection_opened.address.addr[2],
              evt->data.evt_connection_opened.address.addr[1],
              evt->data.evt_connection_opened.address.addr[0]);

      break;

    // This event is generated when a new service is discovered
    case sl_bt_evt_gatt_service_id:
      // Find connection
      app_log("Got service handle\n");
      if ((conn = get_connection_by_handle(evt->data.evt_gatt_service.connection)) == NULL) {
        break;
      }
      // Save service handle for future reference
      if (memcmp((uint8_t*)&(evt->data.evt_gatt_service.uuid.data[0]),
                 cte_service, SERVICE_UUID_LEN) == 0) {
        conn->cte_service_handle = evt->data.evt_gatt_service.service;
      }
      break;

    // This event is generated when a new characteristic is discovered
    case sl_bt_evt_gatt_characteristic_id:
      // Find connection
      app_log("Got new characteristic\n");
      if ((conn = get_connection_by_handle(evt->data.evt_gatt_characteristic.connection)) == NULL) {
        break;
      }
      // Save characteristic handle for future reference
      if (memcmp((uint8_t*)&(evt->data.evt_gatt_characteristic.uuid.data[0]),
                 cte_enable_char, CHAR_UUID_LEN) == 0) {
        conn->cte_enable_char_handle = evt->data.evt_gatt_characteristic.characteristic;
      }
      conn->connection_state = DISCOVER_CHARACTERISTICS;
      break;

    // This event is generated for various procedure completions, e.g. when a
    // write procedure is completed, or service discovery is completed
    case sl_bt_evt_gatt_procedure_completed_id:
      // Find connection
      if ((conn = get_connection_by_handle(evt->data.evt_gatt_procedure_completed.connection)) == NULL) {
        break;
      }

      switch (conn->connection_state) {
        // If service discovery finished
        case DISCOVER_SERVICES:
          app_log("Service discovering finished.\n");
          // Discover CTE enable characteristic on the slave device
          sc = sl_bt_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
                                                           conn->cte_service_handle,
                                                           CHAR_UUID_LEN,
                                                           cte_enable_char);
          app_assert(sc == SL_STATUS_OK,
                     "[E: 0x%04x] Failed to discover characteristics\n",
                     (int)sc);
          break;
        // If characteristic discovery finished
        case DISCOVER_CHARACTERISTICS:
          app_log("Services discovered. Enabling CTE...\n");
          uint8_t data = 0x01;

          // enable CTE on slave device (by writing 0x01 into the CTE enable characteristic)
          sc = sl_bt_gatt_write_characteristic_value(evt->data.evt_gatt_procedure_completed.connection,
                                                     conn->cte_enable_char_handle,
                                                     1,
                                                     &data);
          app_assert(sc == SL_STATUS_OK,
                     "[E: 0x%04x] Failed to write characteristic\n",
                     (int)sc);
          conn->connection_state = ENABLE_CTE;

          break;
        case ENABLE_CTE:
          // If CTE was enabled
          app_log("CTE enabled. Start IQ sampling...\n");
          // The tag has started sending CTE packets. Start IQ sampling on the receiver side
          sc = sl_bt_cte_receiver_enable_connection_cte(evt->data.evt_gatt_procedure_completed.connection,
                                                        CTE_SAMPLING_INTERVAL,
                                                        CTE_MIN_LENGTH,
                                                        CTE_TYPE_AOA,
                                                        CTE_SLOT_DURATION,
                                                        sizeof(antenna_array),
                                                        antenna_array);
          app_assert(sc == SL_STATUS_OK,
                     "[E: 0x%04x] Failed to enable CTE\n",
                     (int)sc);
          // Restart the scanner to discover new tags
          app_log("Restart scanning.\n");
          sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
          app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_INVALID_STATE,
                     "[E: 0x%04x] Failed to start scanner\n",
                     (int)sc);
          conn->connection_state = RUNNING;
          break;
        default:
          break;
      }
      break;
    // This event is generated when a connection is dropped
    case sl_bt_evt_connection_closed_id:
      // remove connection from active connections
      app_log("Connection lost.\n");
      remove_connection((uint16_t)evt->data.evt_connection_closed.connection);
      // Restart the scanner to discover new tags
      sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
      app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_INVALID_STATE,
                 "[E: 0x%04x] Failed to start scanner\n",
                 (int)sc);

      break;

    case sl_bt_evt_cte_receiver_connection_iq_report_id:
    {
      aoa_iq_report_t iq_report;

      if (evt->data.evt_cte_receiver_connection_iq_report.samples.len == 0) {
        // Nothing to be processed.
        break;
      }

      // Check if asset tag is known.
      if ((conn = get_connection_by_handle(evt->data.evt_cte_receiver_connection_iq_report.connection)) == NULL) {
        break;
      }

      // Convert event to common IQ report format.
      iq_report.channel = evt->data.evt_cte_receiver_connection_iq_report.channel;
      iq_report.rssi = evt->data.evt_cte_receiver_connection_iq_report.rssi;
      iq_report.event_counter = evt->data.evt_cte_receiver_connection_iq_report.event_counter;
      iq_report.length = evt->data.evt_cte_receiver_connection_iq_report.samples.len;
      iq_report.samples = (int8_t *)evt->data.evt_cte_receiver_connection_iq_report.samples.data;

      app_on_iq_report(conn, &iq_report);
    }
    break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
