/***************************************************************************//**
 * @file
 * @brief Connection handler module, responsible for storing states of open connections
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
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

#include <string.h>
#include "app_config.h"
#include "conn.h"

#define CONNECTION_HANDLE_INVALID     (uint16_t)0xFFFFu
#define SERVICE_HANDLE_INVALID        (uint32_t)0xFFFFFFFFu
#define CHARACTERISTIC_HANDLE_INVALID (uint16_t)0xFFFFu
#define TABLE_INDEX_INVALID           (uint8_t)0xFFu

/***************************************************************************************************
 * Static Variable Declarations
 **************************************************************************************************/

// Array for holding properties of multiple (parallel) connections
static conn_properties_t conn_properties[AOA_MAX_TAGS];

// Counter of active connections
static uint8_t active_connections_num;

/***************************************************************************************************
 * Public Function Definitions
 **************************************************************************************************/
void init_connection(void)
{
  uint8_t i;
  active_connections_num = 0;

  // Initialize connection state variables
  for (i = 0; i < AOA_MAX_TAGS; i++) {
    conn_properties[i].connection_handle = CONNECTION_HANDLE_INVALID;
    conn_properties[i].cte_service_handle = SERVICE_HANDLE_INVALID;
    conn_properties[i].cte_enable_char_handle = CHARACTERISTIC_HANDLE_INVALID;
  }
}

conn_properties_t* add_connection(uint16_t connection, bd_addr *address, uint8_t address_type)
{
  conn_properties_t* ret = NULL;

  // If there is place to store new connection
  if (active_connections_num < AOA_MAX_TAGS) {
    // Store the connection handle, and the server address
    conn_properties[active_connections_num].connection_handle = connection;
    conn_properties[active_connections_num].address = *address;
    conn_properties[active_connections_num].address_type = address_type;
    conn_properties[active_connections_num].connection_state = DISCOVER_SERVICES;
    aoa_init(&conn_properties[active_connections_num].aoa_states);
    // Entry is now valid
    ret = &conn_properties[active_connections_num];
    active_connections_num++;
  }
  return ret;
}

uint8_t remove_connection(uint16_t connection)
{
  uint8_t i;
  uint8_t table_index = TABLE_INDEX_INVALID;

  // If there are no open connections, return error
  if (active_connections_num == 0) {
    return 1;
  }

  // Find the table index of the connection to be removed
  for (uint8_t i = 0; i < active_connections_num; i++) {
    if (conn_properties[i].connection_handle == connection) {
      table_index = i;
      break;
    }
  }

  // If connection not found, return error
  if (table_index == TABLE_INDEX_INVALID) {
    return 1;
  }

  aoa_deinit(&conn_properties[table_index].aoa_states);

  // Decrease number of active connections
  active_connections_num--;

  // Shift entries after the removed connection toward 0 index
  for (i = table_index; i < active_connections_num; i++) {
    conn_properties[i] = conn_properties[i + 1];
  }

  // Clear the slots we've just removed so no junk values appear
  for (i = active_connections_num; i < AOA_MAX_TAGS; i++) {
    conn_properties[i].connection_handle = CONNECTION_HANDLE_INVALID;
    conn_properties[i].cte_service_handle = SERVICE_HANDLE_INVALID;
    conn_properties[i].cte_enable_char_handle = CHARACTERISTIC_HANDLE_INVALID;
  }

  return 0;
}

uint8_t is_connection_list_full(void)
{
  // Return if connection state table is full
  return (active_connections_num >= AOA_MAX_TAGS);
}

conn_properties_t* get_connection_by_handle(uint16_t connection_handle)
{
  conn_properties_t* ret = NULL;
  // Find the connection state entry in the table corresponding to the connection handle
  for (uint8_t i = 0; i < active_connections_num; i++) {
    if (conn_properties[i].connection_handle == connection_handle) {
      // Return a pointer to the connection state entry
      ret = &conn_properties[i];
      break;
    }
  }
  // Return error if connection not found
  return ret;
}

conn_properties_t* get_connection_by_address(bd_addr* address)
{
  conn_properties_t* ret = NULL;
  // Find the connection state entry in the table corresponding to the connection address
  for (uint8_t i = 0; i < active_connections_num; i++) {
    if (0 == memcmp(address, &(conn_properties[i].address), sizeof(bd_addr))) {
      // Return a pointer to the connection state entry
      ret = &conn_properties[i];
      break;
    }
  }
  // Return error if connection not found
  return ret;
}
