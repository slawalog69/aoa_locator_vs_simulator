/***************************************************************************//**
 * @file
 * @brief Connection handler header file
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

#ifndef CONN_H
#define CONN_H

#include <stdint.h>
#include "sl_bt_api.h"
#include "aoa.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************//**
 * \defgroup app Application Code
 * \brief Sample Application Implementation
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/***************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

// Connection state, used only in connection oriented mode
typedef enum {
  DISCOVER_SERVICES,
  DISCOVER_CHARACTERISTICS,
  ENABLE_CTE,
  RUNNING
} connection_state_t;

typedef struct {
  uint16_t connection_handle;   //This is used for connection handle for connection oriented, and for sync handle for connection less mode
  bd_addr address;
  uint8_t address_type;
  uint32_t cte_service_handle;
  uint16_t cte_enable_char_handle;
  connection_state_t connection_state;
  aoa_libitems_t aoa_states;
} conn_properties_t;

/***************************************************************************************************
 * Function Declarations
 **************************************************************************************************/

void init_connection(void);

conn_properties_t* add_connection(uint16_t connection, bd_addr *address, uint8_t address_type);

uint8_t remove_connection(uint16_t connection);

uint8_t is_connection_list_full(void);

conn_properties_t* get_connection_by_handle(uint16_t connection_handle);
conn_properties_t* get_connection_by_address(bd_addr* address);

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */

#ifdef __cplusplus
};
#endif

#endif /* CONN_H */
