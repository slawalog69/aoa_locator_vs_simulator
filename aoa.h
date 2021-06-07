/***************************************************************************//**
 * @file
 * @brief AoA header file
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
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

#ifndef AOA_H
#define AOA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "aoa_types.h"
#include "sl_bt_api.h"
#include "sl_rtl_clib_api.h"
#include "sl_ncp_evt_filter_common.h"

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

typedef struct aoa_libitems {
  sl_rtl_aox_libitem libitem;
  sl_rtl_util_libitem util_libitem;
} aoa_libitems_t;

/***************************************************************************************************
 * Public variables
 **************************************************************************************************/
extern float aoa_azimuth_min;
extern float aoa_azimuth_max;

/***************************************************************************************************
 * Function Declarations
 **************************************************************************************************/

void aoa_init_buffers(void);
void aoa_init(aoa_libitems_t *aoa_state);
sl_status_t aoa_calculate(aoa_libitems_t *aoa_state, aoa_iq_report_t *iq_report, aoa_angle_t *angle);
sl_status_t aoa_deinit(aoa_libitems_t *aoa_state);

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */

#ifdef __cplusplus
};
#endif

#endif /* AOA_H */
