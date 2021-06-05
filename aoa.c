/***************************************************************************//**
 * @file
 * @brief Process IQ samples and calculate angle estimation.
 *
 * Module responsible for processing IQ samples and calculate angle estimation
 * from them using the AoX library
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

// standard library headers
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include "aoa.h"
#include "app_log.h"
#include "app_config.h"

/***************************************************************************************************
 * Public Variables
 **************************************************************************************************/
float aoa_azimuth_min = AOA_AZIMUTH_MASK_MIN_DEFAULT;
float aoa_azimuth_max = AOA_AZIMUTH_MASK_MAX_DEFAULT;

/***************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static float **ref_i_samples;
static float **ref_q_samples;
static float **i_samples;
static float **q_samples;

/***************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/

static enum sl_rtl_error_code aox_process_samples(aoa_libitems_t *aoa_state, aoa_iq_report_t *iq_report, float *azimuth, float *elevation, uint32_t *qa_result);
static float calc_frequency_from_channel(uint8_t channel);
static uint32_t allocate_2D_float_buffer(float*** buf, uint32_t rows, uint32_t cols);
static void get_samples(aoa_iq_report_t *iq_report);

/***************************************************************************************************
 * Public Function Definitions
 **************************************************************************************************/
void aoa_init_buffers(void)
{
  allocate_2D_float_buffer(&ref_i_samples, AOA_NUM_SNAPSHOTS, AOA_NUM_ARRAY_ELEMENTS);
  allocate_2D_float_buffer(&ref_q_samples, AOA_NUM_SNAPSHOTS, AOA_NUM_ARRAY_ELEMENTS);

  allocate_2D_float_buffer(&i_samples, AOA_NUM_SNAPSHOTS, AOA_NUM_ARRAY_ELEMENTS);
  allocate_2D_float_buffer(&q_samples, AOA_NUM_SNAPSHOTS, AOA_NUM_ARRAY_ELEMENTS);
}

void aoa_init(aoa_libitems_t *aoa_state)
{
  app_log("AoA library init...\n");
  // Initialize AoX library
  sl_rtl_aox_init(&aoa_state->libitem);
  // Set the number of snapshots - how many times the antennas are scanned during one measurement
  sl_rtl_aox_set_num_snapshots(&aoa_state->libitem, AOA_NUM_SNAPSHOTS);
  // Set the antenna array type
  sl_rtl_aox_set_array_type(&aoa_state->libitem, AOX_ARRAY_TYPE);
  // Select mode (high speed/high accuracy/etc.)
  sl_rtl_aox_set_mode(&aoa_state->libitem, AOX_MODE);
  // Enable IQ sample quality analysis processing
  sl_rtl_aox_iq_sample_qa_configure(&aoa_state->libitem);
  // Add azimuth constraint if min and max values are valid
  if (!isnan(aoa_azimuth_min) && !isnan(aoa_azimuth_max)) {
    app_log("Disable azimuth values between %f and %f\n", aoa_azimuth_min, aoa_azimuth_max);
    sl_rtl_aox_add_constraint(&aoa_state->libitem, SL_RTL_AOX_CONSTRAINT_TYPE_AZIMUTH, aoa_azimuth_min, aoa_azimuth_max);
  }
  // Create AoX estimator
  sl_rtl_aox_create_estimator(&aoa_state->libitem);
  // Initialize an util item
  sl_rtl_util_init(&aoa_state->util_libitem);
  sl_rtl_util_set_parameter(&aoa_state->util_libitem, SL_RTL_UTIL_PARAMETER_AMOUNT_OF_FILTERING, FILTERING_AMOUNT);
}

sl_status_t aoa_calculate(aoa_libitems_t *aoa_state, aoa_iq_report_t *iq_report, aoa_angle_t *angle)
{
  uint32_t quality_result;
  char *iq_sample_qa_string;
  sl_status_t ret_val = SL_STATUS_OK;

  // Process new IQ samples and calculate Angle of Arrival (azimuth, elevation)
  enum sl_rtl_error_code ret = aox_process_samples(aoa_state, iq_report, &angle->azimuth, &angle->elevation, &quality_result);
  // sl_rtl_aox_process will return SL_RTL_ERROR_ESTIMATION_IN_PROGRESS until it has received enough packets for angle estimation
  if (ret == SL_RTL_ERROR_SUCCESS) {
    // Check the IQ sample quality result and present a short string according to it
    if (quality_result == 0) {
      iq_sample_qa_string = "Good                                   ";
    } else if (SL_RTL_AOX_IQ_SAMPLE_QA_IS_SET(quality_result, SL_RTL_AOX_IQ_SAMPLE_QA_REF_ANT_PHASE_JITTER)
               || SL_RTL_AOX_IQ_SAMPLE_QA_IS_SET(quality_result, SL_RTL_AOX_IQ_SAMPLE_QA_ANT_X_PHASE_JITTER)) {
      iq_sample_qa_string = "Caution - phase jitter too large       ";
    } else if (SL_RTL_AOX_IQ_SAMPLE_QA_IS_SET(quality_result, SL_RTL_AOX_IQ_SAMPLE_QA_SNDR)) {
      iq_sample_qa_string = "Caution - reference period SNDR too low";
    } else {
      iq_sample_qa_string = "Caution (other)                        ";
    }
    // Calculate distance from RSSI, and calculate a rough position estimation
    sl_rtl_util_rssi2distance(TAG_TX_POWER, iq_report->rssi / 1.0, &angle->distance);
    sl_rtl_util_filter(&aoa_state->util_libitem, angle->distance, &angle->distance);
    app_log("azimuth: %6.1f  \televation: %6.1f  \trssi: %6.0f  \tch: %2d  \tSequence: %5d  \tDistance: %6.3f  \tIQ sample Quality: %s\n",
            angle->azimuth, angle->elevation, iq_report->rssi / 1.0, iq_report->channel, iq_report->event_counter, angle->distance, iq_sample_qa_string);
    angle->rssi = iq_report->rssi;
    angle->channel = iq_report->channel;
    angle->sequence = iq_report->event_counter;
  } else {
    app_log("Failed to calculate angle. (%d) \n", ret);
    ret_val = SL_STATUS_FAIL;
  }

  return ret_val;
}

static enum sl_rtl_error_code aox_process_samples(aoa_libitems_t *aoa_state, aoa_iq_report_t *iq_report, float *azimuth, float *elevation, uint32_t *qa_result)
{
  float phase_rotation;

  get_samples(iq_report);

  // Calculate phase rotation from reference IQ samples
  sl_rtl_aox_calculate_iq_sample_phase_rotation(&aoa_state->libitem, 2.0f, ref_i_samples[0], ref_q_samples[0], AOA_REF_PERIOD_SAMPLES, &phase_rotation);

  // Provide calculated phase rotation to the estimator
  sl_rtl_aox_set_iq_sample_phase_rotation(&aoa_state->libitem, phase_rotation);

  // Estimate Angle of Arrival / Angle of Departure from IQ samples
  enum sl_rtl_error_code ret = sl_rtl_aox_process(&aoa_state->libitem, i_samples, q_samples, calc_frequency_from_channel(iq_report->channel), azimuth, elevation);

  // fetch the quality results
  *qa_result = sl_rtl_aox_iq_sample_qa_get_results(&aoa_state->libitem);

  return ret;
}

static float calc_frequency_from_channel(uint8_t channel)
{
  static const uint8_t logical_to_physical_channel[40] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
                                                           13, 14, 15, 16, 17, 18, 19, 20, 21,
                                                           22, 23, 24, 25, 26, 27, 28, 29, 30,
                                                           31, 32, 33, 34, 35, 36, 37, 38,
                                                           0, 12, 39 };

  // return the center frequency of the given channel
  return 2402000000 + 2000000 * logical_to_physical_channel[channel];
}

sl_status_t aoa_deinit(aoa_libitems_t *aoa_state)
{
  enum sl_rtl_error_code ret;
  sl_status_t retval = SL_STATUS_OK;

  ret = sl_rtl_aox_deinit(&aoa_state->libitem);

  if (ret != SL_RTL_ERROR_SUCCESS) {
    retval = SL_STATUS_FAIL;
  }

  ret = sl_rtl_util_deinit(&aoa_state->util_libitem);

  if (ret != SL_RTL_ERROR_SUCCESS) {
    retval = SL_STATUS_FAIL;
  }

  return retval;
}

static uint32_t allocate_2D_float_buffer(float*** buf, uint32_t rows, uint32_t cols)
{
  *buf = malloc(sizeof(float*) * rows);
  if (*buf == NULL) {
    return 0;
  }

  for (uint32_t i = 0; i < rows; i++) {
    (*buf)[i] = malloc(sizeof(float) * cols);
    if ((*buf)[i] == NULL) {
      return 0;
    }
  }

  return 1;
}

static void get_samples(aoa_iq_report_t *iq_report)
{
  uint32_t index = 0;
  // Write reference IQ samples into the IQ sample buffer (sampled on one antenna)
  for (uint32_t sample = 0; sample < AOA_REF_PERIOD_SAMPLES; ++sample) {
    ref_i_samples[0][sample] = iq_report->samples[index++] / 127.0;
    if (index == iq_report->length) {
      break;
    }
    ref_q_samples[0][sample] = iq_report->samples[index++] / 127.0;
    if (index == iq_report->length) {
      break;
    }
  }
  index = AOA_REF_PERIOD_SAMPLES * 2;
  // Write antenna IQ samples into the IQ sample buffer (sampled on all antennas)
  for (uint32_t snapshot = 0; snapshot < AOA_NUM_SNAPSHOTS; ++snapshot) {
    for (uint32_t antenna = 0; antenna < AOA_NUM_ARRAY_ELEMENTS; ++antenna) {
      i_samples[snapshot][antenna] = iq_report->samples[index++] / 127.0;
      if (index == iq_report->length) {
        break;
      }
      q_samples[snapshot][antenna] = iq_report->samples[index++] / 127.0;
      if (index == iq_report->length) {
        break;
      }
    }
    if (index == iq_report->length) {
      break;
    }
  }
}
