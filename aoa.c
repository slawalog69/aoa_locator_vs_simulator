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

sl_rtl_clib_iq_sample_qa_dataset_t qa_dataset;
  sl_rtl_clib_iq_sample_qa_antenna_data_t qa_antenna;
/***************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/

static enum sl_rtl_error_code aox_process_samples(aoa_libitems_t *aoa_state, aoa_iq_report_t *iq_report, float *azimuth, float *elevation, uint32_t *qa_result);
static float calc_frequency_from_channel(uint8_t channel);
static uint32_t allocate_2D_float_buffer(float*** buf, uint32_t rows, uint32_t cols);
static void get_samples(aoa_iq_report_t *iq_report,float fr);


const char ARR_TYP_STRNG[3][19]={"ARRAY_TYPE_4x4_URA",
		"ARRAY_TYPE_3x3_URA",
		"ARRAY_TYPE_1x4_ULA"};
const char Strng_Mode[12][64] = {
		 "SL_RTL_AOX_MODE_ONE_SHOT_BASIC", ///< Medium filtering, medium response. Returns 2D angle, requires 10 rounds. Most suitable for single shot measurement.
		  "SL_RTL_AOX_MODE_ONE_SHOT_BASIC_LIGHTWEIGHT", ///< Medium filtering, medium response, low CPU cost & low elevation resolution. 2D angle, req. 10 rounds. Most suitable for single shot measurement.
		  "SL_RTL_AOX_MODE_ONE_SHOT_FAST_RESPONSE", ///< Low filtering, fast response, low CPU cost & low elevation resolution. 2D angle, requires 2 rounds. Most suitable for single shot measurement.
		  "SL_RTL_AOX_MODE_ONE_SHOT_HIGH_ACCURACY", ///< High filtering, slow response. 2D angle, requires 20 rounds. Most suitable for single shot measurement.

		  "SL_RTL_AOX_MODE_ONE_SHOT_BASIC_AZIMUTH_ONLY", ///< Equivalent to ONE_SHOT_BASIC with low CPU cost and returns 1D angle. Most suitable for single shot measurement.
		  "SL_RTL_AOX_MODE_ONE_SHOT_FAST_RESPONSE_AZIMUTH_ONLY", ///< Equivalent to ONE_SHOT_FAST_RESPONSE with low CPU cost, 1D angle. Most suitable for single shot measurement.
		  "SL_RTL_AOX_MODE_ONE_SHOT_HIGH_ACCURACY_AZIMUTH_ONLY", ///< Equivalent to ONE_SHOT_HIGH_ACCURACY with low CPU cost, 1D angle. Most suitable for single shot measurement.

		"SL_RTL_AOX_MODE_REAL_TIME_FAST_RESPONSE",
		"SL_RTL_AOX_MODE_REAL_TIME_BASIC",
		"SL_RTL_AOX_MODE_REAL_TIME_HIGH_ACCURACY" };
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

  //****************************


  app_log("AOA_NUM_SNAPSHOTS  %i\n\
AOX_ARRAY_TYPE  %s\n\
AOX_MODE %s\n",
		  AOA_NUM_SNAPSHOTS,
		  ARR_TYP_STRNG[AOX_ARRAY_TYPE],
		  Strng_Mode[AOX_MODE-3]);


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
               || SL_RTL_AOX_IQ_SAMPLE_QA_IS_SET(quality_result, SL_RTL_AOX_IQ_SAMPLE_QA_ANT_X_PHASE_JITTER)
			   ) {
      iq_sample_qa_string = "Caution - phase jitter too large       ";
    } else if (SL_RTL_AOX_IQ_SAMPLE_QA_IS_SET(quality_result, SL_RTL_AOX_IQ_SAMPLE_QA_SNDR)) {
      iq_sample_qa_string = "Caution - reference period SNDR too low";
    } else {
      iq_sample_qa_string = "Caution (other)                        ";
    }
    // Calculate distance from RSSI, and calculate a rough position estimation
    sl_rtl_util_rssi2distance(TAG_TX_POWER, iq_report->rssi / 1.0, &angle->distance);
    sl_rtl_util_filter(&aoa_state->util_libitem, angle->distance, &angle->distance);

//    app_log("azimuth: %6.1f  elevation: %6.1f  rssi: %6.0f  ch: %2d  Sequence: %5d    Distance: %6.3f  IQ sample Quality: %s quality_result %i\n",
//            angle->azimuth, angle->elevation, iq_report->rssi / 1.0, iq_report->channel, iq_report->event_counter, angle->distance, iq_sample_qa_string, quality_result);
    app_log("azimuth: %6.1f ° rssi: %6.0f  ch: %2d   IQ sample Quality: %s -(%i)\n",
            angle->azimuth,iq_report->rssi / 1.0, iq_report->channel, iq_sample_qa_string,quality_result);
    angle->rssi = iq_report->rssi;
    angle->channel = iq_report->channel;
    angle->sequence = iq_report->event_counter;
  } else {

    app_log("Failed to calculate angle. (%d) \n", ret);
    ret_val = SL_STATUS_FAIL;
  }

  return ret_val;
}


//void get_qa_detal(sl_rtl_clib_iq_sample_qa_dataset_t** s,
//		sl_rtl_clib_iq_sample_qa_antenna_data_t **a){
//	*s = &qa_dataset;
//	*a = &qa_antenna;
//}


float REFERENCE_SAMPL_RATE = 1.0;  //us

static enum sl_rtl_error_code aox_process_samples(aoa_libitems_t *aoa_state,
		aoa_iq_report_t *iq_report,
		float *azimuth,
		float *elevation,
		uint32_t *qa_result)
{
  float phase_rotation;
float fr = calc_frequency_from_channel(iq_report->channel);

  get_samples(iq_report,fr);

  // Calculate phase rotation from reference IQ samples
 enum sl_rtl_error_code e = sl_rtl_aox_calculate_iq_sample_phase_rotation(&aoa_state->libitem,
		 REFERENCE_SAMPL_RATE,
		  ref_i_samples[0],
		  ref_q_samples[0],
		  AOA_REF_PERIOD_SAMPLES,
		  &phase_rotation);

	app_log("Phase rotation on ref period:  %.1f - err: (%i) \n", phase_rotation,e);


  // Provide calculated phase rotation to the estimator
  e =sl_rtl_aox_set_iq_sample_phase_rotation(&aoa_state->libitem, phase_rotation);
	app_log("Set Phase rotation.. - err: (%i) \n", e);
	app_log("Channel freq : %0.1f MHz\n Estimate AOA..\n", fr/1000000.0);

  // Estimate Angle of Arrival / Angle of Departure from IQ samples
  enum sl_rtl_error_code ret = sl_rtl_aox_process(&aoa_state->libitem,
		  i_samples,
		  q_samples,
		  fr,
		  azimuth,
		  elevation);

  // fetch the quality results
  *qa_result = sl_rtl_aox_iq_sample_qa_get_results(&aoa_state->libitem);

//  sl_rtl_aox_iq_sample_qa_get_details(&aoa_state->libitem,&qa_dataset,&qa_antenna);
//	if (s.data_available) {
//		app_log(
//				"dataset:  %s  curr_chan %i  ref_freq %0.1f  ref_sndr %0.1f switching_jitter %0.1f \n",
//				s.data_available ? "TRUE" : "FALSE", s.curr_channel, s.ref_freq,
//				s.ref_sndr, s.switching_jitter);
//
//		app_log(
//				"ant_data:  sig levl %0.1fdB,  levl sig_to_noise ratio %0.1fdB,  avrg unrotated phase %0.1f°,  Phase variation %0.1f°",
//				a.level, a.snr, a.phase_value * rad2Dg,
//				a.phase_jitter * rad2Dg);
//	}

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
extern FILE *fSampl;
extern bool onLog;
extern float SAMPLING_RATE;
extern float CTE_FREQ;
static void get_samples(aoa_iq_report_t *iq_report,float fr)
{

	if (fSampl  == NULL) {
		if (onLog) {
			fSampl = fopen("Sample.csv", "wb");
			fprintf(fSampl, ";;;****** CREATE SAMPLES IN  get_samples()(aoa.c file)*****\r\n");
			fprintf(fSampl,
					"\r\n===CURRENT SETTINGS=======\r\n \
				AOX_ARRAY_TYPE;;;%s\r\n \
				NUM_ARRAY_ELEMENTS;;;%i\r\n \
				rtl_aox_mode;;;%s\r\n \
				AOA_NUM_SNAPSHOTS;;;%i\n\
				SAMPLING_RATE REF PERIOD;;;%0.1f;us\r\n \
				SAMPLING_RATE SNAPSHOTS;;;%0.1f;us\r\n \
				CTE_FREQ;;;%0.1f;kHz\r\n",

					ARR_TYP_STRNG[AOX_ARRAY_TYPE],
					AOA_NUM_ARRAY_ELEMENTS, Strng_Mode[AOX_MODE - 3],
					AOA_NUM_SNAPSHOTS, REFERENCE_SAMPL_RATE, SAMPLING_RATE,
					CTE_FREQ);
		}
	}


	if(onLog){
	fprintf(fSampl , "=================================================\r\n");
	fprintf(fSampl , "\r\nChannel frq;;;%0.1f;MHz\r\n;;;reference samples;\r\nI;Q\r\n",fr/1000000.0f);
	}
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

	if(onLog)
		fprintf(fSampl ,"%i;%i\r\n",(s8)(ref_i_samples[0][sample]*127),
				(s8)(ref_q_samples[0][sample]*127));
  }


	if(onLog)
		fprintf(fSampl , ";;;snapshots\r\n");

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
		if(onLog)
			fprintf(fSampl ,"%i;%i;;;",(s8)(i_samples[snapshot][antenna]*127),
					(s8)(q_samples[snapshot][antenna]*127));
    }
    if(onLog)
			fprintf(fSampl ,"\r\n");

    if (index == iq_report->length) {
      break;
    }
  }
	if(onLog)
		  fprintf(fSampl, "\r\n============================================\r\n\r\n");

  	if((!onLog)&&(fSampl  != NULL))
			fclose(fSampl );

}

//*****************************
/*
 * modulation float value on 2xPi (360°) base
 *
 */

float restrictRad(float in){

	return fmod (in, fullRad);
}
