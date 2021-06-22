/*
 * log2CSV.c
 *
 *  Created on: 17 èþí. 2021 ã.
 *      Author: logunov
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "aoa_util.h"
#include <log2CSV.h>
#include "conn.h"
#include "app_config.h"
#include "aoa.h"


extern FILE *fCsv;
extern float OneSwitchRotate;

u32 DeadCnt = DEAD_START_CNT;
uint32_t cnt_to_csv = LOG_TO_CSV;
bool onLog = 0;

void I_Q_to_CSV(aoa_iq_report_t *iq_report, int len, conn_properties_t *tag)
{
	if (DeadCnt) DeadCnt--;

	if ((cnt_to_csv == 0) || (DeadCnt!= 0)) return;


	cnt_to_csv--;
	if (cnt_to_csv == 0) {
		if (fCsv != NULL)
			fclose(fCsv);
		onLog = 0;
	}
	else onLog = 1;

	int8_t* iq_data = iq_report->samples;

	if (fCsv == NULL) {
		fCsv = fopen("test.csv", "wb");
		fprintf(fCsv, "N;");		for (int r = 0; r < AOA_NUM_ARRAY_ELEMENTS; ++r) {
			fprintf(fCsv, "i%i;q%i;Degree%i;Own Shft%i;Power%i;;"
					,r,r,r,r,r);
		}
		for (int r = 0; r < AOA_NUM_ARRAY_ELEMENTS-1; ++r) {
			fprintf(fCsv, "Degr%i-Degr%i;"
					,r+1,r);
		}

		fprintf(fCsv, "\r\n\r\n" );
	}

//  sl_rtl_clib_iq_sample_qa_dataset_t *qa_s =&qa_dataset;
//  sl_rtl_clib_iq_sample_qa_antenna_data_t *qa_a = &qa_antenna;
//  if(sl_rtl_aox_iq_sample_qa_get_details(&tag->aoa_states.libitem,qa_s,qa_a)==SL_RTL_ERROR_SUCCESS){
//	fprintf(fCsv,"dataset:;  %s  curr_chan %i,  ref_freq  %0.1f,   ref_sndr  %0.1f,  switching_jitter  %0.1f  \r\n",
//				(qa_s->data_available ? "TRUE" : "FALSE"), qa_s->curr_channel, qa_s->ref_freq,
//				qa_s->ref_sndr, qa_s->switching_jitter);
//	fprintf(fCsv,"ant_data: sig_levl  %0.1fdB, level sig_to_noise ratio  %0.1fdB, avrg unrotated phase  %0.1f°,  Phase variation  %0.1f° \r\n \r\n",
//				qa_a->level, qa_a->snr, qa_a->phase_value * rad2Dg,
//				qa_a->phase_jitter * rad2Dg);
//  }


	fprintf(fCsv, ";;;;Channel %i, Rssi %i\r\n ", iq_report->channel, iq_report->rssi );

	int8_t *end =   iq_data+len;
		float prevDeg[16] = {0,0,0,0};
		float OwnDeg[16] = {0,0,0,0};
		int  N =1;
	while (iq_data < end)
	{
			fprintf(fCsv, "%i;", N);
			N++;
		for (int b = 0; b < AOA_NUM_ARRAY_ELEMENTS; b++)
		{

			s8 _i = *iq_data;iq_data++;
			fprintf(fCsv, "%i;", _i);
			s8 _q = *iq_data;iq_data++;
			fprintf(fCsv, "%i;", _q);

			float rad = atan2(_q,_i);
			OwnDeg[b] = rad;
			float deg = (rad+ t_pi)* rad2Dg;
			fprintf(fCsv, "%.1f;", deg );

			float OwnDiff = restrictRad(prevDeg[b] - rad);
//			float diff = (prevDeg[b] - rad)* rad2Dg;

			fprintf(fCsv, "%.1f;", OwnDiff* rad2Dg );
			prevDeg[b] =   rad ;

			float Power = sqrt(_i*_i + _q*_q);
			fprintf(fCsv, "%.1f;;", Power);

			//**************************
//			char s[24];
//			sprintf(s, "%f;", deg );
//			replace_dot( s);
//			fprintf(fCsv, s);
//
//			float diff = prevDeg - deg;
//			sprintf(s, "%f;", diff);
//			replace_dot( s);
//			fprintf(fCsv, s);
//			prevDeg = deg;
//
//			float Power = sqrt(_i*_i + _q*_q);
//			sprintf(s, "%f;;;", Power);
//			replace_dot( s);
//			fprintf(fCsv, s);
			//*******************************
			if (iq_data > end)
				break;
		}
//		extern float tShftsample;
		for (int b = 0; b < AOA_NUM_ARRAY_ELEMENTS-1; b++)
		{
//			float diff = convto360(OwnDeg[b+1]- OwnDeg[b]- 360*tShftsample);
			float diff = restrictRad(OwnDeg[b+1]- OwnDeg[b]-OneSwitchRotate)*rad2Dg;
			fprintf(fCsv, "%.1f;", diff );
		}
		fprintf(fCsv, "\r\n");
	}



	fprintf(fCsv, "\r\n\r\n");

}



