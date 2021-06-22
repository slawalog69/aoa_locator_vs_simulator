/*
 * Simulator_I_Q.c
 *
 *  Created on: 17 θών. 2021 γ.
 *      Author: logunov
 */

#include "Simulator_I_Q.h"
#include "aoa_util.h"
#include "aoa.h"
#include "app_log.h"
#include "app_config.h"
#include "time.h"
#include <stdlib.h>     /* srand, rand */




#define DUMP 512

float SAMPLING_RATE = 2.0;  //us
float CTE_FREQ = 250.0;		//kHz
extern float REFERENCE_SAMPL_RATE;


float StartAngle = 90;	//degree
//float shift;
float tShftsample; // koeff freq vs 2us
float OneSwitchRotate;
s8 Simul_IQ_DATA[DUMP];
#define toRad(x) x/rad2Dg

/*
 * get angle next sample in reference period
 * [in] angle of current reference sample
 * as p 3.1 in  AN1297
 * in radians only
 */

float Reference_sampling(float angl_from){

	float tShftRef = (REFERENCE_SAMPL_RATE*CTE_FREQ/1000); // reference is 1us
	float OneRefShift = fullRad * tShftRef;
	float DestAngle = restrictRad(angl_from + OneRefShift);
	return DestAngle;
}
/*
 * Calculate angle per one switch antenna in snapshot
 *
 */
void calcOneSwitchRotate(){

	tShftsample = (SAMPLING_RATE * CTE_FREQ / 1000);
	OneSwitchRotate = fullRad * tShftsample;// Rotate per one snapshot
}

static float findAnglShiftperSample(float angl_from, float AOA_shift) {

	float DestAngle = restrictRad(angl_from + OneSwitchRotate + AOA_shift);

	return DestAngle;
}

static inline float GetNoise(float min, float max)
{
	int f1 = (max-min) *1000.0;
	int f2 = min*1000.0;

	/*
	 * uncomment next if need
	 */

//	return ((rand() % f1 + f2)/1000.0);
	return 1.0f;
}


s8* make_I_Q(u8 len, float AOA_shift) {

//	app_log("make_I_Q.. len: %i \n",len );
	 calcOneSwitchRotate();
	srand (time(NULL));
	float aoa_shft_rad = toRad(AOA_shift);
float rndNoise;
	s8 *psimData = Simul_IQ_DATA;
	s8 * end = psimData+len;


	StartAngle = 30;//(rand() % 360);
	float currAnglRad = toRad(StartAngle);

	float firstAnglRad =currAnglRad;
//=========Ref period ===================
		for (int t = 0; t < AOA_REF_PERIOD_SAMPLES; t++) {


			rndNoise = GetNoise(0.99,1.0);
			*psimData = cos(currAnglRad) * 127*rndNoise; // i
			psimData++;

			rndNoise =  GetNoise(0.99,1.0);
			*psimData = sin(currAnglRad) * 127*rndNoise; // q
			psimData++;
			currAnglRad = Reference_sampling(currAnglRad);
		}

			firstAnglRad =currAnglRad;
	// ============= Snapshots ==========================
	while (psimData<end) {
		for (int d = 0; d < AOA_NUM_ARRAY_ELEMENTS; d++) {
			rndNoise =  GetNoise(0.6,1.0);
			*psimData = cos(currAnglRad) * 127*rndNoise;
			psimData++;
			rndNoise =  GetNoise(0.6,1.0);
			*psimData = sin(currAnglRad) * 127*rndNoise;
			psimData++;
			currAnglRad = findAnglShiftperSample(currAnglRad, aoa_shft_rad);
		}
	// ================ Set angle on first path of antenna ========
		//
		float fSwAnglePerOneSnapshot = AOA_NUM_ARRAY_ELEMENTS * OneSwitchRotate;
			firstAnglRad = restrictRad(firstAnglRad + fSwAnglePerOneSnapshot );
			currAnglRad = firstAnglRad;
	}

	return Simul_IQ_DATA;

}

