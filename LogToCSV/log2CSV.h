/*
 * log2CSV.h
 *
 *  Created on: 17 θών. 2021 γ.
 *      Author: logunov
 */

#ifndef LOG2CSV_H_
#define LOG2CSV_H_


#define LOG_TO_CSV                   15
#define DEAD_START_CNT               30

#include "conn.h"

extern void I_Q_to_CSV(aoa_iq_report_t *iq_report, int len, conn_properties_t *tag);

extern FILE *fCsv;
extern u32 cnt_to_csv;
#endif /* LOG2CSV_H_ */
