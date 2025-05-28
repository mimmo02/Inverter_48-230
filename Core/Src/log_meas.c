/*
 * log_meas.c
 *
 *  Created on: Feb 11, 2023
 *      Author: Sebastien
 */

#include "log_meas.h"


float db_meas[LOG_MEAS_NB+LOG_MEAS_MARGIN];
uint16_t db_cnt_meas = 0, db_cnt_div = 0 , db_cnt_meas_tx = 0;


