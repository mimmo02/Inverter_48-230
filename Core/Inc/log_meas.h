/*
 * log_meas.h
 *
 *  Created on: Feb 11, 2023
 *      Author: Sebastien
 */

#ifndef INC_LOG_MEAS_H_
#define INC_LOG_MEAS_H_


#include <stdint.h>


#define LOG_MEAS_NB 20000
#define LOG_MEAS_MARGIN 50

extern float db_meas[LOG_MEAS_NB+LOG_MEAS_MARGIN];
extern uint16_t db_cnt_meas;
extern uint16_t db_cnt_meas_tx;

#endif /* INC_LOG_MEAS_H_ */
