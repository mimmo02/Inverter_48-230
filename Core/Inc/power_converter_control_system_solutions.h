/*
 * power_converter_control_system_solutions.h
 *
 *  Created on: Mar 19, 2023
 *      Author: Sebastien
 */

#ifndef INC_POWER_CONVERTER_CONTROL_SYSTEM_SOLUTIONS_H_
#define INC_POWER_CONVERTER_CONTROL_SYSTEM_SOLUTIONS_H_


#include "stdint.h"
#include "stdbool.h"

typedef struct {
	float a[3];
	float da;
	float up;
	float upmax;
	float udcmeas;
	float udcinv;
	float dup;
	float x[3];
	float iref;
	float i;
	float e;
	float ep;
	float u;
	float d;
} TMycon1;


void InitControlExampleOpenLoopSin();
void RunControlExampleOpenLoopSin();

void InitSolutionControlEx1();
void RunSolutionControlEx1();

void LogMeasurementsExampleCurrentController();


#endif /* INC_POWER_CONVERTER_CONTROL_SYSTEM_SOLUTIONS_H_ */
