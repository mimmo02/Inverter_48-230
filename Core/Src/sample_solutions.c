/*
 * sample_solutions.c
 *
 *  Created on: Nov 27, 2023
 *      Author: Sebastien
 */

#include "power_converter_control_system.h"
#include "main.h"
#include "stdbool.h"
#include "stdint.h"
#include "log_meas.h"
#include "math.h"


int my_cnt = 0;

void RunControlAtEachSamplingPeriod1()
{
	my_cnt++;
	// count from 1 to 10000, at 10000, get back to 0, at 5000 switch off the led signals
	if( my_cnt>=10000) {
		my_cnt = 0;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
		myconvvsi.d[1] = 0.01;
	} else if( my_cnt>=5000) {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
		myconvvsi.d[1] = 0.99;
	}
}


void LogMeasurementsAtEachSamplingPeriod1()
{
	if( db_cnt_meas<LOG_MEAS_NB) {
		db_meas[db_cnt_meas++] = my_cnt;
	}
}


typedef enum{ sm_triangle_rise, sm_triangle_fall} TSMTriangle;

TSMTriangle my_sm_triangle;
float my_triangle = 0;
float dac[2] = {0,0};
float x, y = 0;

void RunControlAtEachSamplingPeriod2()
{
	switch( my_sm_triangle) {
	case sm_triangle_rise:
		my_triangle+=0.1;
		if( my_triangle >= 1-1e-4)
			my_sm_triangle = sm_triangle_fall;
		break;
	case sm_triangle_fall:
		my_triangle-=0.1;
		if( my_triangle <= -1+1e-4)
			my_sm_triangle = sm_triangle_rise;
		break;
	default:
		my_sm_triangle = sm_triangle_rise;
	}

	dac[0] = (0.5+0.5*0.98*my_triangle)*4095.0;

	x = ad_volt_float[4];
	y += 0.1*(x-y);

	dac[1] = y/3.3*4095.0;

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)dac[0]);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)dac[1]);
}

void LogMeasurementsAtEachSamplingPeriod2()
{
	if( db_cnt_meas<LOG_MEAS_NB) {
		db_meas[db_cnt_meas++] = my_triangle;
		db_meas[db_cnt_meas++] = dac[0];
		db_meas[db_cnt_meas++] = dac[1];
		db_meas[db_cnt_meas++] = x;
		db_meas[db_cnt_meas++] = y;
		db_meas[db_cnt_meas++] = (float)my_sm_triangle;
	}
}




#if 0==1

// code declaration

// constant parameters

float K=1; // change with right value
float zi=0.99; // change with the right value
float Kzi=0.99; // change with right value

// state variables

float uPIkm1=0; // past controller output
float uPkm1=0; // past controller input
// controller output

float uPIk;

// controller input (control error)

float uPk;

// end of code declaration

// controller periodic function (at each sampling instant) in output function in PLECS

void PIController()
{
	uPIk = K * uPIkm1 + K * uPk - Kzi*uPkm1;
	// update the state variable for next run
	uPIkm1 = uPIk;
	uPkm1 = uPk;
}


// In PLECS; does not work on microcontroller
// will be replaced with ADC signal

uPk = InputSignal(0,0);
PIController();
// will be replaced with DAC or PWM signal
OutputSignal(0,0) = uPIk;

// declarations

// state variables input and output

float uPI[2] = {0,0};
float u[2] = {0,0};

// controller parameters

float K=1; // change with right value
float zi=0.99; // change with the right value
float Kzi=0.99; // change with right value

void PIController()
{
	uPI[0] = K*uPI[1]+K*u[0]-Kzi*u[1];
	uPI[1] = uPI[0];
	u[1] = u[0];
}


// declarations

// structure definition with all PI parameters and variables

typedef struct {
	float uPI[2];
	float u[2];
	float K;
	float mKzi;
} TPIController;

// PI controller declaration

TPIController mypi = {
	.uPI = {0,0},
	.u = {0,0},
	.K = 1,
	.mKzi = 0.99,
};

void PIController()
{
	pi.uPI[0] = pi.K*pi.uPI[1]
				+pi.K*pi.u[0]
				+pi.mKzi*pi.u[1];
	pi.uPI[1] = pi.uPI[0];
	pi.u[1] = pi.u[0];
}


void PIController( TPIController *pi)
{
	pi->uPI[0] = pi->K*pi->uPI[1]
				+pi->K*pi->u[0]
				+pi->mKzi*pi->u[1];
	pi->uPI[1] = pi->uPI[0];
	pi->u[1] = pi->u[0];
}


// structure definition with all lead parameters and variables

typedef struct {
	float x[2];
	float y[2];
	float z[3];
} TPIController;

void LeagLagPICompensator()
{
	y[0] = z[0]*y[1]
		  +z[1]*x[0]
		  +z[2]*x[1];
	x[1] = x[0];
	y[1] = y[0];
}




#endif



