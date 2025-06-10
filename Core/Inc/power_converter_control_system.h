/*
 * power_converter_control_system.h
 *
 *  Created on: Feb 3, 2023
 *      Author: Sebastien
 */

#ifndef INC_POWER_CONVERTER_CONTROL_SYSTEM_H_
#define INC_POWER_CONVERTER_CONTROL_SYSTEM_H_

#include "main.h"
#include "stdint.h"
#include "stdbool.h"

//#define USE_DAC 0

void InitConverterControlSystemPeripherals();

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

#ifdef STM32H743xx
#else
#define hdac1 hdac
#endif

extern DAC_HandleTypeDef hdac1;

extern DMA_HandleTypeDef hdma_adc1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim2;


// define an enumeration for the state machine states

typedef enum{
	SMInit,
	SMWaitForRun, // wait for the PC message to start
	SMStartPWM,
	SMRun,
	SMSoftStop,
	SMFault,
	SMFaultSignal,
	SMStop,
	SMStopSignal
} TStateMachine;


// put all converter control variables in a structure

typedef struct{
	float imeas[3];	// measured currents in A
	float umeas[3]; // measured voltages in V
	float ilem[3];	// measured currents in A
	float ileg[3];	// measured currents in A
	float ilemf[3];	// measured currents in A
	float ilegf[3];	// measured currents in A
	float da[4];	// duty cycles conv A
	float db[4];	// duty cycles conv B
	float d[4];
	float ref;
	float ref0;
	float ref1;
	uint32_t k;	// time index
	TStateMachine sm;	// state
	bool signal_run;	// signal from uart handler to start
	bool signal_stop;	// signal from uart handler to stop
	uint16_t mode;
} TmyconvVSI;


#define NB_AD_CONV 3
#define AD_RANK 1
#define AD_RANK_3 1
#define NB_AD_VALUES (NB_AD_CONV*AD_RANK+AD_RANK_3-AD_RANK)


void LogMeasurementsExample();



void ADCHalfCompleteCallback(ADC_HandleTypeDef* hadc);
void ADCCompleteCallback(ADC_HandleTypeDef* hadc);
void ADCErrorCallback(ADC_HandleTypeDef* hadc);

extern TmyconvVSI myconvvsi;
extern float ad_volt_float[NB_AD_CONV*AD_RANK+AD_RANK_3-AD_RANK+1];


#endif /* INC_POWER_CONVERTER_CONTROL_SYSTEM_H_ */
