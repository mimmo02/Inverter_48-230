/*
 * power_converter_control_system.c
 *
 *  Created on: Feb 3, 2023
 *      Author: Sebastien Mariethoz
 */


//#include "power_converter_control_system_solutions.h"
#include "power_converter_control_system.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "log_meas.h"
#include "math.h"
#include "display_uart_console.h"
#include "dab_pwm.h"
#include "my_solutions.h"


#ifdef STM32H743xx

#include "stm32h7xx_hal_adc.h"
#define memory_place_adc __attribute__ ((section (".sdramD3")))
#define AD_DIV 65536.0
#else

#include "stm32f7xx_hal_adc.h"
#define memory_place_adc
#define AD_DIV 4096.0
#endif

#include "my_solutions.h"
#include "InverterControlFunctions.h"

// define topology configuration
myInverterConfig inverterConfiguration = CONFIG_B;

// DMA copies values from ADC to this buffer

uint16_t ad_dma_buffer[NB_AD_VALUES+1] memory_place_adc;

// AD values converter to volt at input of AD converter

float ad_volt_float[NB_AD_VALUES+1];


TmyconvVSI myconvvsi = {
		.sm = SMInit,
};



void StartPWMFan()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	TIM2->CCR3 = TIM2->ARR*0.01;
	TIM2->CCR3 = TIM2->ARR*0.01;
}


void StartPWM()
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_5);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
}


void StopPWM()
{
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_4);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_4);
}


void InitConverterControlSystemPeripherals()
{
	// Init ADC

	hadc1.ConvCpltCallback = ADCCompleteCallback;
	hadc1.ConvHalfCpltCallback = ADCHalfCompleteCallback;
	hadc1.ErrorCallback = ADCErrorCallback;

	__HAL_ADC_ENABLE(&hadc3);
	__HAL_ADC_ENABLE(&hadc2);
	__HAL_ADC_ENABLE(&hadc1);
	__HAL_ADC_ENABLE_IT(&hadc1, ( ADC_IT_OVR));
	__HAL_ADC_ENABLE_IT(&hadc2, ( ADC_IT_OVR));
	__HAL_ADC_ENABLE_IT(&hadc3, ( ADC_IT_OVR));

	// Launch DMA to be ready to receive next AD values

	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&ad_dma_buffer[0], NB_AD_VALUES);

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	// Init Timer 1 and 8

	HAL_TIM_Base_Start(&htim8);

	HAL_TIM_Base_Start(&htim1);

	// Init UART for communicating with PC

	InitDisplayUartConsole();
}


#define GAIN_I_MEAS (1.0/20.0/6e-3)
#define GAIN_U_MEAS 54.0
#define GAIN_I_MEAS_LEM (2000.0/75.0)


void InitControlProcess()
{
	for( uint16_t k_tmp=0; k_tmp<4; k_tmp++) {
		myconvvsi.da[k_tmp] = 0;
	}
	for( uint16_t k_tmp=0; k_tmp<4; k_tmp++) {
		myconvvsi.db[k_tmp] = 0;
	}
}


uint32_t db_ADCCompleteCallback = 0;

#ifdef CONVERTER_PLEXI
GPIO_PinState pin_state = 0;
#endif

void ADCCompleteCallback(ADC_HandleTypeDef* hadc)
{
	db_ADCCompleteCallback++;

#ifdef CONVERTER_PLEXI
	pin_state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
#endif

	// convert 12 bit integer values to the voltage measured at input of ADC

	for( uint16_t k_tmp=0; k_tmp<3*AD_RANK; k_tmp++) {
		ad_volt_float[k_tmp] = 3.3/4096.0*(float)ad_dma_buffer[k_tmp];
	}

	// Run state machine

	switch(myconvvsi.sm) {

	// Initialize control system and move to next state

	case SMInit:
		myconvvsi.k = 0;
		InitControlProcess();
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);		// reset cmd
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);		// set 0 to Vi- (not used)
		DisplayMessageInit();
		StartPWMFan();
		initInverterManagementStructure(inverterConfiguration);
		initSineLookupTable();
		myconvvsi.sm = SMWaitForRun;
		break;

	// Wait for the user to give the start order from the PC via USB and UART

	case SMWaitForRun:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);			// reset clear cmd
		if( myconvvsi.signal_run) {
			myconvvsi.sm = SMStartPWM;
			myconvvsi.k = 0;
			myconvvsi.signal_run = false;
			DisplayMessageRun();
		}
		break;

	case SMStartPWM:
		if( myconvvsi.k == 1) {
			StartPWM();

			// Wait 10 periods for bootstrap capacitors to charge before running

		} else if( myconvvsi.k > 10) {
			myconvvsi.k = 0;
			myconvvsi.sm = SMRun;
		}
		break;

	case SMRun:
		if( myconvvsi.signal_stop) {
			myconvvsi.k = 0;
			myconvvsi.sm = SMSoftStop;
			myconvvsi.signal_stop = false;
		}
#ifdef CONVERTER_PLEXI
		if( pin_state != GPIO_PIN_SET) {
			myconvvsi.sm = SMFault;
		}
#endif

		functionalTestRoutine(&myconvvsi);

		// call you control function here

		RunControlAtEachSamplingPeriodEx1();
		break;

	case SMSoftStop:
		if( myconvvsi.ref0 > 0) {
			myconvvsi.k = 0;
			myconvvsi.ref0 -= 0.002;
			if( myconvvsi.ref0 < 0) {
				myconvvsi.ref0 = 0;
			}
		} else if( myconvvsi.ref>0) {
			myconvvsi.k = 0;
			myconvvsi.ref -= 0.002;
			if( myconvvsi.ref < 0) {
				myconvvsi.ref = 0;
			}
		} else if(myconvvsi.k>10){
			HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
		} else if(myconvvsi.k>15){
			myconvvsi.sm = SMStop;
		}
#ifdef CONVERTER_PLEXI
		if( pin_state != GPIO_PIN_SET) {
			myconvvsi.sm = SMFault;
		}
#endif
		break;

	case SMFault:
		StopPWM();
		DisplayMessageFault();
		myconvvsi.sm = SMFaultSignal;
		break;

	case SMFaultSignal:
		break;

	case SMStop:
		StopPWM();
		DisplayMessageStop();
		myconvvsi.sm = SMStopSignal;
		break;

	case SMStopSignal:
		if( myconvvsi.signal_run) {
			myconvvsi.sm = SMStartPWM;
			myconvvsi.k = 0;
			myconvvsi.signal_run = false;
			DisplayMessageRun();
		}
		break;
	}

	// Apply duty cycles on Timer 1

	float arr = TIM1->ARR+1;

	TIM1->CCR1 = arr * myconvvsi.da[0]+0.5;
	TIM1->CCR2 = arr * myconvvsi.da[1]+0.5;
	TIM1->CCR3 = arr * myconvvsi.da[2]+0.5;
	TIM1->CCR4 = arr * myconvvsi.da[3]+0.5;
	TIM1->CCR5 = arr * myconvvsi.da[2]+0.5;
	TIM1->CCR6 = arr * myconvvsi.da[3]+0.5;

	// Apply duty cycles on Timer 8

	TIM8->CCR1 = arr * myconvvsi.db[0]+0.5;
	TIM8->CCR2 = arr * myconvvsi.db[1]+0.5;
	TIM8->CCR3 = arr * myconvvsi.db[2]+0.5;
	TIM8->CCR4 = arr * myconvvsi.db[3]+0.5;
	TIM8->CCR5 = arr * myconvvsi.db[2]+0.5;
	TIM8->CCR6 = arr * myconvvsi.db[3]+0.5;

	// Log measurements

	LogMeasurementsExample();

	// Increase time index

	myconvvsi.k++;

}



uint32_t db_ADCErrorCallback = 0;

void ADCErrorCallback(ADC_HandleTypeDef* hadc)
{
	db_ADCErrorCallback++;
}


uint32_t db_ADCHalfCompleteCallback = 0;

void ADCHalfCompleteCallback(ADC_HandleTypeDef* hadc)
{
	db_ADCHalfCompleteCallback++;
}


uint32_t db_HAL_ADC_ConvCpltCallback = 0;

// Required because Ex do not use the callback

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	db_HAL_ADC_ConvCpltCallback++;
	ADCCompleteCallback( hadc);
}


void LogMeasurementsExample()
{
	// log measurements

	if( myconvvsi.sm>=SMStartPWM) {
		for( uint16_t k_tmp=0; k_tmp<NB_AD_VALUES; k_tmp++) {
			if( db_cnt_meas<LOG_MEAS_NB) {
			//	db_meas[db_cnt_meas++] = ad_volt_float[k_tmp];
			}
		}
	}

}

