/*
 * system_1kHz_control.c
 *
 *  Created on: Mar 19, 2023
 *      Author: Sebastien
 */


//
// 1kHz system interrupt
//


#include "main.h"
#include "display_uart_console.h"
#include "process_1kHz_control.h"

int db_HAL_IncTick = 0;
int blink = 0;


void HAL_IncTick()
{
	uwTick += uwTickFreq;
	db_HAL_IncTick++;
	blink++;
	if(blink>1000) {
		blink = 0;
		DisplayMessageMeas();
	} else if( blink>500) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	}

}


uint32_t db_IdleProcess;

void IdleProcess()
{
	db_IdleProcess++;
}
