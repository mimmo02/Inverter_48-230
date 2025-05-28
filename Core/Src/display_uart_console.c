/*
 * display_uart_console.c
 *
 *  Created on: Mar 19, 2023
 *      Author: Sebastien
 */


#include "display_uart_console.h"
#include "power_converter_control_system.h"
#include "string.h"
#include "stdio.h"

#ifdef STM32H743xx
#define memory_place_uart3 __attribute__ ((section (".sdramD1")))
#else
#define memory_place_uart3
#endif

uint8_t uart3rxbuffer[UART3_READ_BUFFER_SIZE] memory_place_uart3;
uint8_t uart3txbuffer[UART3_WRITE_BUFFER_SIZE] memory_place_uart3;


const char start_str[] = "Init converter control\r\n";
const char start_pwm_str[] = "Starting PWM\r\n";
const char fault_str[] = "!Fault detected!\r\n";
const char stop_str[] = "Stop converter control\r\n";


void DisplayMessageMeas()
{
	if( myconvvsi.sm == SMRun) {
		sprintf( (char *)uart3txbuffer, "%d u123 = %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g\r\n",
				(int)myconvvsi.k,
				(double)myconvvsi.umeas[0], (double)myconvvsi.umeas[1], (double)myconvvsi.umeas[2],
				(double)myconvvsi.ilemf[0], (double)myconvvsi.ilemf[1], (double)myconvvsi.ilemf[2],
				(double)myconvvsi.ilegf[0], (double)myconvvsi.ilegf[1], (double)myconvvsi.ilegf[2]
																												  );
		HAL_UART_Transmit_DMA( &huart3, (uint8_t *)&uart3txbuffer[0], (uint16_t)strlen( (char *)uart3txbuffer));
	}
}


void DisplayMessageInit()
{
	sprintf( (char *)uart3txbuffer, start_str);
	HAL_UART_Transmit_DMA( &huart3, (uint8_t *)&uart3txbuffer[0], (uint16_t)sizeof(start_str)-1);
}


void DisplayMessageRun()
{
	sprintf( (char *)uart3txbuffer, start_pwm_str);
	HAL_UART_Transmit_DMA( &huart3, (uint8_t *)&uart3txbuffer[0], (uint16_t)sizeof(start_pwm_str)-1);
}


void DisplayMessageFault()
{
	sprintf( (char *)uart3txbuffer, fault_str);
	HAL_UART_Transmit_DMA( &huart3, (uint8_t *)&uart3txbuffer[0], (uint16_t)sizeof(fault_str)-1);
}


void DisplayMessageStop()
{
	sprintf( (char *)uart3txbuffer, stop_str);
	HAL_UART_Transmit_DMA( &huart3, (uint8_t *)&uart3txbuffer[0], (uint16_t)sizeof(stop_str)-1);
}


typedef struct{
	uint32_t cnt;

} TCBStatus;


static TCBStatus huart3Txstatus, huart3Rxstatus;


void huart3TxCpltCallback(struct __UART_HandleTypeDef *huart)
{
	huart3Txstatus.cnt++;
}


void huart3RxEventCallback(struct __UART_HandleTypeDef *huart, uint16_t Pos)
{
}


#define LENGTH_DB_BUFFER_UART3 20
uint8_t db_buffer_uart3[LENGTH_DB_BUFFER_UART3+2];
uint16_t db_cnt_buffer_uart3 = 0;


void huart3RxCpltCallback(struct __UART_HandleTypeDef *huart)
{
	huart3Rxstatus.cnt++;

	// process the character coming from uart

	db_buffer_uart3[db_cnt_buffer_uart3++] = uart3rxbuffer[0];
	if( db_cnt_buffer_uart3 >= LENGTH_DB_BUFFER_UART3)
		db_cnt_buffer_uart3 = 0;

	switch( uart3rxbuffer[0]) {
	case 'r':
		myconvvsi.signal_run = true;
		break;
	case 's':
		myconvvsi.signal_stop = true;
		break;
	case '+':
	case '1':
		if( myconvvsi.ref<0.95)
			myconvvsi.ref += 0.01;
		break;
	case '-':
	case '2':
		if( myconvvsi.ref>0.0)
			myconvvsi.ref -= 0.01;
		break;
	case 'p':
		if( myconvvsi.ref0<0.45)
			myconvvsi.ref0 += 0.01;
		break;
	case 'm':
		if( myconvvsi.ref0>0.0)
			myconvvsi.ref0 -= 0.01;
		break;
	case 'f':
		if( myconvvsi.ref1<0.1)
			myconvvsi.ref1 += 0.01;
		break;
	case 'g':
		if( myconvvsi.ref1>-0.1)
			myconvvsi.ref1 -= 0.01;
		break;
	case '5':
		if( myconvvsi.mode<2)
			myconvvsi.mode++;
		break;
	case '6':
		if( myconvvsi.mode>0)
			myconvvsi.mode--;
		break;
	default:
		;
	}
	HAL_UART_Receive_IT(&huart3, &uart3rxbuffer[0], 1);
}


void InitDisplayUartConsole()
{
	huart3.TxCpltCallback = huart3TxCpltCallback;
	huart3.RxCpltCallback = huart3RxCpltCallback;

	HAL_UART_Receive_IT(&huart3, &uart3rxbuffer[0], 1);
}
