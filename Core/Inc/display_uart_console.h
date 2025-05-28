/*
 * display_uart_console.h
 *
 *  Created on: Mar 19, 2023
 *      Author: Sebastien
 */

#ifndef INC_DISPLAY_UART_CONSOLE_H_
#define INC_DISPLAY_UART_CONSOLE_H_

#include "main.h"
#include "stdint.h"
#include "stdbool.h"




void DisplayMessageMeas();
void DisplayMessageInit();
void DisplayMessageRun();
void DisplayMessageFault();
void DisplayMessageStop();
void InitDisplayUartConsole();

#define UART3_READ_BUFFER_SIZE 20
#define UART3_WRITE_BUFFER_SIZE 100
extern uint8_t uart3rxbuffer[UART3_READ_BUFFER_SIZE];
extern uint8_t uart3txbuffer[UART3_WRITE_BUFFER_SIZE];
extern UART_HandleTypeDef huart3;




#endif /* INC_DISPLAY_UART_CONSOLE_H_ */
