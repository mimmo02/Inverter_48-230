/*
 * ll_can.h
 *
 *  Created on: 2 mars 2020
 *      Author: Marc
 */

#ifndef INC_LL_CAN_H_
#define INC_LL_CAN_H_

//#include <ll_can_procedures.h>
#include "stdint.h"

void ll_CanInitCallback();
void ll_CanSetupFilter();
void canSendMessage(int ID, uint8_t * data, int l);
//void ll_CanInitController(TCanSpeedConfig canSpeed);
void llCan1SendNext();

#endif /* INC_LL_CAN_H_ */
