/*
 * InverterControlFunctions.h
 *
 *  Created on: May 31, 2025
 *      Author: Domenico Aquilino aquid1@bfh.ch
 */

#ifndef INC_INVERTERCONTROLFUNCTIONS_H_
#define INC_INVERTERCONTROLFUNCTIONS_H_

/*************************************************************************/
//  INCLUDES
/*************************************************************************/
# include "power_converter_control_system.h"

/*************************************************************************/
//  ENUMARATORS
/*************************************************************************/
typedef enum{
	CONFIG_A,		// Leg A and Leg B switching with symmetrical control strategy
	CONFIG_B		// Leg A and Leg C switching with totem pole control strategy (A fast switching, C slow switching)
}myInverterConfig;

/*************************************************************************/
//  PUBLIC FUNCTIONS
/*************************************************************************/
void initInverterManagementStructure(myInverterConfig config);
void initSineLookupTable();
void functionalTestRoutine(TmyconvVSI *converter);

#endif /* INC_INVERTERCONTROLFUNCTIONS_H_ */
