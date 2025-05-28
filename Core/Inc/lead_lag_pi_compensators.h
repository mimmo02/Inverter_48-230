/*
 * lead_lag_pi_compensators.h
 *
 *  Created on: Dec 19, 2023
 *      Author: Sebastien
 */

#ifndef INC_LEAD_LAG_PI_COMPENSATORS_H_
#define INC_LEAD_LAG_PI_COMPENSATORS_H_


// structure definition with all lead parameters and variables

typedef struct {
	float x[2];
	float y[2];
	float z[3];
} TCompensator;


typedef struct {
	float Tz, Tp, Ts;
} TLeadLagCompensatorInit;


void LeagLagPICompensator( TCompensator *comp);



#endif /* INC_LEAD_LAG_PI_COMPENSATORS_H_ */
