/*
 * InverterControlFunctions.c
 *
 *  Created on: May 31, 2025
 *      Author: Domenico Aquilino aquid1@bfh.ch
 */

/*************************************************************************/
//  INCLUDES
/*************************************************************************/
#include "InverterControlFunctions.h"

#include "Math.h"
#include "log_meas.h"
#include "power_converter_control_system.h"
#include "display_uart_console.h"
#include "stdbool.h"

/*************************************************************************/
//  DEFINES
/*************************************************************************/
// Duty cycle limit
#define DUTY_CYCLE_MAX_VALUE 1.0        // Max value of duty cycle
#define DUTY_CYCLE_MIN_VALUE 0.0		// Min value of duty cycle

// time and management constants
#define FREQ_PWM	10000   	// [Hz]
#define FREQ_SMPL	10000		// [Hz]
#define FREQ_SINE	50			// [Hz]
#define T_PWM		1/FREQ_PWM  // [s]
#define T_SMPL		1/FREQ_SMPL	// [s]
#define T_SINE		1/FREQ_SINE // [s]
#define N	FREQ_SMPL/FREQ_SINE	// N = number of samples, T_SINE = N * T_SAMPLE

// Electrical constants
#define R_LOAD		5.0			// [ohm]
#define L_LOAD		0.000160	// [H]
#define U_DC		10			// [V]
#define SINE_AMPL	5.0			// [V]
#define I_PEAK		4			// [A]
#define Z_LOAD		sqrt(pow(R_LOAD,2)+pow(2*M_PI*FREQ_SINE*L_LOAD,2))	// [ohm] Load impedance

// analog measurement constants
#define ADC_OFFSET	    1.65		// [V] (from PeLab interface PCB - differential measurement)
#define GAIN_MEAS_UDC	1.0			// [V/V]
#define GAIN_MEAS_UAC	1.0			// [V/V]
#define GAIN_MEASIAC	1.0			// [V/A]

/*************************************************************************/
//  ENUMARATORS
/*************************************************************************/
// Fault states
typedef enum{
	NO_ERROR,
	CURRENT_MEAS_OCF,
	GD_FAULT
}FaultType;

// control strategies definition
typedef enum{
	SYM,	// symmetrical control strategy
	ASYM	// totem pole control strategy
}CtrlType;

/*************************************************************************/
//  PRIVATE STRUCTURES
/*************************************************************************/
// inverter topology definition
typedef struct{
	FaultType system_fault;
	CtrlType system_ctrl_strategy;
	bool Leg_A;
	bool Leg_B;
	bool Leg_C;
	float delta_d;
	float d_a;
	float d_b;
	float d_c;
}myInverterCtrlStruct;

/*************************************************************************/
//  PRIVATE GLOBAL VARIABLES
/*************************************************************************/
/// Lookup table declaration
double sine_wave[N];

// inverter structure declaration
myInverterCtrlStruct myInverter;

/*************************************************************************/
//  PRIVATE FUNCTIONS
/*************************************************************************/
// duty cycle computation based on control strategy
void compute_duty_cycle(myInverterCtrlStruct *INV, int idx, float sineAmplitude, float udc){
	INV->delta_d = (sineAmplitude*sine_wave[idx])/udc;
	if(INV->system_ctrl_strategy == SYM){								// symmetrical control strategy
		if(INV->Leg_B == true && INV->Leg_C == false){					// leg A and leg B switching
			INV->d_a = 0.5 + 0.5*INV->delta_d;
			INV->d_b = 0.5 - 05*INV->delta_d;
			INV->d_c = 0.0;
		}
		else if(INV->Leg_B == false && INV->Leg_C == true){				// leg A and leg C switching
			INV->d_a = 0.5 + 0.5*INV->delta_d;
			INV->d_c = 0.5 - 05*INV->delta_d;
			INV->d_b = 0.0;
		}
	}
	else{																// totem pole control strategy
		if(INV->Leg_B == true && INV->Leg_C == false){					// leg A and leg B switching
			if(INV->delta_d > 0){										// positive half-wave
				INV->d_a = INV->delta_d;
				INV->d_b = 0.0;
				INV->d_c = 0.0;
			}
			else{														// negative half-wave
				INV->d_a = 1 + INV->delta_d;
				INV->d_b = 1.0;
				INV->d_c = 0.0;
			}
		}
		else if(INV->Leg_B == false && INV->Leg_C == true){				// leg A and leg C switching
			if(INV->delta_d > 0){										// positive half-wave
				INV->d_a = INV->delta_d;
				INV->d_c = 0.0;
				INV->d_b = 0.0;
			}
			else{														// negative half-wave
				INV->d_a = 1 + INV->delta_d;
				INV->d_c = 1.0;
				INV->d_b = 0.0;
			}
		}
	}

	// duty cycle value saturation
	if(INV->d_a > DUTY_CYCLE_MAX_VALUE || INV->d_a < DUTY_CYCLE_MIN_VALUE || INV->d_b > DUTY_CYCLE_MAX_VALUE || INV->d_a < DUTY_CYCLE_MIN_VALUE || INV->d_c > DUTY_CYCLE_MAX_VALUE || INV->d_c < DUTY_CYCLE_MIN_VALUE){
		if(INV->d_a > DUTY_CYCLE_MAX_VALUE)
			INV->d_a = DUTY_CYCLE_MAX_VALUE;
		else if(INV->d_a < DUTY_CYCLE_MIN_VALUE)
			INV->d_a = DUTY_CYCLE_MIN_VALUE;
		else if(INV->d_b > DUTY_CYCLE_MAX_VALUE)
			INV->d_b = DUTY_CYCLE_MAX_VALUE;
		else if(INV->d_b < DUTY_CYCLE_MIN_VALUE)
			INV->d_b = DUTY_CYCLE_MIN_VALUE;
		else if(INV->d_c > DUTY_CYCLE_MAX_VALUE)
			INV->d_c = DUTY_CYCLE_MAX_VALUE;
		else if(INV->d_c < DUTY_CYCLE_MIN_VALUE)
			INV->d_c = DUTY_CYCLE_MIN_VALUE;
	}
}

/*************************************************************************/
//  PUBLIC FUNCTIONS
/*************************************************************************/
// init invetrer managment structure
void initInverterManagementStructure(myInverterConfig config){
	myInverter.system_fault = NO_ERROR;
	myInverter.delta_d = 0.0;
	myInverter.d_a = 0.0;
	myInverter.d_b = 0.0;
	myInverter.d_c = 0.0;
	if(config == CONFIG_A){
		myInverter.system_ctrl_strategy = SYM;
		myInverter.Leg_A = true;
		myInverter.Leg_B = true;
		myInverter.Leg_C = false;
	}
	else{
		myInverter.system_ctrl_strategy = ASYM;
		myInverter.Leg_A = true;
		myInverter.Leg_B = false;
		myInverter.Leg_C = true;
	}
}

// Lookup table init function
void initSineLookupTable(){
	for (int i = 0; i < N; i++) {
		sine_wave[i] = sin(2.0 * M_PI * (double)i / ((double)N));
	}
}

void functionalTestRoutine(TmyconvVSI *converter){
	static int i = 0;
	static int period_counter = 0;

	float udc = U_DC;		// fixed DC voltage value
	//float udc = u_dc_ref;	// measured DC voltage value

	compute_duty_cycle(*myInverter, i, (float)SINE_AMPL, udc);

	// converter.da used for higher semiconductors
	// converter.db used for lower semiconductors	- complementary (it is sufficient to control da)

	converter->da[0] = myInverter.d_a;  // Update leg A
	converter->da[1] = myInverter.d_b;  // Update leg B
	converter->da[2] = myInverter.d_c;  // Update leg C

	// index and periods counter management
	i++;
	if(i==N){
		i=0;
		period_counter++;
	}
}



// duty cycle values update according to sine wave function
void update_duty_cycle(TmyconvVSI *converter, AmplitudeMode state, int i) {







	// change here to choose the mono phase legs
    converter->da[0] = da;  // Update leg A
    converter->da[1] = db;  // Update leg B
    converter->da[2] = 0;   // Update leg C

}

