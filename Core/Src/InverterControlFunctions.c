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
#define FREQ_SINE	250			// [Hz]
#define T_PWM		1/FREQ_PWM  // [s]
#define T_SMPL		1/FREQ_SMPL	// [s]
#define T_SINE		1/FREQ_SINE // [s]
#define N	FREQ_SMPL/FREQ_SINE	// N = number of samples, T_SINE = N * T_SAMPLE

// Electrical constants
#define R_LOAD		5.0			// [ohm]
#define L_LOAD		0.000160	// [H]
#define U_DC		10.0		// [V]
#define SINE_AMPL	8.0			// [V]
#define I_PEAK		4			// [A]
#define Z_LOAD		sqrt(pow(R_LOAD,2)+pow(2*M_PI*FREQ_SINE*L_LOAD,2))	// [ohm] Load impedance

// analog measurement constants
#define ADC_OFFSET	    1.65		// [V] (from PeLab interface PCB - differential measurement)
#define GAIN_MEAS_UDC	488.3		// [V/V]
#define GAIN_MEAS_UAC	400.5		// [V/V]
#define GAIN_MEAS_IAC	60.61		// [V/A]




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

// measuremets
float udc = 0;
float uac = 0;
float iac = 0;

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
			INV->d_a = 0.5 + (0.5*INV->delta_d);
			INV->d_b = 0.5 - (0.5*INV->delta_d);
			INV->d_c = 0.0;
		}
		else if(INV->Leg_B == false && INV->Leg_C == true){				// leg A and leg C switching
			INV->d_a = 0.5 + 0.5*INV->delta_d;
			INV->d_c = 0.5 - 0.5*INV->delta_d;
			INV->d_b = 0.0;
		}
	}
	else{																// totem pole control strategy
		if(INV->Leg_B == true && INV->Leg_C == false){					// leg A and leg B switching
			if(INV->delta_d > 0.0){										// positive half-wave
				INV->d_a = INV->delta_d;
				INV->d_b = 0.0;
				INV->d_c = 0.0;
			}
			else{														// negative half-wave
				INV->d_a = 1.0 + INV->delta_d;
				INV->d_b = 1.0;
				INV->d_c = 0.0;
			}
		}
		else if(INV->Leg_B == false && INV->Leg_C == true){				// leg A and leg C switching
			if(INV->delta_d > 0.0){										// positive half-wave
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

float ACCurrentMeasProcessing(float ad_volt){
	float u_diff = ad_volt - ADC_OFFSET;
	float eq_i = u_diff * GAIN_MEAS_IAC;
	return eq_i;
}

float ACVoltageMeasProcessing(float ad_volt){
	float u_diff = ad_volt - ADC_OFFSET;
	float eq_uac = u_diff * GAIN_MEAS_UAC;
	return eq_uac;
}

float DCVoltageMeasProcessing(float ad_volt){
	float u_diff = ad_volt - ADC_OFFSET;
	float eq_udc = u_diff * GAIN_MEAS_UDC;
	return eq_udc;
}

void AnalogMeasRoutine(){
	float u_meas_uac = ad_volt_float[0];	// ADC voltage level ADC1 CH5	rank 1
	float u_meas_udc = ad_volt_float[1];	// ADC voltage level ADC2 CH14	rank 1
	float u_meas_iac = ad_volt_float[2];	// ADC voltage level ADC3 CH4	rank 1

	udc = DCVoltageMeasProcessing(u_meas_udc);
	uac = ACVoltageMeasProcessing(u_meas_uac);
	iac = ACCurrentMeasProcessing(u_meas_iac);

	// store measurements in memory to export them
	if( db_cnt_meas<LOG_MEAS_NB) {
		db_meas[db_cnt_meas++] = u_meas_uac;
		db_meas[db_cnt_meas++] = u_meas_udc;
		db_meas[db_cnt_meas++] = u_meas_iac;
		db_meas[db_cnt_meas++] = udc;
		db_meas[db_cnt_meas++] = uac;
		db_meas[db_cnt_meas++] = iac;
	}
}

void FaultHandlingRoutine(myInverterCtrlStruct *INV){
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)){				// gate driver fault
		INV->system_fault = GD_FAULT;
	}
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)){				// over current meas fault
		INV->system_fault = CURRENT_MEAS_OCF;
	}
}


void signalsManagmentRoutine(){
	// Fault handling
	FaultHandlingRoutine(&myInverter);
	// meas handling
	AnalogMeasRoutine();
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

// functional test execution function
void functionalTestRoutine(TmyconvVSI *converter){
	static int i = 0;
	static int period_counter = 0;

	// signals handling
	signalsManagmentRoutine();

	/*
	if(myInverter.system_fault != NO_ERROR)
		converter->sm = SMFault;
	 */

	float udc = U_DC;		// fixed DC voltage value
	//float udc = u_dc_ref;	// measured DC voltage value

	compute_duty_cycle(&myInverter, i, (float)SINE_AMPL, udc);

	// Fixed duty cycle test
	//myInverter.d_a = 0.90;
	//myInverter.d_b = 0.75;
	//myInverter.d_c = 0.60;

	// converter.da used for higher semiconductors
	// converter.db used for lower semiconductors	- complementary (it is sufficient to control da)

	converter->da[0] = 1.0 - myInverter.d_a;  // Update leg A
	converter->da[1] = 1.0 - myInverter.d_b;  // Update leg B
	converter->da[2] = 1.0 - myInverter.d_c;  // Update leg C

	converter->db[0] = 1.0 - myInverter.d_a;  // Update leg A
	converter->db[1] = 1.0 - myInverter.d_b;  // Update leg B
	converter->db[2] = 1.0 - myInverter.d_c;  // Update leg C


	// index and periods counter management
	i++;
	if(i==N){
		i=0;
		period_counter++;
	}
}

