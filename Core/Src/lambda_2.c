/*
 * lambda_2.c
 *
 *  Created on: 21 Feb 2022
 *      Author: DrMotor
 *
 *      This file will be used with Lambda Shield 2 from Bylund Automotive
 *      The code was initially based on Bylund's Arduino example, but very
 *      little of the original code remains now, therefore I have also deleted
 *      Bylund's copyright notice. Some main changes include:
 *
 *      1. STM32 under RTOS (in stead of Arduino)
 *      2. Bosch LSU ADV (another, and newer lambda sensor, with different characteristics)
 *      3. The use of STM32's hardware timer
  */
/*
 * 100 Hz heater PWM is recommended by Bosch.  This code uses a hardware timer for the PWM.
 * 100 Hz could for example be configured in CubeMX (or tim.c) as 96 MHz clock / ( 96 prescaler * 10000 reload )
 *
 * However here is used 1000 Hz = 96 MHz clock / ( 96 prescaler * 1000 reload )
 * Reason: We run the PWM as same frequency as we update the output.
 * We can make smoother curves by reading more often and low-pass filter the results.
 * UA, UR are updated at approx. 3000Hz (ref. CJ125 datasheet).
 */



//included headers
#include <SPI.h>
#include "lambda_2.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


//Define CJ125 registers used.
#define CJ125_IDENT_REG_REQUEST             0x4800        // Identify request, gives revision of the chip.
#define CJ125_DIAG_REG_REQUEST              0x7800        // Diagnostic request, gives the current status.
#define CJ125_INIT_REG1_REQUEST             0x6C00        // Requests the first init register.
#define CJ125_INIT_REG1_MODE_CALIBRATE      0x569D        // Sets calibration mode.
#define CJ125_INIT_REG1_MODE_NORMAL_V8      0x5688        // Sets operation mode with V=8 amplification.
#define CJ125_INIT_REG1_MODE_NORMAL_V17     0x5689        // Sets operation mode with V=17 amplification.
#define CJ125_INIT_REG2_REQUEST             0x7E00        // Requests the second init register.
#define CJ125_INIT_REG2_20uA		        0x5A02        // Pump reference current 20 uA ON
#define CJ125_DIAG_REG_STATUS_OK            0x00FF        // The response of the diagnostic register when everything is ok.
#define CJ125_DIAG_REG_STATUS_NOPOWER       0x0055        // The response of the diagnostic register when power is low.
#define CJ125_DIAG_REG_STATUS_NOSENSOR      0x007F        // The response of the diagnostic register when no sensor is connected.
#define CJ125_INIT_REG1_STATUS_0            0x0088        // The response of the init register when V=8 amplification is in use.
#define CJ125_INIT_REG1_STATUS_1            0x0089        // The response of the init register when V=17 amplification is in use.
#define	CJ125_OK  0x07ff  // like CJ125_DIAG_STATUS but with additional use of most significant byte

//Define adjustable parameters.
#define UBAT_MIN                   6.0f           // [V] Minimum voltage to operate. The CJ125 will also trigger a low-voltage fault independent of this
#define	LSU_DEBUG

//Pin assignments: _Pins and _GPIO_Ports are defined via CubeMX in main.h
// Define	short hand
#define set_power_led		LL_GPIO_SetOutputPin( LS_LED1_GPIO_Port, LS_LED1_Pin )
#define reset_power_led 	LL_GPIO_ResetOutputPin( LS_LED1_GPIO_Port, LS_LED1_Pin )
#define set_heater_led		LL_GPIO_SetOutputPin( LS_LED2_GPIO_Port, LS_LED2_Pin )
#define reset_heater_led 	LL_GPIO_ResetOutputPin( LS_LED2_GPIO_Port, LS_LED2_Pin )

//CJ125 variables
volatile uint16_t adcValue_UA = 0;                /* ADC value read from the CJ125 UA output pin */
const float k_IP = 3300.0f / (ADCrange * 61.9f * 17.0 );  // [mA/NV] 17.0 = amplifier setting in CJ125, 61.9 Ohm = shunt resistior
volatile uint16_t adcValue_UR = 0;                /* ADC value read from the CJ125 UR output pin */
volatile uint16_t adcValue_UB = 0;                /* ADC value read from the voltage divider calculating Ubat */
volatile float UA_delta, UR_delta;				  // Smoothed difference UA-UA0 and UR-UR0;
uint16_t UA0 = 0;                        /* UA ADC value stored when CJ125 is in calibration mode, λ=1 */
uint16_t UR0 = 0;                        /* UR ADC value stored when CJ125 is in calibration mode, optimal temperature */
uint16_t CJ125_diag = 0;                          /* Latest stored DIAG registry response from the CJ125 */
uint32_t LS_fault_counter = 0;
volatile float UBAT = 30.0f;
float UR = 0.0f;
volatile float T_LSU = 0.0f;    				/* [degC] Temperature calculated based on adcValue_UR */
float IP = 0.0f;
float FuelExcess = 0.0f;
float Lambda = 0.0f;
volatile float Heater_power = 0.0f;
//float P_heater_max = 0.0f;
float R_heater = 3.0;  // [Ohm] initial value = measured at room temp. on a LSU ADV

void LS_delay(const unsigned int ms)
{
	vTaskDelay( ms / portTICK_PERIOD_MS );
	//HAL_Delay(ms);
}


void set_heater( const float P )
{
	//float duty = ( U_RMS * U_RMS ) / ( UBAT * UBAT );
	float duty = (P * R_heater) / ( UBAT * UBAT );
	if (duty > 1.0f) duty = 1.0f;
	if (duty < 0.0f) duty = 0.0f;
	LL_TIM_OC_SetCompareCH2( TIM4, (uint16_t) ( duty * LL_TIM_GetAutoReload(TIM4) ) );
}

//Function for transferring SPI data to and from the CJ125.  Using polling (blocking) transfer
uint16_t CJ125_COM_SPI( uint16_t TX_data )
{
	//Configure SPI for CJ125 controller.  -- moved to spi.c (config is done via CubeMX)
	LL_GPIO_ResetOutputPin( CJ125_NSS_GPIO_Port, CJ125_NSS_Pin );   //Set chip select pin low, chip in use.
	//LS_delay(1);
	LL_SPI_ReceiveData16( CJ125_dev ); // clear RX register
	LL_SPI_TransmitData16( CJ125_dev, TX_data );
	while ( CJ125_dev->SR & SPI_SR_BSY );  // wait...
	LL_GPIO_SetOutputPin( CJ125_NSS_GPIO_Port, CJ125_NSS_Pin );    //Set chip select pin high, chip not in use.
	return ( LL_SPI_ReceiveData16( CJ125_dev ) & 0x00ff );
}

float LSU_ADV_temp_linear()
{
	return 785.0f - ( UR_delta * 0.136459 );  // See LSU_ADC.docx
}

struct PID{
	const float kP;
	const float kI;
	const float kD;
	float iMin, iMax;
	float min, max;
	float iState, last_delta;
};


struct PID heater={
		.kP = 0.1f,     // [W/NV] Proportional gain.
		.kI = 0.05f,
		.kD = 0.0f,
		.iMin = 0.0f,
		.iMax = 12.0f,
		.min = 0.0f,
		.max = 15.0f,
		.iState = 0.0f,
		.last_delta = 0.0f
};

// ToDo: move variables that will be needed for each CJ125 into
// a struct in order to prepare for the use of more than one LSU ADV.
struct CJ125{
	volatile float T_LSU;
	// ..etc, etc.
};




// Temperature regulating PID.  Return value = heater power [W] to be applied to the LSU
float PID_Control( struct PID pid, float delta )
{
  float p = pid.kP * delta;   //Calculate proportional term.
  pid.iState += pid.kI * p;
  if (pid.iState > pid.iMax ) pid.iState=pid.iMax;
  if (pid.iState < pid.iMin ) pid.iState=pid.iMin;
  float sum = p + pid.iState + pid.kD * ( delta - pid.last_delta );
  //float sum = p + iState;
  pid.last_delta = delta;
  if (sum < pid.min ) sum = pid.min;
  if (sum > pid.max ) sum = pid.max;
  return sum;  // [W]
}


/*@brief Calculation of fuel excess ratio
 * > 0.0 ---> rich, e.g. 0.3 means 30% fuel excess relative to stoichiometric
 * < 0.0 ---> lean, e.g. -0.2 means 20% fuel is "missing" for stoichiometric combustion
 *@param IP [mA] measured pump current
 */
float fuel_excess_ratio( const float IP )
{
    float F_rich = -0.3761f * IP - 0.01f;  // parameters based on LSU ADV datasheet
    float F_lean = -0.7577f * IP - 0.02f;
    if ( F_lean < F_rich ) return F_lean;
    else return F_rich;
}


/* @brief Write Numeric Broadcast Protocol (NBP) update message
 *
 *  Example message:
 *  *NBP1,UPDATE,2356.567
 *  "Battery","V":13.56
 *  "Brake Pedal","%":100.0
 *  #
*/
void Lambda_nbp_sprintf( char buf[] )
{
	uint32_t curTime = HAL_GetTick();
	sprintf(buf,"*NBP1,UPDATEALL,%d.%03d\n", (int)(curTime / 1000), (int)(curTime % 1000) );
	sprintf(buf + strlen(buf),"\"Fuel Excess\",\"%%\":%.1f\n", FuelExcess*100.0f );
	sprintf(buf + strlen(buf),"\"Battery\",\"V\":%.1f\n", UBAT );
#ifdef LSU_DEBUG
	sprintf(buf + strlen(buf),"\"CJ125_diag\",\"\":%d\n", CJ125_diag );
	sprintf(buf + strlen(buf),"\"LS_Faults\",\"\":%ld\n", LS_fault_counter );
	sprintf(buf + strlen(buf),"\"IP\",\"\":%.1f\n", IP );
	sprintf(buf + strlen(buf),"\"UA\",\"\":%.1f\n", UA_delta );
	sprintf(buf + strlen(buf),"\"UA0\",\"\":%d\n", UA0 );
	sprintf(buf + strlen(buf),"\"UR_delta\",\"\":%.1f\n", UR_delta );
	sprintf(buf + strlen(buf),"\"UR0\",\"\":%d\n", UR0 );
	sprintf(buf + strlen(buf),"\"Heater_power\",\"W\":%.1f\n", Heater_power );
#endif //LSU_DEBUG
	sprintf(buf + strlen(buf),"\"LSU Temp\",\"C\":%.1f\n#\n", T_LSU );
}

/* @brief Write fuel excess, battery voltage and LSU temperature to a string (text buffer)
*/
void Lambda_sprintf( char buf[] )
{
	sprintf(buf,",%.2f,%.1f,%.1f,%.1f", Lambda, UBAT, T_LSU, Heater_power );
}


void heat_up()
{
	//Condensation phase,  2 W for 10 s (or should it be until reaching a temperature ?)
	R_heater = 3.0;  // [Ohm] measured at room temp
	heater.max = 2.0f;
	for (int t=0; t < 100; t++ )
	{
		set_heater_led;  //Flash Heater LED in condensation phase.
		LS_delay( 25 );
		reset_heater_led;
		LS_delay( 75 );
	}


	// Heat at high power until 640 degC is reached.
	R_heater = (7.5 * 7.5) / 8.7;  // [Ohm] estimated based on 8.7 W at 7.5 V -- ref. LSU ADV product info
	heater.max = 15.0f;

	do {
		set_heater_led;
		LS_delay( 75 );
		reset_heater_led;
		LS_delay( 25 );
	} while ( ( T_LSU < 640.0f ) && ( UBAT > UBAT_MIN) );

    // Enable reference pump current.  Quote from "LSU ADV Gasoline" page 3:
	// "Pumping current IP must only be activated after the heater has been switched on and the sensor
	// element has reached a sufficient temperature (internal resistance of λ=1 Nernst cell Ri,N ≤ 2.3 kOhm )."
	//
	// 2.3 kOhm is approx. 640 degC -- which we have reached at this point.
	CJ125_COM_SPI( CJ125_INIT_REG2_20uA );

	// Constant 12.5W for 1.5 s
	heater.max = 12.5f;
	for (int i=0; i<15; i++ )
	{
		set_heater_led;
		LS_delay( 50 );
		reset_heater_led;
		LS_delay( 50 );
	}

	// Ramp up phase to 750.0 degC
	// 1 W/s until 20 W from 12.5 W
	heater.max = 12.5f;
	do {
		set_heater_led;
		LS_delay( 50 );
		reset_heater_led;
		LS_delay( 50 );
		if ( heater.max <= 19.91f ) heater.max += 0.1f; //Increase power by 0.1 W
	} while ( ( T_LSU < 750.0f) && (UBAT > UBAT_MIN) );
}

void calibrate()
{
	heater.max = 0.0f;  // turn off the heat during calibration
	LS_delay( 100 );
	do  //Wait until power is present and CJ125 reports ready.
	{
		CJ125_diag = CJ125_COM_SPI(CJ125_DIAG_REG_REQUEST);  //Read CJ125 diagnostic register from SPI.
		LS_delay( 100 );
	}
	while ( ( UBAT < UBAT_MIN)
			|| ( CJ125_diag != CJ125_DIAG_REG_STATUS_OK )
			);

	set_power_led; //Start of operation. Start Power LED.
	CJ125_COM_SPI( CJ125_INIT_REG1_MODE_CALIBRATE );    //Set CJ125 in calibration mode.
	UA0 = 0.0;
	UR0 = 0.0;
	LS_delay( 1000 );
	uint16_t REG1,REG2;
	do
	{   // Check that CJ125 is in calibration mode
		LS_delay( 100 );
		REG1 = CJ125_COM_SPI( CJ125_INIT_REG1_REQUEST );
		// 8 bits, expect 0b10011101: EN_HOLD=1, PA=0, 0, RA=1 (Ri cal.), EN_F3K=1, LA=1 (UA cal.), 0, VL=1 (gain 17);
	}
	while ( (REG1 & 0b10011100) != 0b10011100
			|| (UA_delta < 0x0300) // Check if the calibration values are plausible
			|| (UA_delta > 0x0800)
			|| (UR_delta < 0x0300)
			|| (UR_delta > 0x0800)
			);

	UA0 = UA_delta;   // The adcValues get updated by IRQ_handler
	UR0 = UR_delta;   // Store optimal values before leaving calibration mode

	//CJ125_COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V8);  /* V=0 */  // remember to edit the gain factor in the IP calc. if you use this.
	CJ125_COM_SPI( CJ125_INIT_REG1_MODE_NORMAL_V17 );  /* V=1 */  //Set CJ125 in normal operation mode with gain = 17

	do
	{   // Check that CJ125 is in normal measurement mode
		REG1 = CJ125_COM_SPI( CJ125_INIT_REG1_REQUEST );
		// expect 0b10001001: EN_HOLD=1, PA=0, 0, RA=0 (Ri cal.), EN_F3K=1, LA=0 (UA cal.), 0, VL=1 (gain 17);

		REG2 = CJ125_COM_SPI( CJ125_INIT_REG2_REQUEST );  // expect 0b00000000
	}
	while ( (REG1 & 0b10011100) != 0b10001000 );
}


void fault_handler( uint16_t status )
{
	LS_fault_counter++;
	do{  // Stay in the fault handler, fast alternating the LEDs, until CJ125_diag == CJ125_DIAG_REG_STATUS_OK
		reset_power_led;
		set_power_led;
		LS_delay( 50 );
		reset_power_led;
		LS_delay( 50 );
		heater.iState = 8.5;        // [W]  Adjust the integral state.
		if ( (CJ125_diag & 0x0200) == 0 )  // The LSU is getting too hot  ??
		{
			set_heater( 0.0f );  // [W]  Temporarily turn heater off.
			CJ125_diag |= 0x0200;
			return; // no need to re-heat and calibrate
		}
		CJ125_diag = CJ125_COM_SPI( CJ125_DIAG_REG_REQUEST ); //Update CJ125 diagnostic via SPI.
	} while ( CJ125_diag != CJ125_DIAG_REG_STATUS_OK );
	calibrate();
	heat_up();
}


void Lambda_task()  // to be called by OS
{
	//Start of operation. (Test LED's).
	set_power_led;
	set_heater_led;
	LS_delay( 200 );
	reset_power_led;
	reset_heater_led;

	//LL_GPIO_SetPinPull(CJ125_NSS_GPIO_Port, CJ125_NSS_Pin, LL_GPIO_PULL_UP);
	if( !LL_SPI_IsEnabled( CJ125_dev ) ) LL_SPI_Enable( CJ125_dev );

	heater.max = 0.0f;  // no heater output for now
	set_heater( 0.0f );

	// TIM4 TRGO triggers injected ADC channels
	// TIM4 CH2 controls the heater PWM
	LL_ADC_EnableIT_JEOS( ADC1 );
	LL_TIM_CC_EnableChannel( TIM4, LL_TIM_CHANNEL_CH2 );
	LL_TIM_EnableCounter( TIM4 );
	LL_TIM_GenerateEvent_UPDATE( TIM4 );

	calibrate();
	heat_up();
	set_power_led;
	reset_heater_led;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		vTaskDelayUntil( &xLastWakeTime, 10 );  // 10 ms between
		IP = k_IP * UA_delta;  // [mA] pump current
		FuelExcess = fuel_excess_ratio( IP );
		Lambda = 1.0f / ( FuelExcess + 1.0f );

		CJ125_diag = CJ125_COM_SPI( CJ125_DIAG_REG_REQUEST ); //Update CJ125 diagnostic
		if ( UBAT >= UBAT_MIN ) CJ125_diag |= 0x0100; // set a bit indicating that the voltage read by MCU is OK
		if ( T_LSU < 900.0f)    CJ125_diag |= 0x0200; // LSU is not too hot for accuracy and durability
		if ( T_LSU > 400.0f)    CJ125_diag |= 0x0400; // LSU is not way too cold (cold is OK only during start-up)
		if ( CJ125_diag != CJ125_OK )
			{
				fault_handler( CJ125_diag );
			}
	}
}

float LSU_ADV_temp_Steinhart()
{
	// Estimation of LSU temperature using a Steinhart–Hart equation
	//const float R0=300.0f;           // Thermistor nominal resistance at T0
	//const float T0=785.0f + 273.15f; // Temperature at nominal resistance [K]
	//const float Beta=9000.0f;        // The beta coefficient of the thermistor -- approx value from graph in datasheet
	//float R = R0 * ( 1.0f + ( adcValue_UR - UR0 ) * ( 1000.0f / ( ADCrange * 15.5f ) ) ); //[Ohm] Thermistor resistance
	//float temp = log(R / R0) / Beta + 1.0f/T0;      // ln(R/R0) / B  + 1/T0
	const float T0_inv = 1.0f/(785.0f + 273.15f); //  [1/K] Inverse of temperature at nominal resistance
	const float Beta_inv = 1.0f/9000.0f;    // Inverse of beta coefficient of the thermistor -- approx value from graph in datasheet
	const float k_UR = 15.5; // UR amplifier gain -- ref CJ125 data
	const float I_RM = 5.0/31.6e3; // [A] Injected current (158 uA) for measuring cell resistance
	const float R0 = 300.0f;  // [Ohm]
	const float kR = 3.3f / ( ADCrange * k_UR * I_RM * R0 ); // [-]
	float R_rel = 1.0f + kR * ( UR_delta ); //[-] R/R0 resistance ratio
	float T_inv = logf(R_rel) * Beta_inv + T0_inv;         // [1/K]  1/T = ln(R/R0) / B  + 1/T0
	if ( isnan(T_inv) ) return 785.0f;
	else return 1.0f / T_inv - 273.15f;  // Invert and convert to degC
}



void Lambda_IRQHandler()  // to be called by ADC injected group EOC (end of conversion)
{
	adcValue_UB = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
	adcValue_UR = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
	adcValue_UA = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);
	// UBAT = ( 3.3f * 110.0f / ( ADCrange * 10.0f ) ) * adcValue_UB ;
	UBAT = 0.99 * UBAT + 0.01 * adcValue_UB * 3.3f * 110.0f / ( ADCrange * 10.0f ); // [V] smooth=0.01, VDD=3.3, 10k and 100k resistors
	if ( UBAT < 1.0 ) UBAT=1.0; // avoid division by zero on other parts of the code
	UA_delta = 0.99*UA_delta + 0.01*(adcValue_UA - UA0);  // make smooth curve after subtracting ref. value
	UR_delta = 0.95*UR_delta + 0.05*(adcValue_UR - UR0);  // make smooth curve after subtracting ref. value
	T_LSU = LSU_ADV_temp_linear();  // [degC]
	Heater_power = PID_Control( heater, UR_delta );  // [W] Calculate new heater output target.
	set_heater( Heater_power );
}

