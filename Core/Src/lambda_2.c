/*
 * lambda_2.c
 *
 *  Created on: 21 Feb 2022
 *      Author: viggo
 *
 *      This file will be used with Lambda Shield 2 from Bylund Automotive
 *      Some of the code is based on Bylund's Arduino example.
 *      1. STM32 under RTOS
 *      2. Bosch LSU ADV (lambda sensor)
 *      3. 100 Hz heater PWM (as recommended in LSU ADV datasheet)
 *
 */

/*
    Example code compatible with the Lambda Shield for Arduino.

    Copyright (C) 2017 - 2020 Bylund Automotive AB.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Contact information of author:
    http://www.bylund-automotive.com/

    info@bylund-automotive.com

    Version history:
    2017-12-30        v1.0.0        First release to GitHub.
    2018-10-25        v1.1.0        Implemented an improved lambda conversion.
    2019-04-19        v1.2.0        Implemented an improved oxygen conversion.
    2019-06-26        v1.3.0        Adjusted PID regulation of heater.
    2019-07-14        v1.3.1        Modified Lookup_Lambda() function.
    2019-07-14        v1.4.0        Implemented an analog output function.
    2020-03-26        v1.4.1        Optimized analog output function.
    2020-06-30        v1.5.0        Implemented support for data logging.

    2022-02-22		  v2.0.0beta	testing STM32G4
*/

//included headers
#include <SPI.h>
#include "lambda_2.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


//Define CJ125 registers used.
#define CJ125_IDENT_REG_REQUEST             0x4800        /* Identify request, gives revision of the chip. */
#define CJ125_DIAG_REG_REQUEST              0x7800        /* Diagnostic request, gives the current status. */
#define CJ125_INIT_REG1_REQUEST             0x6C00        /* Requests the first init register. */
#define CJ125_INIT_REG2_REQUEST             0x7E00        /* Requests the second init register. */
#define CJ125_INIT_REG1_MODE_CALIBRATE      0x569D        /* Sets the first init register in calibration mode. */
#define CJ125_INIT_REG1_MODE_NORMAL_V8      0x5688        /* Sets the first init register in operation mode. V=8 amplification. */
#define CJ125_INIT_REG1_MODE_NORMAL_V17     0x5689        /* Sets the first init register in operation mode. V=17 amplification. */
#define CJ125_DIAG_REG_STATUS_OK            0x00FF        /* The response of the diagnostic register when everything is ok. */
#define CJ125_DIAG_REG_STATUS_NOPOWER       0x0055        /* The response of the diagnostic register when power is low. */
#define CJ125_DIAG_REG_STATUS_NOSENSOR      0x007F        /* The response of the diagnostic register when no sensor is connected. */
#define CJ125_INIT_REG1_STATUS_0            0x0088        /* The response of the init register when V=8 amplification is in use. */
#define CJ125_INIT_REG1_STATUS_1            0x0089        /* The response of the init register when V=17 amplification is in use. */

#define	 CJ125_OK  0x07ff  // like CJ125_DIAG_STATUS but with additional use of most significant byte

//Define adjustable parameters.
#define UBAT_MIN                   6.0f           // [V] Minimum voltage to operate. The CJ125 will also trigger a low-voltage fault independent of this
#define	LSU_DEBUG

//Pin assignments: _Pins and _GPIO_Ports are defined via CubeMX in main.h
// Define	short hand
#define set_power_led		LL_GPIO_SetOutputPin( LS_LED1_GPIO_Port, LS_LED1_Pin )
#define reset_power_led 	LL_GPIO_ResetOutputPin( LS_LED1_GPIO_Port, LS_LED1_Pin )
#define set_heater_led		LL_GPIO_SetOutputPin( LS_LED2_GPIO_Port, LS_LED2_Pin )
#define reset_heater_led 	LL_GPIO_ResetOutputPin( LS_LED2_GPIO_Port, LS_LED2_Pin )

//Global variables.
volatile uint16_t adcValue_UA = 0;                /* ADC value read from the CJ125 UA output pin */
volatile uint16_t adcValue_UR = 0;                /* ADC value read from the CJ125 UR output pin */
volatile uint16_t adcValue_UB = 0;                /* ADC value read from the voltage divider calculating Ubat */
uint16_t adcValue_UA_Optimal = 0;                 /* UA ADC value stored when CJ125 is in calibration mode, λ=1 */
uint16_t adcValue_UR_Optimal = 0;                 /* UR ADC value stored when CJ125 is in calibration mode, optimal temperature */
uint16_t CJ125_diag = 0;                          /* Latest stored DIAG registry response from the CJ125 */
uint32_t LS_fault_counter = 0;
float UBAT = 30.0f;
float UR = 0.0f;
float T_LSU = 0.0f;    				/* [degC] Temperature calculated based on adcValue_UR */
float Ip = 0.0f;
float FuelExcess = 0.0f;
float FuelExcess_smo = 0.0f;
float Lambda = 0.0f;
float Heater_power = 0.0f;
float P_heater_max = 0.0f;

//PID regulation variables.
// STM32 vs Arduino: 4x higher input and 4x higher output. 1024 / 256  thus same gain, just increase integrator limits by x4
int dState=0;                                                       /* Last position input. */
float iState=8.0;                                                   /* [W] Integrator state. */
const float iMax = 12.0f;                                           /* [W] Maximum allowable integrator state. Default = 12.0 */
const float iMin =  0.0f;                                           /* [W] Minimum allowable integrator state. Default = 0.0 */
const float pGain = 0.1f;                                           /* [W/K] Proportional gain. Default = 1 */
const float iGain = 0.01f;                                          /* Integral gain. Default = 0.1 */
//const float dGain = 0.0f;                                           /* Derivative gain. Default = 10*/


void LS_delay(const unsigned int ms)
{
	vTaskDelay( ms / portTICK_PERIOD_MS );
	//HAL_Delay(ms);
}

/*
 * 100 Hz heater PWM is recommended by Bosch
 * 100 Hz = 96 MHz clock / ( 96 prescaler * 10000 reload )
 */
void set_heater( const float P )
{
	const float R_heater = (7.5 * 7.5) / 8.7;  // [Ohm] estimated based on 8.7 W at 7.5 V -- ref. LSU ADV product info
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
	//HW NSS    LL_GPIO_ResetOutputPin( CJ125_CS_GPIO_Port, CJ125_CS_Pin );   //Set chip select pin low, chip in use.
	LL_SPI_TransmitData16( CJ125_dev, TX_data );
	while (CJ125_dev->SR & SPI_SR_BSY);  // wait...
	//HW NSS   LL_GPIO_SetOutputPin( CJ125_CS_GPIO_Port, CJ125_CS_Pin );    //Set chip select pin high, chip not in use.
	return LL_SPI_ReceiveData16( CJ125_dev ) & 0x00ff;
}


// Temperature regulating PI (no D-term).  Return value = heater power [W] to be applied
// The sensor has a heat capacity C_heat,
// The control loop has a delay: T_loop = ?
float Heater_PID_Control()
{
  //const float C_heat = 0.05f; // [J/K] The sensor's heat capacity -- guessed value
  float error = T_LSU - 785.0f;   //Calculate error term.
  float pTerm = -pGain * error;   //Calculate proportional term.
  iState += iGain * pTerm;
  if (iState > iMax ) iState=iMax;
  if (iState < iMin ) iState=iMin;
  float sum = pTerm+iState;
  if (sum < 0.0f) sum = 0.0f;
  if (sum > P_heater_max ) sum = P_heater_max;
  return sum;  // [W]
}


/*@brief Calculation of fuel excess ratio
 * > 0.0 ---> rich, e.g. 0.3 means 30% fuel excess relative to stoichiometric
 * < 0.0 ---> lean, e.g. -0.2 means 20% fuel is "missing" for stoichiometric combustion
 *@param Ip [mA] measured pump current
 */
float fuel_excess_ratio( const float Ip )
{
    float F_rich = -0.3761f * Ip - 0.01f;  // parameters based on LSU ADV datasheet
    float F_lean = -0.7577f * Ip - 0.02f;
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
	sprintf(buf + strlen(buf),"\"Fuel Excess\",\"%%\":%.1f\n", FuelExcess_smo*100.0f );
	sprintf(buf + strlen(buf),"\"Battery\",\"V\":%.1f\n", UBAT );
#ifdef LSU_DEBUG
	sprintf(buf + strlen(buf),"\"CJ125_diag\",\"\":%d\n", CJ125_diag );
	sprintf(buf + strlen(buf),"\"LS_Faults\",\"\":%ld\n", LS_fault_counter );
	sprintf(buf + strlen(buf),"\"UR\",\"\":%d\n", adcValue_UR );
	sprintf(buf + strlen(buf),"\"UR_Optimal\",\"\":%d\n", adcValue_UR_Optimal );
	sprintf(buf + strlen(buf),"\"Heater_power\",\"W\":%.1f\n", Heater_power );
#endif //LSU_DEBUG
	sprintf(buf + strlen(buf),"\"LSU Temp\",\"C\":%.1f\n#\n", T_LSU );
}

/* @brief Write fuel excess, battery voltage and LSU temperature to a string (text buffer)
*/
void Lambda_sprintf( char buf[] )
{
	sprintf(buf,",%.1f,%.1f,%.1f", FuelExcess_smo*100.0f, UBAT, T_LSU);
}


void heat_up()
{
	//Condensation phase,  2 W for 10 s (or should it be until reaching a temperature ?)
	P_heater_max = 2.0f;
	for (int t=0; t < 100; t++ )
	{
		set_heater_led;  //Flash Heater LED in condensation phase.
		LS_delay( 25 );
		reset_heater_led;
		LS_delay( 75 );
		t += 1;
	}

	// Heat at high power until 640 degC is reached.
	P_heater_max = 20.0f;
	while ( (T_LSU < 640.0f ) && ( UBAT > UBAT_MIN) )
	{
		set_heater_led;
		LS_delay( 75 );
		reset_heater_led;
		LS_delay( 25 );
	}

	// Constant 12.5W for 1.5 s
	P_heater_max = 12.5f;
	for (int i=0; i<15; i++ )
	{
		set_heater_led;
		LS_delay( 50 );
		reset_heater_led;
		LS_delay( 50 );
	}

	// Ramp up phase to 785.0 degC,
	// 1 W/s until 20 W from 12.5 W
	P_heater_max = 12.5f;
	do {
		set_heater_led;
		LS_delay( 50 );
		reset_heater_led;
		LS_delay( 50 );
		if ( P_heater_max <= 19.91f ) P_heater_max += 0.1f; //Increase power by 0.1 W
	} while ( (T_LSU < 785.0f) && (UBAT > UBAT_MIN) );
}


void calibrate()  // todo: make this non-blocking via a state machine
{
	P_heater_max = 0.0f;  // turn off the heat during calibration
	do  //Wait until power is present and CJ125 reports ready.
	{
		CJ125_diag = CJ125_COM_SPI(CJ125_DIAG_REG_REQUEST);  //Read CJ125 diagnostic register from SPI.
		LS_delay( 100 );
	    //analogRead();      //Read input voltages.
	}
	while ( ( UBAT < UBAT_MIN) || (CJ125_diag != CJ125_DIAG_REG_STATUS_OK) );

	set_power_led; //Start of operation. Start Power LED.
	CJ125_COM_SPI( CJ125_INIT_REG1_MODE_CALIBRATE );    //Set CJ125 in calibration mode.
	LS_delay( 500 );         //Let values settle.

	adcValue_UA_Optimal = adcValue_UA;   //	The adcValues get updated by IRQ_handler
	adcValue_UR_Optimal = adcValue_UR;   // Store optimal values before leaving calibration mode.
	// ToDo: Check if the _Optimal values are plausible
	//CJ125_COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V8);  /* V=0 */  // remember to edit the gain factor in the Ip calc. if you use this.
	CJ125_COM_SPI( CJ125_INIT_REG1_MODE_NORMAL_V17 );  /* V=1 */  //Set CJ125 in normal operation mode with gain = 17
	// ToDo: read back REG1    if ( CJ125_COM_SPI( CJ125_INIT_REG1_REQUEST ) != )
}


void fault_handler( uint16_t status )
{
	LS_fault_counter++;
	do{  // Stay in the fault handler, fast alternating the LEDs, until CJ125_diag == CJ125_OK
		reset_power_led;
		set_power_led;
		LS_delay( 50 );
		set_power_led;
		reset_power_led;
		LS_delay( 50 );
		if ( (CJ125_diag & 0x0200) == 0 )  // The LSU is getting too hot
		{
			set_heater( 0.0f );  // [W]  Temporarily turn heater off.
			iState = 8.5;        // [W]  Adjust the integral state.
			return; // no need to re-heat and calibrate
		}
		CJ125_diag = CJ125_COM_SPI( CJ125_DIAG_REG_REQUEST ); //Update CJ125 diagnostic via SPI.
	} while ( CJ125_diag != CJ125_OK );
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

	LL_SPI_Enable( CJ125_dev );

	P_heater_max = 0.0f;  // no heater output for now
	LL_TIM_CC_EnableChannel( TIM4, LL_TIM_CHANNEL_CH2 ); // TIM4 CH2 controls the heater PWM
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
		Ip = ( 3.3f * 17.0f / ADCrange ) * adcValue_UA;
		FuelExcess = fuel_excess_ratio( Ip );
		FuelExcess_smo = 0.95*FuelExcess_smo + 0.05*FuelExcess;
		Lambda = 1.0f / ( FuelExcess_smo + 1.0f );

		CJ125_diag = CJ125_COM_SPI( CJ125_DIAG_REG_REQUEST ); //Update CJ125 diagnostic
		if ( UBAT >= UBAT_MIN ) CJ125_diag |= 0x0100; // set a bit indicating that the voltage read by MCU is OK
		if ( T_LSU < 850.0f)    CJ125_diag |= 0x0200; // LSU is not too hot for accuracy and durability
		if ( T_LSU > 640.0f)    CJ125_diag |= 0x0400; // LSU is not way too cold (cold is OK only during start-up)
		if ( CJ125_diag != CJ125_OK ) fault_handler( CJ125_diag );
	}
}


void Lambda_IRQHandler()  // to be called by ADC injected group EOC (end of conversion)
{
	adcValue_UB = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
	adcValue_UR = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
	adcValue_UA = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);
	UBAT = ( 3.3f * 110.0f / ( ADCrange * 10.0f ) ) * adcValue_UB ;

	// Estimation of LSU temperature using a Steinhart–Hart equation
	//const float R0=300.0f;           // Thermistor nominal resistance
	//const float T0=785.0f + 273.15f; // Temperature at nominal resistance [K]
	//const float Beta=9000.0f;        // The beta coefficient of the thermistor -- approx value from graph in datasheet
	//float R = R0 * ( 1.0f + ( adcValue_UR - adcValue_UR_Optimal ) * ( 1000.0f / ( ADCrange * 15.5f ) ) ); //[Ohm] Thermistor resistance
	//float temp = log(R / R0) / Beta + 1.0f/T0;      // ln(R/R0) / B  + 1/T0
	const float T0_inv=1.0f/(785.0f + 273.15f); //  [1/K] Inverse of temperature at nominal resistance
	const float Beta_inv = 1.0f/9000.0f;    // Inverse of beta coefficient of the thermistor -- approx value from graph in datasheet
	const float kR = 1000.0f / ( ADCrange * 15.5f ); // [-] resistance coeff
	float R_rel = 1.0f + kR * ( adcValue_UR - adcValue_UR_Optimal ); //[-] R/R0 resistance ratio
	float T_inv = logf(R_rel) * Beta_inv + T0_inv;    // [1/K]  1/T = ln(R/R0) / B  + 1/T0
	T_LSU = 1.0f / T_inv - 273.15f;                   // Invert and convert to degC

	set_heater( Heater_PID_Control() ); //Calculate and set new heater output.
}

