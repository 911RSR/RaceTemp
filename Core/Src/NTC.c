/*
 * NTC.c
 *
 *  Created on: 20 Oct 2019
 *      Author: DrMotor
 */

#include "NTC.h"
#include <math.h>
#include "main.h" // for ADCrange

/**
 *
 * @brief  Calculate temperature [degC]
 * @param  argument: adc_value
 * @param  argument: par = NTC parameters
 * @retval Temperature
 * */
float NTC_temp(const uint16_t adc_val, const struct NTC_t par) {  //

	float R = par.Rs / (ADCrange/adc_val - 1);          // Thermistor resistance [Ohm]
	float temp = log(R / par.R0) / par.Beta + 1.0/par.T0;   // ln(R/R0) / B  + 1/T0
	return 1.0 / temp - 273.15;                     // Invert and convert to degC
}
