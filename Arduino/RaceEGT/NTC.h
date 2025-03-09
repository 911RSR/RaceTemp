/*
 * NTC.h
 *
 *  Created on: 20 Oct 2019
 *      Author: viggo
 */

#ifndef NTC_H_
#define NTC_H_

#include <stdint.h>

class NTC {
public:
	float degC(const uint16_t adc_val);
	// Default values for Volvo 240 water temp sensor (also fits Rotax Max)
	// https://www.aliexpress.com/item/Brand-New-Coolant-Temperature-Sensor-Sender-For-VOLVO-240-740-760-940-2-4-TD-Turbo/32670042910.html
	// This sensor has approx 2000 Ohm at 20 degC and 200 Ohm at 100 degC -- R0=2000.0; T0=20+273.15; Beta=3150.0;
	// The Volvo sensor needs approx. 1000 Ohm pull-up, so I use external resistor
	const float ADCrange=4095.0;  // ADC range -- depends on setup (left or right adjusted) and ADC bits
	float R0=2000.0;        // Thermistor resistance at 20 degC
	float T0=20.0 + 273.15; // temp. for nominal resistance [K] (almost always 25 C +273.15)
	float Beta=3150.0;      // The beta coefficient of the thermistor (usually 3000-4000)
	float Rs=1000.0;        // Pull-up resistor: 40000 is value of the chip internal pullup for Nano -- enable in setup() if needed
};

#endif /* NTC_H_ */
