/*
 * NTC.h
 *
 *  Created on: 20 Oct 2019
 *      Author: DrMotor
 */

#ifndef NTC_H_
#define NTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct NTC_t{
	float R0;        // [Ohm] Thermistor resistance at the reference temperature
	float T0;        // [K] Temperature for nominal resistance (often 25.0 + 273.15 )
	float Beta;      // [-] Beta coefficient of the thermistor (usually in the 3000 to 4000 range)
	float Rs;        // [Ohm] Pull-up resistor: 40000.0 Ohm is approximate value of the STM32's internal pull-up -- enable in MX if needed
};


// Go karts (Rotax Max, KZ and others) often use a water temperature sensor with M10x1 (or 1/8" 27NPT) threads.
// Also scooters (Piaggio, Aprilia) and old Volvo cars use sensors with same threads.
//
// Volvo sensor:
// https://www.aliexpress.com/item/Brand-New-Coolant-Temperature-Sensor-Sender-For-VOLVO-240-740-760-940-2-4-TD-Turbo/32670042910.html
// Approx. 2000 Ohm at 25 degC and 200 Ohm at 100 degC -- R0=2000.0; T0=20+273.15; Beta=3150.0;
// I use 1 kOhm pull-up with this sensor.
const static struct NTC_t NTC_Volvo = { 2000.0, 20.0 + 273.15, 3150.0, 1000.0 };

// Koso's MC sensor: https://www.aliexpress.com/item/1005001460278209.html
// can also be used (even if the display is not needed for our use).
// I use 10 kOhm pull up with this sensor.
const static struct NTC_t NTC_KOSO = { 50000.0, 25.0 + 273.15, 4000.0, 10000.0 };

float NTC_temp( const uint16_t adc_val, const struct NTC_t ntc );

#ifdef __cplusplus
}
#endif

#endif /* NTC_H_ */
