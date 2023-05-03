/*
 * RaceTemp.h
 *
 *  Created on: 26 Oct 2019
 *      Author: viggo
 *
 *
 *  C lang header for functions visible to main
 */

#ifndef RACETEMP_H_
#define RACETEMP_H_
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

void RaceTemp( void );  // RTOS task
void RaceTemp_ADC_isr( void );  // call this from ADC_IRQHandler() in stm32????_it.c
void RaceTemp_ignition_pulse_isr( uint32_t CCR ); // call this from TIM2_IRQHandler(void)  in stm32????_it.c
void RaceTemp_magstrip_isr( uint32_t CCR ); // call this from TIM2_IRQHandler(void)  in stm32????_it.c

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* RACETEMP_H_ */
