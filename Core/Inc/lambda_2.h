/*
 * lambda_2.h
 *
 *  Created on: 2022-02-20
 *      Author: viggo
 */

#ifndef CJ125_H_
#define CJ125_H_

uint16_t Lambda_read( float* Lambda, float* F, float* Ip, float* Vcc );
void Lambda_task();
void Lambda_IRQHandler();
void Lambda_nbp_sprintf( char [] );
void Lambda_sprintf_FE_Batt_TLSU( char [] );

#endif /* CJ125 */
