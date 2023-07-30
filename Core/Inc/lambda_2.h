/*
 * lambda_2.h
 *
 *  Created on: 2022-02-20
 *      Author: DrMotor
 */

#ifndef CJ125_H_
#define CJ125_H_

uint16_t Lambda_read( float* Lambda, float* F, float* Ip, float* Vcc );
void Lambda_task();
void Lambda_IRQHandler();
void Lambda_nbp_sprintf( char [] ); // prints NBP message
void Lambda_sprintf( char [] ); // prints Lambda, UBat, T_LSU, Lambda,
#endif /* CJ125 */
