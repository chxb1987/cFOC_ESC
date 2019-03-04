#ifndef __CORDICTRIG_H__
#define __CORDICTRIG_H__

#include "main.h"

/***********************************************************************
* CORDIC, 16 bits, 14 iterations
* MUL = 1.0 = 8192.000000 multiplication factor
* A = 1.743165 convergence angle (limit is 1.7432866 = 99.9deg)
* F = 1.646760 gain (limit is 1.64676025812107)
* 1/F = 0.607253 inverse gain (limit is 0.607252935008881)
* pi = 3.141593 (3.1415926536897932384626)
**********************************************************************/
#define CORDIC_A 1.743165f 				// CORDIC convergence angle A
#define CORDIC_F 13490 						// CORDIC gain F
#define CORDIC_1F 4975 						// CORDIC inverse gain 1/F
#define CORDIC_HALFPI 12868				// pi / 2
#define CORDIC_PI 25736           // pi
#define CORDIC_TWOPI 51472        // pi * 2
#define CORDIC_MUL 8192.00000f 		// CORDIC multiplication factor M = 2^13
#define CORDIC_MAXITER 14

_iq CORDIC_ZTBL[] = { 6434, 3798, 2007, 1019, 511, 256, 128, 64, 32, 16, 8, 4, 2, 1 };

void sincos(_iq a, _iq m, _iq *s, _iq *c);
void atan2sqrt(_iq *a, _iq *m, _iq y, _iq x);

/*******************************************************************************
* Function Name : CORDICsincos
* Description :   Функция вычисления значения синуса и косинуса угла.
*     Аргументы и возвращаемое значение - 32-х битные целые числа со знаком.
*     Тип данных аргумента и возвращаемого значения - GLOBAL_Q.
*			Вход: угол 'a' (рад), амплитуда 'm'
*     Выход: синус 's', косинус 'c'
*     Диапазон значений для аргументов: весь диапазон типа данных.
*     Время исполнения ...
*******************************************************************************/

static inline void CORDICsincos(_iq a, _iq m, _iq *s, _iq *c) {
 _iq k, tx, x=m, y=0, z=a, fl=0;
 if (z>+CORDIC_HALFPI) { fl=+1; z = (+CORDIC_PI) - z; }
 else if (z<-CORDIC_HALFPI) { fl=+1; z = (-CORDIC_PI) - z; }
 for (k=0; k<CORDIC_MAXITER; k++) {
 tx = x;
 if (z>=0) { x -= (y>>k); y += (tx>>k); z -= CORDIC_ZTBL[k]; }
 else { x += (y>>k); y -= (tx>>k); z += CORDIC_ZTBL[k]; } }
 if (fl) x=-x;
 *c = x; // m*cos(a) multiplied by gain F and factor M
 *s = y; // m*sin(a) multiplied by gain F and factor M
}

/*******************************************************************************
* Function Name : CORDICatan2sqrt
* Description :   Функция вычисления арктангенса.
*     Аргументы и возвращаемое значение - 32-х битные целые числа со знаком.
*     Тип данных аргумента и возвращаемого значения - GLOBAL_Q.
*			Вход: синус 's', косинус 'c'
*     Выход: угол 'a' (рад), амплитуда 'm'
*     Время исполнения ...
*******************************************************************************/
static inline void CORDICatan2sqrt(_iq *a, _iq *m, _iq y, _iq x) {
 _iq k, tx, z=0, fl=0;
 if (x<0) { fl=((y>0)?+1:-1); x=-x; y=-y; }
 for (k=0; k<CORDIC_MAXITER; k++) {
 tx = x;
 if (y<=0) { x -= (y>>k); y += (tx>>k); z -= CORDIC_ZTBL[k]; }
 else { x += (y>>k); y -= (tx>>k); z += CORDIC_ZTBL[k]; } }
 if (fl!=0) { z += fl*CORDIC_PI; }
 *a = z; // radians multiplied by factor M
 *m = x; // sqrt(x^2+y^2) multiplied by gain F
}


#endif
