#include "main.h"
#include "stdint.h"

#define _HI(x) (uint16_t) ( (x) >> 8 )      // запись в старший байт
#define _LO(x) (uint16_t) ( (x) & 0xFF )            // запись в младший байт
#define _HI16(x) (uint16_t) ( (x) >> 16 )      // запись в старший байт
#define _LO16(x) (uint16_t) ( (x) & 0xFFFF )            // запись в младший байт

typedef volatile int32_t _iq;

#define GLOBAL_Q 24
#define HALF_GLOBAL_Q (GLOBAL_Q / 2)

#define M_PI            3.141592653589793f
#define M_2PI           (M_PI * 2.0f)
#define M_PI2           (M_PI / 2.0f)
#define M_SQRT3         1.7320508075688772f

#define   _IQ30(A)      (int32_t) ((A) * 1073741824.0L)
#define   _IQ29(A)      (int32_t) ((A) * 536870912.0L)
#define   _IQ28(A)      (int32_t) ((A) * 268435456.0L)
#define   _IQ27(A)      (int32_t) ((A) * 134217728.0L)
#define   _IQ26(A)      (int32_t) ((A) * 67108864.0L)
#define   _IQ25(A)      (int32_t) ((A) * 33554432.0L)
#define   _IQ24(A)      (int32_t) ((A) * 16777216.0L)
#define   _IQ23(A)      (int32_t) ((A) * 8388608.0L)
#define   _IQ22(A)      (int32_t) ((A) * 4194304.0L)
#define   _IQ21(A)      (int32_t) ((A) * 2097152.0L)
#define   _IQ20(A)      (int32_t) ((A) * 1048576.0L)
#define   _IQ19(A)      (int32_t) ((A) * 524288.0L)
#define   _IQ18(A)      (int32_t) ((A) * 262144.0L)
#define   _IQ17(A)      (int32_t) ((A) * 131072.0L)
#define   _IQ16(A)      (int32_t) ((A) * 65536.0L)
#define   _IQ15(A)      (int32_t) ((A) * 32768.0L)
#define   _IQ14(A)      (int32_t) ((A) * 16384.0L)
#define   _IQ13(A)      (int32_t) ((A) * 8192.0L)
#define   _IQ12(A)      (int32_t) ((A) * 4096.0L)
#define   _IQ11(A)      (int32_t) ((A) * 2048.0L)
#define   _IQ10(A)      (int32_t) ((A) * 1024.0L)
#define   _IQ9(A)       (int32_t) ((A) * 512.0L)
#define   _IQ8(A)       (int32_t) ((A) * 256.0L)
#define   _IQ7(A)       (int32_t) ((A) * 128.0L)
#define   _IQ6(A)       (int32_t) ((A) * 64.0L)
#define   _IQ5(A)       (int32_t) ((A) * 32.0L)
#define   _IQ4(A)       (int32_t) ((A) * 16.0L)
#define   _IQ3(A)       (int32_t) ((A) * 8.0L)
#define   _IQ2(A)       (int32_t) ((A) * 4.0L)
#define   _IQ1(A)       (int32_t) ((A) * 2.0L)

#if GLOBAL_Q == 24
#define _IQ(A)          _IQ24(A)
#elif GLOBAL_Q == 20
#define _IQ(A)          _IQ20(A)
#elif GLOBAL_Q == 15
#define _IQ(A)          _IQ15(A)
#else
#error "Wrong IQ type."
#endif 

#define _IQ8toIQ(A) ((int32_t)  (A) << (GLOBAL_Q - 8))
#define _IQ9toIQ(A) ((int32_t)  (A) << (GLOBAL_Q - 9))
#define _IQ10toIQ(A) ((int32_t) (A) << (GLOBAL_Q - 10))
#define _IQ11toIQ(A) ((int32_t) (A) << (GLOBAL_Q - 11))
#define _IQ12toIQ(A) ((int32_t) (A) << (GLOBAL_Q - 12))
#define _IQ13toIQ(A) ((int32_t) (A) << (GLOBAL_Q - 13))
#define _IQ14toIQ(A) ((int32_t) (A) << (GLOBAL_Q - 14))
#define _IQ15toIQ(A) ((int32_t) (A) << (GLOBAL_Q - 15))
#define _IQ16toIQ(A) ((int32_t) (A) << (GLOBAL_Q - 16))

#define _IQtoIQ8(A) ((int32_t)  (A) >> (GLOBAL_Q - 8))
#define _IQtoIQ9(A) ((int32_t)  (A) >> (GLOBAL_Q - 9))
#define _IQtoIQ10(A) ((int32_t) (A) >> (GLOBAL_Q - 10))
#define _IQtoIQ11(A) ((int32_t) (A) >> (GLOBAL_Q - 11))
#define _IQtoIQ12(A) ((int32_t) (A) >> (GLOBAL_Q - 12))
#define _IQtoIQ13(A) ((int32_t) (A) >> (GLOBAL_Q - 13))
#define _IQtoIQ14(A) ((int32_t) (A) >> (GLOBAL_Q - 14))
#define _IQtoIQ15(A) ((int32_t) (A) >> (GLOBAL_Q - 15))
#define _IQtoIQ16(A) ((int32_t) (A) >> (GLOBAL_Q - 16))

#define _IQdiv2(A)          ((A) >> 1)
#define _IQdiv4(A)          ((A) >> 2)
#define _IQdiv8(A)          ((A) >> 3)
#define _IQdiv16(A)         ((A) >> 4)
#define _IQdiv32(A)         ((A) >> 5)
#define _IQdiv64(A)         ((A) >> 6)
#define _IQdiv128(A)        ((A) >> 7)
#define _IQdiv256(A)        ((A) >> 8)

#define _IQmpy(A, B)        (int32_t) ( ( (int64_t) (A) * (int64_t) (B) ) >> GLOBAL_Q )
#define _IQmpylp(A, B)      (int32_t) ( ( (A) >> HALF_GLOBAL_Q ) * ( (B) >> HALF_GLOBAL_Q ) )

#define _IQdiv(A, B)        (int32_t) ( ( (int64_t) (A) << GLOBAL_Q ) / (B) )
#define _IQdivlp(A, B)      (int32_t) ( ( (A) << HALF_GLOBAL_Q ) / ( (B) << HALF_GLOBAL_Q ) )

/* Description :  Макрос преобразования биполярного значения
*                к униполярному с масштабированием
* Input : x    - число в формате GLOBAL_Q: 8000..FFFF-0000..7FFF
* Param : BASE - разрядность преобразуемого числа: 15 если _IQ15 
* Return :       0x0000 .. (base - 1)
*/
#define _IQb2u(x, BASE) ( (uint32_t) ( (int32_t) (x) + ( (1 << BASE) - 1 ) ) )

/* Description :  Макрос ограничения числа врхним (u) и нижним (l) пределом
* Input : x    - число в формате GLOBAL_Q
*/
#define _IQsat(x, u, l) ( (x > u) ? x = u : (x < l) ? x = l )


/*******************************************************************************
* Macros Name :   _IQsin
* Description :   Возвращает полиномиальное приближение функции sin (-pi/2 : pi/2)
*     Тип данных аргумента и возвращаемого значения - GLOBAL_Q.
*     PHI_EXPR: Угол  
*     Второй аргумент - выходная переменная - GLOBAL_Q.
*     Точность результата зависит от количества членов ряда:
*     1) x * (0.98557 - x * x * 0.142595)
*                           абсолютная погрешность менее 0.0045
*     2) x * (0.9996951 - x * x * (0.16567 - x * x * 0.0075132))
*                           абсолютная погрешность менее 68e-6
*******************************************************************************/
_iq _IQsin(_iq VAR);

inline _iq _IQsin(_iq VAR){
    VAR = VAR > _IQ(M_PI2) ? -VAR + _IQ(M_PI)
        : VAR < -_IQ(M_PI2) ? -VAR - _IQ(M_PI) : VAR;
    VAR = _IQmpy(VAR, (_IQ(0.98557) - _IQmpy(VAR, _IQmpy(VAR, _IQ(0.142595)))));
	return VAR;
}

_iq _IQsinPU(_iq VAR);

inline _iq _IQsinPU(_iq VAR){
		VAR *= _IQ(M_PI);
    VAR = VAR > _IQ(M_PI2) ? -VAR + _IQ(M_PI)
        : VAR < -_IQ(M_PI2) ? -VAR - _IQ(M_PI) : VAR;
    VAR = _IQmpy(VAR, (_IQ(0.98557) - _IQmpy(VAR, _IQmpy(VAR, _IQ(0.142595)))));
	return VAR;
}

/*******************************************************************************
* Macros Name :   _IQcos
* Description :   Возвращает полиномиальное приближение функции cos
*     Тип данных аргумента и возвращаемого значения - GLOBAL_Q.
*     Второй аргумент - вспомогательная переменная - GLOBAL_Q.
*     Точность результата зависит от количества членов ряда:
*     1) x * (0.98557 - x * x * 0.142595)
*                           абсолютная погрешность менее 0.0045
*     2) x * (0.9996951 - x * x * (0.16567 - x * x * 0.0075132))
*                           абсолютная погрешность менее 68e-6
*******************************************************************************/
//#define _IQcos(PHI_EXPR, VAR) (                                                 \
//    VAR = (PHI_EXPR),                                                           \
//    VAR = (VAR > 0 ? -VAR : VAR) + _IQ(M_PI / 2),                               \
//    _IQmpy(VAR, (_IQ(0.98557) - _IQmpy(VAR, _IQmpy(VAR, _IQ(0.142595)))))       \
//)

_iq _IQcos(_iq VAR);

static inline _iq _IQcos(_iq VAR){
    VAR = (VAR > 0 ? -VAR : VAR) + _IQ(M_PI2);
    VAR = _IQmpy(VAR, (_IQ(0.98557) - _IQmpy(VAR, _IQmpy(VAR, _IQ(0.142595)))));
	return VAR;
}

/*******************************************************************************
* Function Name : _IQdiv14BIT
* Description :   Функция деления. Делит первый аргумент на второй.
*     Аргументы - 32-х битные целые числа со знаком.
*     Тип данных аргументов и возвращаемого значения - GLOBAL_Q.
*     Точность результата - 14 бит (30 / 2).
*     При одновременном уменьшении значений аргументов - не падает.
*     Время исполнения от значений аргументов не зависит - 1.1 uS.
*******************************************************************************/

_iq _IQdiv14BIT(_iq A, _iq B);

static inline _iq _IQdiv14BIT(_iq A, _iq B)
{
    _iq x = A > 0 ? A : -A;
    _iq y = B > 0 ? B : -B;
    y = x > y ? x : y;
    unsigned char i, j;
    if (y & 0xFFFF8000U) {
        if (y & 0xFF800000U) {
            if (y & 0xF8000000U) {
                i = y & 0xE0000000U ? 0 : 2;
            } else {
                i = y & 0xFE000000U ? 4 : 6;
            }
        } else {
            if (y & 0xFFF80000U) {
                i = y & 0xFFE00000U ? 8 : 10;
            } else {
                i = y & 0xFFFE0000U ? 12 : 14;
            }
        }
    } else {
        if (y & 0xFFFFFF80U) {
            if (y & 0xFFFFF800U) {
                i = y & 0xFFFFE000U ? 30 - 14 : 30 - 12;
            } else {
                i = y & 0xFFFFFE00U ? 30 - 10 : 30 - 8;
            }
        } else {
            if (y & 0xFFFFFFF8U) {
                i = y & 0xFFFFFFE0U ? 30 - 6 : 30 - 4;
            } else {
                i = y & 0xFFFFFFFEU ? 30 - 2 : 30 - 0;
            }
        }
    }
    j = i >> 1;
    j = i <= GLOBAL_Q ? (GLOBAL_Q >> 1) - j : j - (GLOBAL_Q >> 1);
    return
    i <= GLOBAL_Q
        ? ((long)((long)(A << i) / (B >> j)) << j)
        : ((long)((long)(A << i) / (B << j)) >> j);
}

/*******************************************************************************
* Function Name : _IQatan
* Description :   Базовая функция вычисления значения арктангенса полиномом.
*     Аргумент и возвращаемое значение - 32-х битные целые числа со знаком.
*     Тип данных аргумента и возвращаемого значения - GL_Q.
*     Диапазон значений для аргумента: от _IQ(0.0) до _IQ(1.0)
*     Абсолютная погрешность вычисления результата - 0.0035 рад.
*     Время исполнения от значений аргументов не зависит - #.# uS.
*******************************************************************************/

_iq _IQatan(_iq A);

static inline _iq _IQatan(_iq A) {
    return _IQmpy(A, (_IQ(1.054) - _IQmpy(_IQ(0.265), A))); // 0.42 uS.
}

/*******************************************************************************
* Function Name : _IQatan2
* Description :   Функция вычисления значения арктангенса _IQatan2(Y / X).
*     Аргументы и возвращаемое значение - 32-х битные целые числа со знаком.
*     Тип данных аргумента и возвращаемого значения - GLOBAL_Q.
*     Диапазон значений для аргументов: весь диапазон типа данных.
*     Абсолютная погрешность вычисления результата - 0.0035 рад.
*     Время исполнения от значений аргументов не зависит - 2.7 .. 2.9 uS.
*******************************************************************************/

_iq _IQatan2(_iq A, _iq B);

static inline _iq _IQatan2(_iq A, _iq B) {
    if (A == 0) { return B >= 0 ?          0 : -_IQ(M_PI ); }
    if (B == 0) { return A >  0 ? _IQ(M_PI2) : -_IQ(M_PI2); }
    if (B >  0) {
        if (A > 0) {
            return B > A ? +_IQatan(_IQdiv14BIT(A, B))
                         : -_IQatan(_IQdiv14BIT(B, A)) + _IQ(M_PI2);
        } else {
            A = -A;
            return B > A ? -_IQatan(_IQdiv14BIT(A, B))
                         : +_IQatan(_IQdiv14BIT(B, A)) - _IQ(M_PI2);
        }
    } else {
        B = -B;
        if (A > 0) {
            return B > A ? -_IQatan(_IQdiv14BIT(A, B)) + _IQ(M_PI)
                         : +_IQatan(_IQdiv14BIT(B, A)) + _IQ(M_PI2);
        } else {
            A = -A;
            return B > A ? +_IQatan(_IQdiv14BIT(A, B)) - _IQ(M_PI)
                         : -_IQatan(_IQdiv14BIT(B, A)) - _IQ(M_PI2);
        }
    }
}

/*******************************************************************************
* Function Name : _IQsqrt
* Description :   Функция вычисления значения квадратного корня.
*     Аргументы и возвращаемое значение - 32-х битные целые числа со знаком.
*     Тип данных аргумента и возвращаемого значения - GLOBAL_Q.
*     Диапазон значений для аргументов: весь диапазон типа данных.
*     Время исполнения ...
*******************************************************************************/

_iq _IQsqrt( _iq v );

static inline _iq _IQsqrt( _iq v ) {
	_iq temp, nHat = 0, b = 32768, bshft = 15;
	do {
		if (v >= (temp = (((nHat << 1) + b) << bshft--)))
			{
				nHat += b;
				v -= temp;
			}
		} while (b >>= 1);
	return nHat;
}

/*******************************************************************************
* Function Name : _IQmag
* Description :   Функция вычисления значения суммы квадратов под корнем.
*     Аргументы и возвращаемое значение - 32-х битные целые числа со знаком.
*     Тип данных аргумента и возвращаемого значения - GLOBAL_Q.
*     Диапазон значений для аргументов: весь диапазон типа данных.
*     Время исполнения ...
*******************************************************************************/

_iq _IQmag( _iq x, _iq y );

static inline _iq _IQmag( _iq x, _iq y ){
	return _IQsqrt( _IQmpy(x, x) + _IQmpy(y, y) );
}
	

/*******************************************************************************
* Function Name : _IQisqrt
* Description :   Функция вычисления значения обратного корня.
*     Аргументы и возвращаемое значение - 32-х битные целые числа со знаком.
*     Тип данных аргумента и возвращаемого значения - GLOBAL_Q.
*     Диапазон значений для аргументов: весь диапазон типа данных.
*     Время исполнения ...
*******************************************************************************/

_iq _IQisqrt(_iq x);

static inline _iq _IQisqrt(_iq x) {
	_iq halfx = _IQmpy( _IQ(0.5), x);
	_iq y = x;
	_iq i = *(_iq*)&y;
	i = 1597463007 - (i>>1);
	y = *(_iq*)&i;
	y = _IQmpy(y, ( _IQ(1.5) - _IQmpy( halfx, _IQmpy(y, y) ) ) );
	return y;
}
