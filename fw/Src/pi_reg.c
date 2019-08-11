/*=====================================================================================                
Ôóíêöèÿ ïîäñ÷åòà ìîäåëè ÄÏÒ ñ ÍÂ
=====================================================================================

-------------------------------------------------------------------------------------*/

#include "fxptMath.h"     
#include "pi_reg.h"


inline void pi_reg_calc(PI_REG *v)
{
// compute error[k]	
_iq err = v->ref - v->fdb;
//compute error integral [k]
v->intgrl += err;
//compute regulator output 
v->out = _IQmpy( v->Ki, v->intgrl ) + _IQmpy( v->Kp, err );
// saturate error integral
if ( v->intgrl > v->max_out ) v->intgrl = v->max_out;
if ( v->intgrl < v->min_out ) v->intgrl = v->min_out;
// saturate regulator output	
if ( v->out > v->max_out ) v->out = v->max_out;
if ( v->out < v->min_out ) v->out = v->min_out;
}


