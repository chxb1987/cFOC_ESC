/*=====================================================================================
 File name:        PARK.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Alphascription:  Park Transformation                   

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/

#include "fxptMath.h"         // Include header for IQmath library
// Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file
#include "park.h"

void park_calc(PARK *v)
{	
  
//    _iq Cosine,Sine;

// Using look-up IQ sine table
//     Sine = _IQsin(v->Angle);
//     Cosine = _IQcos(v->Angle);

     v->Ds = _IQmpy(v->Alpha,v->Cos) + _IQmpy(v->Beta,v->Sin);
     v->Qs = _IQmpy(v->Beta,v->Cos) - _IQmpy(v->Alpha,v->Sin);

}


