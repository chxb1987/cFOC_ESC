/*=====================================================================================
 File name:        IPARK.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Dsscription:  Inverse Park Transformation                   

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/

#include "fxptMath.h"         // Include header for IQmath library
// Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file
#include "ipark.h"

void ipark_calc(IPARK *v)
{	
   
   //_iq Cosine,Sine;
   
// Using look-up IQ sine table
//     Sine = _IQsin(v->Angle);
//     Cosine = _IQcos(v->Angle);
 
     v->Alpha = _IQmpy(v->Ds,v->Cos) - _IQmpy(v->Qs,v->Sin);
     v->Beta = _IQmpy(v->Qs,v->Cos) + _IQmpy(v->Ds,v->Sin);  
}


