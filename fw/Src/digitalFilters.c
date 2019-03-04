#include "fxptMath.h"
#include "digitalFilters.h"

//inline void Filter_Init(FILTER_DATA *filter)
//{
//	//filter->Tf = _IQdiv(filter->Tf, filter->Ts); 
//	filter->ky1=_IQdiv(filter->Tf,( filter->Tf + _IQ(1.0) ) );
//	filter->kx0=_IQdiv( _IQ(1.0), ( filter->Tf + _IQ(1.0) ) );
//}

//inline void Filter_Execute(FILTER_DATA *filter)
//{
//	filter->y = _IQmpy( filter->kx0, filter->x) +
//	_IQmpy(filter->ky1, filter->y);
//}
