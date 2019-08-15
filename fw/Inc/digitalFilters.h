#ifndef __DIGITALFILTERS_H__
#define __DIGITALFILTERS_H__

typedef struct {
	_iq x;
	_iq y;
	_iq kx0;
	_iq ky1;
	_iq Tf;					// ( 1 / cutoff_frequency )
  _iq Ts;					// Sampling time
} FILTER_DATA;

#define FILTER_DEFAULTS { 0, 0, 0, 0, \
													_IQ(10),_IQ(10) }

												
__forceinline void Filter_Init(FILTER_DATA *filter)
{
	filter->Tf = _IQdiv(filter->Tf, filter->Ts); 
	filter->ky1=_IQdiv(filter->Tf,( filter->Tf + _IQ(1.0) ) );
	filter->kx0=_IQdiv( _IQ(1.0), ( filter->Tf + _IQ(1.0) ) );
}

__forceinline void Filter_Execute(FILTER_DATA *filter)
{
	filter->y = _IQmpy( filter->kx0, filter->x) +
	_IQmpy(filter->ky1, filter->y);
}
												
#endif

/***************************** END OF FILE ****/
