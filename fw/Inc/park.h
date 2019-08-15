/* =================================================================================
File name:       PARK.H (IQ version)                    
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type definitions, and 
function prototypes for the PARK.

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20                                                  
------------------------------------------------------------------------------*/
#ifndef __PARK_H__
#define __PARK_H__

typedef struct {  _iq  Alpha;  		// Input: stationary d-axis stator variable 
				  _iq  Beta;	 	// Input: stationary q-axis stator variable 
				  _iq  Sin;		// Input: rotating angle (pu) 
					_iq  Cos;		// Input: rotating angle (pu) 
				  _iq  Ds;			// Output: rotating d-axis stator variable 
				  _iq  Qs;			// Output: rotating q-axis stator variable 
		 	 	  void  (*calc)();	// Pointer to calculation function
				 } PARK;	            

typedef PARK *PARK_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/                     
#define PARK_DEFAULTS {  0, \
                          0, \
                          0, \
													0, \
                          0, \
                          0, \
              			  (void (*)(uint32_t))park_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PARK.C
------------------------------------------------------------------------------*/
void park_calc(PARK_handle);

#endif // __PARK_H__
