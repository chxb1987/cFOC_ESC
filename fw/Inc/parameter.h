/* ==============================================================================
System Name:  ACI33_SIM

File Name:	PARAMETER.H

Description:	Parameters file for the Simulation of Sensored Indirect 
          		Field Orientation Control for a Three Phase AC Induction Motor. 

Originator:		Digital control systems Group - Texas Instruments

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 05-15-2002	Release	Rev 1.0
=================================================================================  */

#ifndef PARAMETER_H
#define PARAMETER_H

/*-------------------------------------------------------------------------------
Next, definitions used in main file.
-------------------------------------------------------------------------------*/
#define PI 3.14159265358979

/* Define the samping period for simulation (sec) (5kHz)*/
#define SAMPLING_PERIOD T_S

/* Define the Induction motor parameters */
#define R_EX 	220               	/* Stator resistance (ohm) */
#define L_EX   	2               	/* Rotor resistance (ohm) */
#define R_A   	1     			/* Stator inductance (H) */
#define L_A   	0.0020				/* Rotor inductance (H) */	
#define LM   	0.159232				/* Rotor inductance (H) */
#define P    	4						/* Number of poles */	

/* Define the mechanical parameters */
#define BB	0.0001     			/* Damping coefficient (N.m.sec/rad) */
#define J_NOM	0.1						/* Moment of inertia of rotor mass (kg.m^2) */		
#define TL  0   						/* Load torque (N.m) */

/* Define the base quantites */
#define BASE_VOLTAGE_EX    220       /* Base peak phase voltage (volt) */
#define BASE_CURRENT_EX    1         /* Base peak phase current (amp) */

#define BASE_VOLTAGE_A    220       /* Base peak phase voltage (volt) */
#define BASE_CURRENT_A    10             /* Base peak phase current (amp) */
#define K_FI_NOM      	1            /* Base electrical frequency (Hz) */

#endif  // end of PARAMETER.H definition

//===========================================================================
// No more.
//===========================================================================
