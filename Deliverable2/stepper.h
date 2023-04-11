/* COMPENG 2DX3 Final Project
 * Ivan Lange
 * April 12, 2023
 * main.c
 *
 *
 * Assumes system clock of 12 MHz
 */
 

/*********************************************************
*                       INCLUDES
*********************************************************/
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
 
/*********************************************************
*                       MACROS
*********************************************************/

#define NUM_SEQUENCE				(4U)
#define DELAY								(3U)
#define STEPS_IN_ROTATION		(2048U)
#define BIT(x)							(1 << x)

/*********************************************************
*                 FUNCTION PROTOTYPES
*********************************************************/
void PortH_Init(void);

/*********************************************************
*                   DATA STRUCTURES
*********************************************************/
typedef enum{
	STEP_CW,
	STEP_CCW,
}TeStepDirection;

/*
Motor Step 2.8125 deg
(2.8125/360)*2048 = 16 steps

Motor Step 5.625 deg
(5.625/360)*2048 = 32 steps

Motor Step 11.25 deg
(11.25/360)*2048 = 64 steps

Motor Step 22.5 deg
(22.5/360)*2048 = 128 steps

Motor Step 45 deg:
(45/360)*2048 = 256 steps
*/
typedef enum{
	STEP_28125 = (16U),
	STEP_56250 = (32U),
	STEP_11250 = (64U),
	STEP_22500 = (128U),
	STEP_45000 = (256U)
}TeStepAngle;

typedef enum{
	STEP_OFF,
	STEP_ON,
}TeStepState;

typedef enum{
	STEP_NORMAL,
	STEP_HOME,
}TeStepOperation;

typedef struct{
	TeStepDirection dir;
	TeStepAngle angle;
	uint16_t currentStep;
	TeStepState state;
	TeStepOperation op;
}TsStepParameters;


/*********************************************************
*                  FUNCTION DECLARATIONS
*********************************************************/
void step(TeStepDirection dir, uint16_t num_steps, TsStepParameters motor){
	uint8_t ccw_sequence[NUM_SEQUENCE] 	= {0b1100, 0b0110, 0b0011, 0b1001};		// full step lookup
	uint8_t cw_sequence[NUM_SEQUENCE] 	= {0b1001, 0b0011, 0b0110, 0b1100};
	
	
	for(uint16_t i = 0; i < num_steps; i++){		// loop through the number of steps
		GPIO_PORTH_DATA_R = (dir == STEP_CW) ? cw_sequence[i % NUM_SEQUENCE] : ccw_sequence[i % NUM_SEQUENCE];
		SysTick_Wait1ms(DELAY);
		
		if(motor.currentStep == STEPS_IN_ROTATION - motor.angle){		// if current step is 
			motor.op = STEP_HOME;
		}
		
		if(motor.dir == STEP_CW){		// increment the step count
			motor.currentStep = (motor.currentStep+1)%STEPS_IN_ROTATION;
		}
		else if(motor.dir == STEP_CCW){		// increment step count if counter clockwise
			if(motor.currentStep >= 0){
				motor.currentStep = (motor.currentStep-1);
			}
			else{
				motor.currentStep = STEPS_IN_ROTATION-1;
			}
		}
	}
	return;
}
 
