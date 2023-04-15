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
void PortM_Init(void);

/*********************************************************
*                   DATA STRUCTURES
*********************************************************/
typedef enum{
	STEP_CW,
	STEP_CCW,
}TeStepDirection;

///*
//Motor Step 2.8125 deg: 16 steps
//Motor Step 5.625 deg: 32 steps
//Motor Step 11.25 deg: 64 steps
//Motor Step 22.5 deg: 128 steps
//Motor Step 45 deg: 256 steps
//*/
//typedef enum{
//	STEP_28125 = (16U),
//	STEP_56250 = (32U),
//	STEP_11250 = (64U),
//	STEP_22500 = (128U),
//	STEP_45000 = (256U)
//}TeStepAngle;

//typedef enum{
//	STEP_OFF,
//	STEP_ON,
//}TeStepState;

typedef enum{
	STEP_NORMAL,
	STEP_HOME,
}TeStepOperation;

typedef struct{
	TeStepDirection dir;
	uint16_t angle;
	uint16_t currentStep;
//	TeStepState state;
	TeStepOperation op;
}TsStepParameters;

volatile TsStepParameters motor;
/*********************************************************
*                  FUNCTION DECLARATIONS
*********************************************************/
//Initialize GPIO Port H (Stepper Motor)
void PortH_Init(void){
	//Use PortH pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xF;        								// make PH0 out 
  GPIO_PORTH_AFSEL_R &= ~0xF;     								// disable alt funct on PN0
  GPIO_PORTH_DEN_R |= 0xF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  GPIO_PORTH_AMSEL_R &= ~0xF;     								// disable analog functionality on PH0		
	return;
}

void step(TeStepDirection dir, uint16_t num_steps){
	uint8_t ccw_sequence[NUM_SEQUENCE] 	= {0b1100, 0b0110, 0b0011, 0b1001};		// full step lookup
	uint8_t cw_sequence[NUM_SEQUENCE] 	= {0b1001, 0b0011, 0b0110, 0b1100};
	
	
	for(uint16_t i = 0; i < num_steps; i++){		// loop through the number of steps
		GPIO_PORTH_DATA_R &= ~0xF;
		GPIO_PORTH_DATA_R |= (dir == STEP_CW) ? cw_sequence[i % NUM_SEQUENCE] : ccw_sequence[i % NUM_SEQUENCE];
		SysTick_Wait1ms(DELAY);
	}
}
 
