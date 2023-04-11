/* 	Lab 6 Milestone 2
 *	Joel Tunikaitis – 400315640
 *	Ivan Lange - 400303704
 *	March 14, 2023
 */
 
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
*                   DATA STRUCTURES
*********************************************************/

typedef enum{
	STEP_CW,
	STEP_CCW,
}TeStepDirection;

/*
Motor Step 11.25 deg:
(11.25/360)*2048 = 64 steps

Motor Step 45 deg:
(45/360)*2048 = 256 steps
*/
typedef enum{
	STEP_4500 = (256U),
	STEP_1125 = (64U),
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
*                   GLOBAL VARIABLES
*********************************************************/

volatile TsStepParameters motor = {
						.angle = STEP_4500,
						.dir = STEP_CW,
						.currentStep = 0,
						.state = STEP_OFF,
						.op = STEP_NORMAL,
};


/*********************************************************
*              PUBLIC FUNCTION DEFINITIONS
*********************************************************/

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTM_DIR_R &= 0b00000000;       								      // Make PM0-PM3 inputs 
	GPIO_PORTM_DEN_R |= 0b00001111;
		
	GPIO_PORTM_PCTL_R &= ~0x0000FFFF;	 								//  Configure PJ1 as GPIO 
	GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0	
	return;
}

void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;                 // Activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTL_DIR_R = 0b00001111;       								      // Make PL0-PL3 outputs 
	GPIO_PORTL_AFSEL_R = 0;
  GPIO_PORTL_DEN_R = 0b00001111;
	GPIO_PORTL_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0	
	return;
}

void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 // Activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTH_DIR_R = 0b00001111;       								      // Make PL0-PL3 outputs 
	GPIO_PORTH_AFSEL_R = 0;
  GPIO_PORTH_DEN_R = 0b00001111;
	GPIO_PORTH_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0	
	return;
}

// Interrupt initialization for GPIO Port J IRQ# 51
void PortM_Interrupt_Init(void){
	
		GPIO_PORTM_IS_R = 0;     						// (Step 1) PM1 is Edge-sensitive 
		GPIO_PORTM_IBE_R = 0;    						//     			PM1 is not triggered by both edges 
		GPIO_PORTM_IEV_R = 0;    						//     			PM1 is falling edge event 
		GPIO_PORTM_ICR_R = 0x0F;      			// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTM_IM_R = 0x0F;      				// 					Arm interrupt on PM0-PM3 by setting proper bit in IM register
    
		NVIC_EN2_R = 0x0000100;            // (Step 2) Enable interrupt 72 in NVIC (which is in Register EN1)
	
		NVIC_PRI18_R = 0x000000A0; 					// (Step 4) Set interrupt priority to 4

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}

//	(Step 5) IRQ Handler (Interrupt Service Routine).  
//  				This must be included and match interrupt naming convention
//	 				in startup_msp432e401y_uvision.s 
//					(Note - not the same as Valvano textbook).
void GPIOM_IRQHandler(void){

	if((GPIO_PORTM_RIS_R & BIT(0))){		// Read RIS register to check if interrupt occured
		GPIO_PORTL_DATA_R ^= 0b00000001;
		SysTick_Wait10ms(10);
		motor.state ^= BIT(0);
		motor.currentStep = 0;
		GPIO_PORTM_ICR_R |= 0x01;     					// Acknowledge flag by setting proper bit in ICR register
	}
	else if((GPIO_PORTM_RIS_R & BIT(1))){
		GPIO_PORTL_DATA_R ^= 0b00000010;
		SysTick_Wait10ms(10);
		motor.dir ^= BIT(0);
		GPIO_PORTM_ICR_R |= 0x02;     					// Acknowledge flag by setting proper bit in ICR register
	}
	else if((GPIO_PORTM_RIS_R & BIT(2))){
		GPIO_PORTL_DATA_R ^= 0b00000100;	
		SysTick_Wait10ms(10);
		motor.angle = (motor.angle == STEP_1125) ? STEP_4500 : STEP_1125;
		GPIO_PORTM_ICR_R |= 0x04;     					// Acknowledge flag by setting proper bit in ICR register
	}
	else if((GPIO_PORTM_RIS_R & BIT(3))){	
		motor.op = STEP_HOME;
		GPIO_PORTM_ICR_R |= 0x08;     					// Acknowledge flag by setting proper bit in ICR register
		
	}
}

void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 // Activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTN_DIR_R = 0b00001111;       								      // Make PN0-PN3 outputs 
	GPIO_PORTN_AFSEL_R = 0;
  GPIO_PORTN_DEN_R = 0b00001111;
	GPIO_PORTN_AMSEL_R &= ~0xFF;     													// disable analog functionality on PN0	
	return;
}

void step(TeStepDirection dir, uint16_t num_steps){
	uint8_t ccw_sequence[NUM_SEQUENCE] 	= {0b1100, 0b0110, 0b0011, 0b1001};
	uint8_t cw_sequence[NUM_SEQUENCE] 	= {0b1001, 0b0011, 0b0110, 0b1100};
	
	
	for(uint16_t i = 0; i < num_steps; i++){
		GPIO_PORTH_DATA_R = (dir == STEP_CW) ? cw_sequence[i % NUM_SEQUENCE] : ccw_sequence[i % NUM_SEQUENCE];
		SysTick_Wait1ms(DELAY);
		
		if(motor.currentStep == STEPS_IN_ROTATION - motor.angle){
			motor.op = STEP_HOME;
		}
		
		if(motor.dir == STEP_CW){
			motor.currentStep = (motor.currentStep+1)%STEPS_IN_ROTATION;
		}
		else if(motor.dir == STEP_CCW){
			if(motor.currentStep >= 0){
				motor.currentStep = (motor.currentStep-1);
			}
			else{
				motor.currentStep = STEPS_IN_ROTATION-1;
			}
		}
		
		if(motor.currentStep % STEP_4500 == 0){
//			GPIO_PORTL_DATA_R ^= 0b00001000;
//			SysTick_Wait1ms(30);
//			GPIO_PORTL_DATA_R ^= 0b00001000;
			GPIO_PORTN_DATA_R ^= 0b00000010;
			SysTick_Wait1ms(30);
			GPIO_PORTN_DATA_R ^= 0b00000010;
		}
		
	}
	return;
}

//	(Step 6) The main program -- 
//		notice how we are only initializing the micro and nothing else.
// 		Our configured interrupts are being handled and tasks executed on an event driven basis.
int main(void){
  PLL_Init();           // Set system clock to 120 MHz
	SysTick_Init();
	PortL_Init();					// Initialize the onboard LED on port N
	PortH_Init();
	PortM_Init();							// Initialize the onboard push button on PJ1
	PortM_Interrupt_Init();		// Initalize and configure the Interrupt on Port J
	PortN_Init();
	GPIO_PORTL_DATA_R |= 0b00000110;
	
	while(1){							// Inside an infinite while loop, 
			if(motor.state){
				step(motor.dir, motor.angle);
			}

			if(motor.op == STEP_HOME){
				motor.state = STEP_OFF;
				motor.op = STEP_NORMAL;
		}
//		if(motor.op == STEP_HOME){
//			step(motor.dir, ((motor.dir == STEP_CW) ? (STEPS_IN_ROTATION - motor.currentStep) : motor.currentStep));  
//			motor.state = STEP_OFF;
//			motor.op = STEP_NORMAL;
//		}
	}
}