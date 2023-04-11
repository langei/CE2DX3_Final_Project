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
#include "stepper.h"
#include "VL53L1X_api.h"
#include "i2c.h"
#include "onboardLEDs.h"
#include "uart.h"

/*********************************************************
*                    		 MACROS
*********************************************************/
//SET MAX_SAMPLES_PER_ROTATION to one of the following:
//Motor Step 2.8125 deg: 16 steps
//Motor Step 5.625 deg: 32 steps
//Motor Step 11.25 deg: 64 steps
//Motor Step 22.5 deg: 128 steps
//Motor Step 45 deg: 256 steps

#define MAX_SAMPLES_PER_ROTATION		(256U)
#define MAX_MEASUREMENTS 						(20U)


/*********************************************************
*                 FUNCTION PROTOTYPES
*********************************************************/
void scanner_Init();
void EnableInt(void);
void DisableInt(void);
void WaitForInt(void);
void PortJ_Init(void);
void PortJ_Interrupt_Init(void);

/*********************************************************
*                   DATA STRUCTURES
*********************************************************/
typedef enum{
	SC_SCANNING,
	SC_COMPLETE
}TeScannerState;

typedef struct{
	TeScannerState state;
}TsScannerParameters;

/*********************************************************
*                  GLOBAL INSTANCES
*********************************************************/
extern volatile TsStepParameters motor;
extern volatile TsScannerParameters scanner;
extern uint8_t transmission_data[MAX_SAMPLES_PER_ROTATION][MAX_MEASUREMENTS];

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

// Give clock to Port J and initalize as input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
		
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ0 and PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000FF;	 								// Configure PJ0 and PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											// Disable analog functionality on PJ0 and PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//Enable weak pull up resistor
}

// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
	
		GPIO_PORTJ_IS_R = 0;     						// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;    						//     			PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0;    						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x03;      			// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x03;      				// 					Arm interrupt on PJ0 and PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;            // (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000; 					// (Step 4) Set interrupt priority to 5

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}

// IRQ Handler (Interrupt Service Routine).  
void GPIOJ_IRQHandler(void){

	FlashLED2(1);													// Flash the LED D2 one time
	GPIO_PORTJ_ICR_R = 0x03;     					// Acknowledge flag by setting proper bit in ICR register
}


void scanner_Init(){
	PortH_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	onboardLEDs_Init();
	SetLED2();
}
