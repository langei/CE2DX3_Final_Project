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
#include "PLL.h"
#include "SysTick.h"
#include "tm4c1294ncpdt.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "stepper.h"

/*********************************************************
*                        MACROS
*********************************************************/


/*********************************************************
*                    GLOBAL VARIABLES
*********************************************************/
volatile TsStepParameters motor = {
					.angle = STEP_45000,
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

int main(void) {
	
	return 0;
}