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
#include "VL53L1X_api.h"
#include "scanner.h"

/*********************************************************
*                        MACROS
*********************************************************/


/*********************************************************
*                    GLOBAL VARIABLES
*********************************************************/
volatile TsStepParameters motor = {
					.angle = MAX_SAMPLES_PER_ROTATION,
					.dir = STEP_CW,
					.currentStep = 0,
					.state = STEP_OFF,
					.op = STEP_NORMAL,
};

volatile TsScannerParameters scanner = {
					.state = SC_COMPLETE
};
uint8_t transmission_data[MAX_SAMPLES_PER_ROTATION][MAX_MEASUREMENTS] = {0};

/*********************************************************
*              PUBLIC FUNCTION DEFINITIONS
*********************************************************/



int main(void) {
	PLL_Init();           // Set system clock to 120 MHz
	SysTick_Init();
	scanner_Init();
	
	while(1){
		
	}
	
	return 0;
}