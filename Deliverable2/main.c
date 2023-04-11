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
					.angle = STEP_SIZE	,
					.dir = STEP_CW,
					.currentStep = 0,
					.op = STEP_NORMAL,
};

volatile TsScannerParameters scanner = {
					.status = SC_COMPLETE,
					.state = SC_SCAN_MODE
};

volatile uint16_t transmissionData[MAX_MEASUREMENTS][NUM_SAMPLES] = {0};
/*********************************************************
*              PUBLIC FUNCTION DEFINITIONS
*********************************************************/



int main(void) {
	PLL_Init();           // Set system clock to 12 MHz
	SysTick_Init();
	scanner_Init();
	
	for(int i = 0; i < 6; i++){
		FlipLED2();
		SysTick_Wait10ms(50);
	}

	while(1){
		if(scanner.state == SC_SCAN_MODE && scanner.status == SC_SCANNING){
			scanYZ();
		}
		if(scanner.state == SC_TRANSMIT_MODE){
			transmitDistanceSerial();
		}
	}
	
	return 0;
}