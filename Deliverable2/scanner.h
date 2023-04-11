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
//SET SIZE_SIZE to one of the following:
//Motor Step 2.8125 deg: 16 steps
//Motor Step 5.625 deg: 32 steps
//Motor Step 11.25 deg: 64 steps
//Motor Step 22.5 deg: 128 steps
//Motor Step 45 deg: 256 steps

#define STEP_SIZE										(256U)


#define NUM_SAMPLES									(STEPS_IN_ROTATION/STEP_SIZE)
#define MAX_MEASUREMENTS 						(20U)
#define DEV													(uint16_t)(0x29)
#define MASK												(uint16_t)(0xFF)


/*********************************************************
*                 FUNCTION PROTOTYPES
*********************************************************/
void scanner_Init();
void EnableInt(void);
void DisableInt(void);
void WaitForInt(void);
void PortJ_Init(void);
void PortJ_Interrupt_Init(void);
void bootVL53L1X(void);
void scanYZ(void);

/*********************************************************
*                   DATA STRUCTURES
*********************************************************/
typedef enum{
	SC_SCANNING,
	SC_COMPLETE
}TeScanningStatus;

typedef enum{
	SC_SCAN_MODE,
	SC_TRANSMIT_MODE
}TeScanningState;

typedef struct{
	TeScanningStatus status;
	TeScanningState state;
}TsScannerParameters;

/*********************************************************
*                  GLOBAL INSTANCES
*********************************************************/
extern volatile TsStepParameters motor;
extern volatile TsScannerParameters scanner;
extern volatile uint16_t transmissionData[MAX_MEASUREMENTS][NUM_SAMPLES];


uint8_t dataReady = 0;
uint8_t sensorState = 0;
uint8_t measurementNum = 0;
uint16_t distance = 0;
uint16_t wordData = 0;
int status = 0;
	

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

void bootVL53L1X(void){
	
	while(sensorState==0){
		status = VL53L1X_BootState(DEV, &sensorState);
		SysTick_Wait10ms(10);
  }
	
	status = VL53L1X_ClearInterrupt(DEV);  // clear interrupt has to be called to enable next interrupt 
	status = VL53L1X_SensorInit(DEV); 		 // initialize the sensor with the default setting
	//status = VL53L1X_StartRanging(DEV);    // enable the ranging
	
	UART_printf("Success");
}

void scanYZ(void){
	if(measurementNum < MAX_MEASUREMENTS){
		status = VL53L1X_StartRanging(DEV);
		ResetLED2();
		for(uint8_t i = 0; i < NUM_SAMPLES; i++){
			step(motor.dir, motor.angle);
			
						
			status = VL53L1X_GetDistance(DEV, &distance);					//The Measured Distance value
			
			transmissionData[measurementNum][i] = distance;
			
			sprintf(printf_buffer,"%u\r\n", distance);
			UART_printf(printf_buffer);
			
			status = VL53L1X_ClearInterrupt(DEV); /* clear interrupt has to be called to enable next interrupt*/
			
			SysTick_Wait10ms(DELAY);
		}
		step(!(motor.dir), STEPS_IN_ROTATION);		// go back home
		scanner.status = SC_COMPLETE;
		measurementNum++;
		SetLED2();
		VL53L1X_StopRanging(DEV);
	}
}

void transmitDistanceSerial(){
	
	SetLED1();
	
	uint8_t input = 0;
	
	//wait for the right transmition initiation code
	while(1){
		input = UART_InChar();		// take in a single byte
		if (input == 's')
			break;
	}
	
	UART_OutChar(measurementNum);
	UART_OutChar(NUM_SAMPLES);
	
	for(uint8_t i = 0; i < measurementNum; i++){
		for(uint8_t j = 0; j < NUM_SAMPLES; j++){
			UART_OutChar((char)(transmissionData[i][j] & MASK));
			UART_OutChar((char)(transmissionData[i][j] >> 8));
		}
	}
	scanner.state = SC_SCAN_MODE;
	scanner.status = SC_COMPLETE;
	measurementNum = 0;
	
	ResetLED1();
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

	if((GPIO_PORTJ_RIS_R & BIT(0))){		// Read RIS register to check if interrupt occured
		scanner.status ^= BIT(0);
		GPIO_PORTJ_ICR_R |= 0x01;     					// Acknowledge flag by setting proper bit in ICR register
	}
	else if((GPIO_PORTJ_RIS_R & BIT(1))){
		scanner.state ^= BIT(0);
		GPIO_PORTJ_ICR_R |= 0x02;     					// Acknowledge flag by setting proper bit in ICR register
	}
}


void scanner_Init(){
	onboardLEDs_Init();
	UART_Init();
	I2C_Init();
	PortM_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	bootVL53L1X();
	SetLED2();
}
