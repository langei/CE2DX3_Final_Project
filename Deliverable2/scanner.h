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
//Motor Step 2.8125 deg, 128 samples: 16 steps
//Motor Step 5.625 deg, 64 samples: 32 steps
//Motor Step 11.25 deg, 32 samples: 64 steps
//Motor Step 22.5 deg, 16 samples: 128 steps
//Motor Step 45 deg, 8 samples: 256 steps

#define STEP_SIZE										(16)


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
uint8_t rangeStatus;
uint8_t measurementNum = 0;
uint16_t distance = 0;
uint16_t wordData = 0;
uint16_t signalRate;
uint16_t ambientRate;
uint16_t spadNum; 

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

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
  //Use PortG0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
  GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

  return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void bootVL53L1X(void){
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(DEV, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(DEV, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(DEV); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(DEV);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

 // status = VL53L1X_StartRanging(DEV);   // This function has to be called to enable the ranging
//	
//	PortG_Init();
//	VL53L1X_XSHUT();
//	
//	while(sensorState==0){
//		status = VL53L1X_BootState(DEV, &sensorState);
//		SysTick_Wait10ms(10);
//  }
//	
//	status = VL53L1X_ClearInterrupt(DEV);  // clear interrupt has to be called to enable next interrupt 
//	status = VL53L1X_SensorInit(DEV); 		 // initialize the sensor with the default setting
//	//status = VL53L1X_StartRanging(DEV);    // enable the ranging
//	
//	UART_printf("Success");
}

void scanYZ(void){
	if(measurementNum < MAX_MEASUREMENTS){
		ResetLED2();
		status = VL53L1X_StartRanging(DEV);
		for(uint8_t i = 0; i < NUM_SAMPLES; i++){
			step(motor.dir, motor.angle);
			GPIO_PORTH_DATA_R &= ~0xF;
			//wait until the ToF sensor's data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(DEV, &dataReady);
						FlashLED3(1);
						VL53L1_WaitMs(DEV, 5);
			}
			dataReady = 0;
			//read the data values from ToF sensor
			status = VL53L1X_GetRangeStatus(DEV, &rangeStatus);
			status = VL53L1X_GetDistance(DEV, &distance);					//The Measured Distance value
			status = VL53L1X_GetSignalRate(DEV, &signalRate);
			status = VL53L1X_GetAmbientRate(DEV, &ambientRate);
			status = VL53L1X_GetSpadNb(DEV, &spadNum);
			
			status = VL53L1X_ClearInterrupt(DEV); /* clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", rangeStatus, distance, signalRate, ambientRate,spadNum);
		UART_printf(printf_buffer);
	
			
			transmissionData[measurementNum][i] = distance;
			
			status = VL53L1X_ClearInterrupt(DEV); /* clear interrupt has to be called to enable next interrupt*/
			SysTick_Wait10ms(DELAY);
		}
		VL53L1X_StopRanging(DEV);
		step(!(motor.dir), STEPS_IN_ROTATION);		// go back home
		scanner.status = SC_COMPLETE;
		measurementNum++;
		SetLED2();

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
	PortH_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	bootVL53L1X();
	SetLED2();
}
