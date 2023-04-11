/* COMPENG 2DX3 Final Project
 * Ivan Lange
 * April 12, 2023
 * main.c
 *
 *
 * Assumes system clock of 12 MHz
 */

/*********************************************************
*                   FUNCTION PROTOTYPES
*********************************************************/
void FlashLED1(int count);
void FlashLED2(int count);
void FlashLED3(int count);
void FlashLED4(int count);
void FlashAllLEDs(void);
void FlashI2CError(int count);
void FlashI2CTx(void);
void FlashI2CRx(void);
void onboardLEDs_Init(void);
void SetLED1();
void SetLED2();
