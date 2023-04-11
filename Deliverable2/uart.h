/* COMPENG 2DX3 Final Project
 * Ivan Lange
 * April 12, 2023
 * main.c
 *
 *
 * Assumes system clock of 12 MHz
 */


/*********************************************************
*                    GLOBAL VARIABLES
*********************************************************/
static char printf_buffer[1023];


/*********************************************************
*                  FUNCTION PROTOTYPES
*********************************************************/
void UART_Init(void);
// Wait for new input, then return ASCII code 
char UART_InChar(void);
// Wait for buffer to be not full, then output 
void UART_OutChar(char data);
void UART_printf(const char* array);
void Status_Check(char* array, int status);