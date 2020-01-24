/**
	Title: µConsole for FreeRTOS in ARM Cortex-M
	
		A minimalistic console for microcontrollers using FreeRTOS

	How this console works:
	
		The console simulates a very simple terminal that allows the user to enter commands and receive
		text messages. Several tasks will maintain the multiple I/O services running while a high priority
		task will process the commands that are ready.
		
	Configuration:
	
		Encoding:				8-bit ASCII character set
		Line break (output):	LF '\n' (ASCII: 10)
		Line break (input):		LF '\n' (ASCII: 10)
		Allowed in input:		ASCII 32 up to ASCII 126 or LF
		
	Data flow:
	
	------- Action ----------------------------------------------------------- Who does it?
		1.	Receive USART byte within ISR	
		2.	Push received byte to the queue 									(RX ISR)
		3.	Whenever queue not empty, push queue to the command buffer 			(TASK 1)
		4.	Check if command buffer is ready (\n was found)						(TASK 2)
		5.	Breakdown command buffer into command and parameters 
		7.	Find the menu entry that match the command
		8.	Invoke the handler whose index is the same as the menu entry
		9.	Write the response string to the output queue
		10.	Whenever output queue is not empty, push to USART by DMA 			(TASK 3)
	----------------------------------------------------------------------------------------
	
	Overall console in/out data flow:
		
		RX -(ISR)-> IN QUEUE -> COMMAND BUFFER -> PROCESSING -> RESPONSE -> OUT QUEUE -(DMA)-> USART -> TX
	
	Implementation decisions:
		
		The data reception: 
			Data input is processed character by character due to the way data 	is entered (user typing)
			in the console. Since interval between characters is long, and length of command is unknown,
			it seems suitable for ISR handling instead of DMA or polling. After receival by the ISR, the 
			data is enqueued for assembling a valid command in the command buffer.
		The data transmission:
			Data output is sent in text blocks handled by a DMA peripheral. Due to the fact that we know
			the amount of data to be transmitted, this transmission can be optimized by DMA. Blocks of
			text, are enqued prior to be scheduled for transfer by DMA. This is achieved by a dedicated
			RTOS task.

	Limitations:
		Due to the restrictions of the device, the following limitations were made:
		
		Maximum command Length: 255 bytes (ASCII characters) including all parameters
		Maximum parameters per command:  10 parameters separated by single blank space character
		
		These limitations can be changed in the header file, but its recommended to stick to the default.
		
*/

#ifndef CONSOLE_H
#define CONSOLE_H

#include "config.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "queue.h"                      // ARM.FreeRTOS::RTOS:Core
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core

#include "board.h"

// Exported definitions
#define PASS	1
#define TRUE	1
#define FAIL	0
#define FALSE	0

#define MAX_COMMAND_LENGTH	255			// limited by device's RAM
#define MAX_ARGUMENT_CNT	10			// increasing this number will increase pre-allocated arrays

#define __LF				"\n"

#define STR_STARTUP			"Welcome to uConsole! -- (c) Giovani Luigi @ 2020"

// Exported typedefs
typedef char* CommandSet_t[];
typedef void(*CommandHandlerSet_t[])(int, char*[]);

typedef struct {						// struct used for command input
	uint8_t data[MAX_COMMAND_LENGTH+1];
	uint32_t length;
} command_buffer_t;

typedef struct{							// struct used for output messages
	uint8_t * data;						// string (not null terminated)
	uint32_t length;					// amount of chars in the string
} output_buffer_t;

typedef struct {						// struct used to store command and its arguments
	uint8_t* cmd;						// null terminated string
	uint32_t argc;						// arg. count
	uint8_t** argv;						// argument vector of null terminated strings
} command_t;

// Imported functions
extern void Console_OutputData(uint8_t * data, uint32_t length);

// Exported functions
void Console_Initialize(void);
void Console_FeedCharFromISR(uint8_t c);
void Console_OutputDmaFinishedFromISR(void);
void Console_Write(const char * data, uint32_t length);

// -----------------------------------------------------------------------------------
// Commands: Edit the values inside this region to customize the MENU: ---------------
// -----------------------------------------------------------------------------------

#define	COMMAND_COUNT	2

// prototypes
void CommandHandler_Led(int argc, char* argv[]);
void CommandHandler_Led2(int argc, char* argv[]);

static CommandSet_t command_set = {
	"LED",
	"LED2"
};

static CommandHandlerSet_t command_handler_set = {
	CommandHandler_Led,		// "LED"
	CommandHandler_Led2,	// "LED2"
};

// -----------------------------------------------------------------------------------

#endif
