# µConsole for FreeRTOS in ARM Cortex-M

This project consists of a very simple but effective and efficient console command interpreter using FreeRTOS.
Only a couple of .c/.h is required to add the console to a project. 

The code ships with a demo project, using µVision Keil IDE, targeting the STM32F746G-DISCO demo board.
This project can be used as starting point for any other project.

## Middleware

 This project requires FreeRTOS 10+
 > I have not tested with other versions, but it should work fine once the used API functions are available.

## Adding the console to your own project

To add the µConsole to your project, just copy the files **console.c** and **console.h**
You also have to define the priorities of the console tasks:

```c
#define priority_CONSOLE_PROCESSING		3		// command processing priority
#define priority_CONSOLE_INPUT			2		// receive from input queue priority
#define priority_CONSOLE_OUTPUT			2		// write output to serial port priority
```
After adding the files and the pre-processor macros, you need to implement a few port-specific functions in order to feed the console input and write the output to the USART peripheral.

For an example of the functions that must be implemented, check the **main.c** file of the project.

You can customize the set of commands of the console by modifying the file **console.h**
```c

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

```
Add the commands as strings in the variable **command_set** then add the respective handler function of the command, to the array **command_handler_set**. Finally the callback handler can be defined using the following signature:
```c
void command_callback(int argc, char * argv []){
	// add here your own code
}
```

## How it works?

The console simulates a very simple terminal that allows the user to enter commands and receive text messages. Several tasks will maintain the multiple I/O services running while a high priority task will process the commands that are ready.
Each command is associated with a callback function that will be called with the standard signature **int argc, char* argv[]**.

## Supported char-set and terminal settings

- Encoding:				8-bit ASCII character set
- Line break (output):	LF '\n' (ASCII: 10)
- Line break (input):		LF '\n' (ASCII: 10)
- Allowed in input:		ASCII 32 up to ASCII 126 or LF

## Rename a file

You can rename the current file by clicking the file name in the navigation bar or by clicking the **Rename** button in the file explorer.

## Console data flow
```c
/*
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
*/
```
