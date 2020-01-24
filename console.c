#include "console.h"

SemaphoreHandle_t hSemaphore_output;	// semaphore to control the output queue

QueueHandle_t queue_uart_input;			// store individual bytes before they can be sent to the command buffer
QueueHandle_t queue_uart_output;		// store strings to be sent to USART using DMA
QueueHandle_t queue_processing;			// store command structures ready for processing

TaskHandle_t hTask_ConsoleProcess;
TaskHandle_t hTask_ConsoleOutput;
TaskHandle_t hTask_CommandBuffer;

void task_CommandBuffer(void * param);	// flush the content of input queue to the command buffer
void task_ConsoleProcess(void * param);	// process the available command structures and post responses to the output queue
void task_ConsoleOutput(void * param);	// dequeue the next output available, and transfer to the USART by DMA

command_buffer_t command_buffer;		// command buffer

/**
* Create queues and tasks for the console application
*/
void Console_Initialize(){
	
	// TASKS:	CALLBACK				NAME		STACK	PARAM	PRIORITY						HANDLE
	xTaskCreate( task_ConsoleProcess, 	"CmdProc", 	500, 	NULL, 	priority_CONSOLE_PROCESSING, 	&hTask_ConsoleProcess);
	xTaskCreate( task_ConsoleOutput, 	"SerOut", 	500, 	NULL, 	priority_CONSOLE_OUTPUT,		&hTask_ConsoleOutput);
	xTaskCreate( task_CommandBuffer, 	"CmdBuff",	500, 	NULL, 	priority_CONSOLE_INPUT,			&hTask_CommandBuffer);
	
	// QUEUES:							LENGTH	ITEM SIZE
	queue_uart_input 	= xQueueCreate( 10, 	sizeof(uint8_t));			// item: individual chars
	queue_uart_output 	= xQueueCreate( 3,		sizeof(output_buffer_t*));	// item: ptr to buffer structure
	queue_processing	= xQueueCreate( 10,	  	sizeof(command_t*)); 		// item: command structure
	
	// initialize buffer:
	command_buffer.length = 0;
	command_buffer.data[0] = NULL;

}

/**
* Feed a single character to the console input.
* Call this function from the USART RX ISR to push the received byte to the console input buffer.
*/
inline void Console_FeedCharFromISR(uint8_t c){
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// COPY the received character to the queue
	xQueueSendFromISR(queue_uart_input, &c, &xHigherPriorityTaskWoken);
	
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
* Free all allocated memory for an output buffer
*/
void Console_DestroyOutputBuffer(output_buffer_t * buffer){

	if(!buffer) return;
	
	// destroy characters pointed by the buffer
	if (buffer->length > 0)
		vPortFree(buffer->data);
	
	// destroy buffer structure
	vPortFree(buffer);
}

/**
* Free all allocated memory for a given command structure
*/
void Console_DestroyCommand(command_t * cmd){

	if (!cmd) return;
	
	// check if there is a command assigned
	if(cmd->cmd) vPortFree(cmd->cmd);
	
	// destroy all created arguments
	if (cmd->argv){	// check if first dimension of array is allocated
		for(uint32_t i = 0; i < cmd->argc; i++)
			vPortFree(cmd->argv[i]);
	}	

	// destroy structure
	vPortFree(cmd);
}

/**
* Receives an array of characters, and enqueue as a message to be sent to the USART
*/
void Console_Write(const char * data, uint32_t length){
	
	// allocate a buffer and copy input data to it
	output_buffer_t * buffer = (output_buffer_t *)pvPortMalloc(sizeof(output_buffer_t));
	if (!buffer) return; // failed to allocate memory
	buffer->length = length;
	buffer->data = (uint8_t *)pvPortMalloc(buffer->length);
	if (!buffer->data) {	// failed to allocate memory	
		Console_DestroyOutputBuffer(buffer);
		return;
	}
	memcpy(buffer->data, data, buffer->length);

	// enqueue for output
	xQueueSend(queue_uart_output,(void*)&buffer, portMAX_DELAY);
}

/**
* Print message for startup
*/
void Console_PrintStartup(){
	char msg [] = __LF __LF STR_STARTUP __LF;
	Console_Write(msg, sizeof(msg));
}

/**
* Print the commands available
*/
void Console_PrintMenu(void){
	
	char msg [1024];	
	strcpy(msg, "List of commands: (use 'help' to show this menu)"__LF);
	
	for(uint32_t i=0;i<COMMAND_COUNT;i++){
		strcat(msg, command_set[i]);
		strcat(msg, __LF);
	}
	Console_Write(msg, strlen(msg));
}

/**
* Print the indicator for user to type the input
*/
void Console_ShowInput(void){
	const char msg [] = __LF"\\>";
	Console_Write(msg, sizeof(msg));
}

/**
* Parse a char buffer to a command structure.
* This function will break down the command buffer into individual elements (tokens)
* and from that it will allocate and populate a command structure in the memory.
* This function returns NULL in case of error.
*/
command_t* Console_ParseCommand(command_buffer_t* buffer) {

	uint32_t start = 0;
	int32_t cursor = 0;
	uint32_t last = start + buffer->length - 1;
	uint32_t token_cnt = 0;

	// check if we have any data to parse
	if (buffer->length < 1) return NULL;

	// allocate a new command structure in the heap using RTOS
	command_t* cmd = (command_t*)pvPortMalloc(sizeof(command_t));
	if (cmd == NULL) return NULL;	// failed to allocate memory
	
	// initialize structure, for proper usage and clean-up
	cmd->cmd = NULL;	// flag that nothing is here yet in case of clean-up 
	cmd->argc = 0;		// no arguments added yet
	cmd->argv = (uint8_t**)pvPortMalloc(sizeof(uint8_t*) * MAX_ARGUMENT_CNT);
	if (cmd->argv == NULL) { // failed to allocate memory
		Console_DestroyCommand(cmd);
		return NULL;	
	}
	
	do {
		// skip any trailing spaces (or until reach end of buffer)
		while (cursor < last && buffer->data[cursor] == ' ') cursor++;
		start = cursor;	// update starting pointer
		
		if (cursor == last && buffer->data[cursor] == ' ')
			break;	// this prevents blank space at the end to be detected as argument

		// move the ptr end till the first ' ' character or the end of the string
		while (cursor < last) {
			if (buffer->data[cursor+1] == ' ') break;
			cursor++;
		}	

		// allocate memory for the newly detected token
		uint32_t token_length = (cursor - start + 1); 
		uint8_t* token = (uint8_t*)pvPortMalloc(token_length+1); // +1: null terminator
		if (token == NULL) {
			Console_DestroyCommand(cmd);
			return NULL;
		}

		// copy token to the new memory
		memcpy(token, &(buffer->data[start]), token_length); 

		// add token to the correct place in the allocated structure:
		if (token_cnt == 0) {	// first token: command
			cmd->cmd = token;
			cmd->cmd[token_length] = NULL;
		}
		else {			// other tokens are arguments
			cmd->argv[cmd->argc] = token;
			cmd->argv[cmd->argc][token_length] = NULL;
			cmd->argc++;
		}
		
		token_cnt++;

		// update pointers for next iteration:
		cursor++;	// move to the next character	

	} while (cursor <= last);	// also 'equal' to include last char in next iteration

	if (token_cnt == 0) { // no tokens were detected
		Console_DestroyCommand(cmd);
		cmd = NULL;
	}

	return cmd;
}

/**
* Returns whether a given character is allowed in the input for building a command
*/
inline uint32_t Console_IsAllowedInputChar(uint8_t c){
	return ((c >= 32) && (c <= 126)) || c=='\n';
}

/**
* Process the received command by invoking the appropriate handler
*/
void Console_ProcessCommand(command_t * cmd){
	
	if (!cmd) return;	// check NULL command

	// first test for reserved keywords:
	if (strcmp((char*)cmd->cmd, "help")==0){
		Console_PrintMenu();
		return;
	}		
	
	// iterate over the menu entries
	for(uint32_t i = 0; i < COMMAND_COUNT; i++){
		if (strcmp((char*)cmd->cmd, command_set[i])==0){
			command_handler_set[i](cmd->argc, (char**)cmd->argv);
			return;
		}		
	}
	
	const char msg [] = "Uknown command."__LF;
	Console_Write(msg, sizeof(msg));
}

/**
* Call this function from DMA transfer complete interrupt to unblock 
* output task.
*/
void Console_OutputDmaFinishedFromISR(void){
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// notify output task	
	vTaskNotifyGiveFromISR(hTask_ConsoleOutput, &xHigherPriorityTaskWoken);
	
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// -----------------------------------------------------------------------------------
// Console tasks ---------------------------------------------------------------------
// -----------------------------------------------------------------------------------

// the task that manage the command buffer resource
void task_CommandBuffer(void * param){

	while(1){
				
		// block until there is a character in the queue
		// then move byte to command buffer
		xQueueReceive(queue_uart_input, &(command_buffer.data[command_buffer.length]), portMAX_DELAY);
		
		// check if its a printable character or command character, if not, ignore it
		if (!Console_IsAllowedInputChar(command_buffer.data[command_buffer.length])) 
			continue;
		
		// echo back the received character (last in the buffer)
		Console_Write((char *)&(command_buffer.data[command_buffer.length]), 1);
		
		// check received data for '\n' character
		if (command_buffer.data[command_buffer.length] == '\n'){
			
			// breakdown command buffer into tokens and create command_t structure
			command_t * new_command = Console_ParseCommand(&command_buffer);
			
			// push new structure to the processing queue that in turn will notify processing task
			xQueueSend(queue_processing, (void*)&new_command, portMAX_DELAY);
						
			// reset buffer
			command_buffer.length = 0;
			
		}else{	// if no '\n' was found yet...

			command_buffer.length++;	// move cursor ahead (notice that this length does not include the \n)

			// protect against buffer overflow
			if (command_buffer.length >= MAX_COMMAND_LENGTH){
				command_buffer.length = 0; 				// makes buffer circular
			}
		}
	}
}

void task_ConsoleProcess(void * param){

	command_t * cmd;
	
	// print info header and command menu
	Console_PrintStartup();
	Console_PrintMenu();
	
	while(1){
		
		Console_ShowInput();
		
		// block until we have a command in the processing queue
		xQueueReceive(queue_processing, &cmd, portMAX_DELAY);
		
		// process command
		Console_ProcessCommand(cmd);
		
		// release allocated memory
		Console_DestroyCommand(cmd);
	}
}

void task_ConsoleOutput(void * param){

	output_buffer_t * buffer;
	
	// block until we have a string in the output queue
	while(1){
		
		// block until we have a command in the processing queue
		xQueueReceive(queue_uart_output, &buffer, portMAX_DELAY);
		
		// start the DMA transfer
		Console_OutputData(buffer->data, buffer->length);
		
		// block until DMA transfer is complete
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	//pdTRUE makes it a binary (rather than a counting) semaphore

		// release allocated memory
		Console_DestroyOutputBuffer(buffer);
	}
}


// -----------------------------------------------------------------------------------
// Handlers for menu items -----------------------------------------------------------
// -----------------------------------------------------------------------------------

// handler for command: "LED #"
void CommandHandler_Led(int argc, char* argv[]){
	if (argc > 0){
		if (strcmp(argv[0], "ON") == 0){
			LED_On();
		}else if(strcmp(argv[0], "OFF") == 0) {
			LED_Off();
		}else{
			const char msg [] = "Invalid argument."__LF;
			Console_Write(msg, sizeof(msg));
		}
	}else{
		const char msg [] = "Expected argument."__LF;
		Console_Write(msg, strlen(msg));
	}
}

// handler for command: "LED2 #"
void CommandHandler_Led2(int argc, char* argv[]){
	if (argc > 0){
		if (strcmp(argv[0], "ON") == 0){
			LED2_On();
		}else if(strcmp(argv[0], "OFF") == 0) {
			LED2_Off();
		}else{
			const char msg [] = "Invalid argument."__LF;
			Console_Write(msg, sizeof(msg));
		}
	}else{
		const char msg [] = "Expected argument."__LF;
		Console_Write(msg, strlen(msg));
	}
}
