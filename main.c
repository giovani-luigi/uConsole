/**
*	-----------------------------------------------------------------------------------------------------------
*
*	Title: A very simple RS232 Console with FreeRTOS and STM32 HAL for STM32F7xx microcontroller
*
*	Board: 	STM32F746G-DISCO
*	Author:	Giovani Luigi R. Brondani
*	Date: 	Jan. 20th - 2020
*	Device:	STM32F746NGHx
*	Family:	STM32F7xx
*
*	This project demonstrates a console over the USART 1 of the board (connected to the ST-Link VCP) allowing
*	connection to any PC USB. The system prints a console in the screen where the user can issue commands
*	using any terminal emulator. The reception and transmission is handled through queues and tasks from the
*	FreeRTOS kernel. USART interrupts and DMA provide a very efficient way of handling data transfers, leaving
*	the CPU free for whatever tasks the user want to create, and all the processing that can be performed for
*	each of the console commands. Additionally the console menu is fully customizable from the Console.h file.
*	
*	To add a new command to the menu:
*		1. Add the syntax of the command in the string array "command_set"
*		2. Increment the command count in the pre-processor macro COMMAND_COUNT
*		3. Add the handler prototype in the callback array "command_handler_set"
*		4. Implement the handler callback following the prototype:  void (int, char*[])
*			-> the callback signature is identical to standard main(int argc, char* argv[])
*
*	The currently supported commands are:
*		"LED {1}" => where parameter {1} can be either "ON" or "OFF";
*		"LED2 {1}" => where parameter {1} can be either "ON" or "OFF";
*
*	The board clock is left at the default reset config. which means we are using the HSI OSC @ 16MHz.
*
*	Virtual Com Port (VCP) in the schematic is connected to the pins:
*	VCP_RX => PB7  (MCU RX -<- ST-LINK TX) Alternate Function AF7 (Datasheet 'Rev4' Page 76)
*	VCP_TX => PA9  (MCU TX ->- ST-LINK RX) Alternate Function AF7 (Datasheet 'Rev4' Page 76)
*
*  -----------------------------------------------------------------------------------------------------------
*/

#include "config.h"
#include "stm32f7xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "FreeRTOS.h"
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "queue.h"

#include "board.h"
#include "console.h"

typedef void(*usart_callback)(void);

TaskHandle_t hTask_Heartbeat;

void task_Heartbeat(void * param); 		// this keeps toggling a LED to indicate CPU activity

UART_HandleTypeDef huart;				// handle for the USART console peripheral
DMA_HandleTypeDef hdma;					// handle for the DMA used for console data output

void UART_Initialize(void){
	
	// enable peripheral clock
	__USART1_CLK_ENABLE();				// USART1 
	__GPIOB_CLK_ENABLE();				// RX pin
	__GPIOA_CLK_ENABLE();				// TX pin
	
	// configure serial module
	huart.Instance = USART1;
	huart.Init.BaudRate = 9600;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	
	// configure pins for TX and RX
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// TX pin A9
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// RX pin B7
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
	// initialize serial module ignoring errors
	HAL_UART_Init(&huart);
	
}

/**
* This function setup manually RX interrupts for USART
*/
void UART_EnableReceiverInterrupt(void){

	// disable global interrupts during configuration
	__disable_irq();
	
	// enable receiver interrupt
	USART1->CR1 |= 0x20;				// enable RX interrupt (bit 5)
	
	// set priority respecting RTOS IRQ priority threshold
	NVIC_SetPriority(USART1_IRQn, priority_UART_ISR);
	
	// enable USART1 interrupt line in the NVIC
	NVIC_EnableIRQ(USART1_IRQn);
	
	// re-enable global interrupt
	__enable_irq();
	
}

void UART_EnableTransmitterDMA(void){

	// use DMA 2 to map data to USART TX data register
	
	// enable DMA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	
	// DMA Configuration:
	
	// disable the stream prior to configure it:
	DMA2_Stream7->CR &=~ DMA_SxCR_EN;	// remark: not immediate effect
	
	// read it back to ensure no ongoing operation
	while (DMA2_Stream7->CR & DMA_SxCR_EN);	// wait until clears so its ready for configuration
	
	// all bits set in status must be cleared before re-enabling the stream
	// Interrupt Status Register: Hi REG. -> Streams 0 to 3, Lo REG. -> Streams 4 to 7.
	DMA2->HISR &=~ 0x0F400000;	// 'DMA_HISR' Status bits for Stream 7: 27,26,25,24 and 22.
	
	// set the Peripheral Address Register address in the DMA_SxPAR
	DMA2_Stream7->PAR = (uint32_t)&(USART1->TDR);

	// Channel Selection: { 4 (0b100) }
	DMA2_Stream7->CR |=  DMA_SxCR_CHSEL_2;		// bit  27 = 1
	DMA2_Stream7->CR &=~ DMA_SxCR_CHSEL_1;		// bit  26 = 0
	DMA2_Stream7->CR &=~ DMA_SxCR_CHSEL_0;		// bit  25 = 0
	// memory burst transfer config. { Single transfer (0b00) }
	DMA2_Stream7->CR &=~ DMA_SxCR_MBURST_1;		// bit  24 = 0
	DMA2_Stream7->CR &=~ DMA_SxCR_MBURST_0;		// bit  23 = 0
	// peripheral burst transfer config. { Single transfer (0b00) }
	DMA2_Stream7->CR &=~ DMA_SxCR_PBURST_1;		// bit  22 = 0
	DMA2_Stream7->CR &=~ DMA_SxCR_PBURST_0;		// bit  21 = 0
	// current target { memory 0 (0) - only for doub. buff. }
	DMA2_Stream7->CR &=~ DMA_SxCR_CT;			// bit  19 = 0
	// double buffer mode { Off = 0 }
	DMA2_Stream7->CR &=~ DMA_SxCR_DBM;			// bit  18 = 0
	// priority level among other streams { low (0/3) = 0b00 }
	DMA2_Stream7->CR &=~ DMA_SxCR_PL_1;			// bit  17 = 0
	DMA2_Stream7->CR &=~ DMA_SxCR_PL_0;			// bit  16 = 0
	// peripheral increment offset size {'0'=PSIZE ('1'= 4 i.e. 32-bit)}
	DMA2_Stream7->CR &=~ DMA_SxCR_PINCOS;		// bit  15 = 0
	// memory data size { byte = 0b00 (half-word = 0b01, word = 0b10) }
	DMA2_Stream7->CR &=~ DMA_SxCR_MSIZE_1;		// bit  14 = 0
	DMA2_Stream7->CR &=~ DMA_SxCR_MSIZE_0;		// bit  13 = 0
	// peripheral data size { byte = 0b00 (half-word = 0b01, word = 0b10) }
	DMA2_Stream7->CR &=~ DMA_SxCR_PSIZE_1;		// bit  12 = 0
	DMA2_Stream7->CR &=~ DMA_SxCR_PSIZE_0;		// bit  11 = 0
	// memory increment mode {1=inc. according to MSIZE (0=no inc.)}
	DMA2_Stream7->CR |=  DMA_SxCR_MINC;			// bit  10 = 1
	// peripheral increment mode {0=no inc. (1=inc. according to PSIZE)}
	DMA2_Stream7->CR &=~ DMA_SxCR_PINC;			// bit   9 = 0
	// circular mode {0=disabled (1=enabled)}
	DMA2_Stream7->CR &=~ DMA_SxCR_CIRC;			// bit   8 = 0
	// data transfer direction {01 mem>per (00=per>mem,10mem>mem) }
	DMA2_Stream7->CR &=~ DMA_SxCR_DIR_1;		// bit   7 = 0
	DMA2_Stream7->CR |=  DMA_SxCR_DIR_0;		// bit   6 = 1
	// peripheral flow controller {0 = DMA controls flow (i.e. now data length)}
	DMA2_Stream7->CR &=~ DMA_SxCR_PFCTRL;		// bit   5 = 0
	// transfer complete interrupt enable
	DMA2_Stream7->CR |=  DMA_SxCR_TCIE;			// bit   4 = 1
	// half transfer complete interrupt enable
	DMA2_Stream7->CR &=~ DMA_SxCR_HTIE;			// bit   3 = 0
	// transfer error interrupt enable
	DMA2_Stream7->CR &=~  DMA_SxCR_TEIE;		// bit   2 = 0
	// direct mode error interrupt enable
	DMA2_Stream7->CR &=~  DMA_SxCR_DMEIE;		// bit   1 = 0
	// enable stream ( flag stream ready when read low ) { off = 0 }
	DMA2_Stream7->CR &=~  DMA_SxCR_EN;			// bit   0 = 0

	// configure USART module for DMA transmission
	USART1->CR3 |= USART_CR3_DMAT;

	// disable global interrupt
	__disable_irq();
	
	// configure priority
	NVIC_SetPriority(DMA2_Stream7_IRQn, priority_DMA2_S7_ISR);
	
	// enable Stream 7 IRQ
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	// re-enable global interrupt
	__enable_irq();

}

void task_Heartbeat(void * param){
	while(1){
		LED2_Toggle();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

int main(void){

	Board_Initialize();
	
	UART_Initialize();
	UART_EnableReceiverInterrupt();
	
	const char * msg = __LF"System starting..."__LF;
	HAL_UART_Transmit(&huart, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	
	UART_EnableTransmitterDMA();
	
	// TASKS:	CALLBACK				NAME			STACK	PARAM	PRIORITY					HANDLE
	xTaskCreate( task_Heartbeat, 		"HeartBeat",	500, 	NULL, 	priority_HEARTBEAT,			&hTask_Heartbeat);
	
	Console_Initialize();
	
	vTaskStartScheduler();
	
	while(1);
}

// -------------------------------------------------------------------------------------------------------
// Console port specific implementation:
// -------------------------------------------------------------------------------------------------------

void USART1_IRQHandler(void){
	if (USART1->ISR & 0x20){				// if bit 5 (Data received flag)
		// send the received character to console
		Console_FeedCharFromISR((uint8_t)(USART1->RDR));		
	}else{	
		if (READ_BIT(USART1->ISR, USART_ISR_ORE)){ 	// usart overrun 
			SET_BIT(USART1->ICR, USART_ICR_ORECF);	// set ICR to clear
			// here the buffer should be corrupted. We can flush and restart the input process if we want...
		}
	}	
}

void DMA2_Stream7_IRQHandler(void){
	if (DMA2->HISR & DMA_HISR_TCIF7){		// test for transfer complete interrupt
		DMA2->HIFCR |=  DMA_HIFCR_CTCIF7; 	// (High) Interrupt Flag Clear Register (write 1 to clear)
		Console_OutputDmaFinishedFromISR();	// flag RTOS to unblock output queue
	}
}

void Console_OutputData(uint8_t * data, uint32_t length){
	
	while( DMA2_Stream7->CR & DMA_SxCR_EN); // wait if dma is busy
	
	// set the Memory Address (0) Register to the beginning of RAM in the memory
	DMA2_Stream7->M0AR = (uint32_t)data;
	
	// set the number of data items for the transfer
	DMA2_Stream7->NDTR = length & 0xFFFF; // (16-bit) this is decremented at every transfer

	// clear transmission flag
	USART1->ICR |= USART_ICR_TCCF;		// clear flag of ISR reg. by setting ICR reg.
	
	// activate the stream (enable dma)
	DMA2_Stream7->CR |= DMA_SxCR_EN;
}

// -------------------------------------------------------------------------------------------------------
