#ifndef CONFIG_H
#define CONFIG_H

// -------------------------------------------------------------------------------------------------------------
// IRQ PRIORITIES: (lower number is higher priority)
// -------------------------------------------------------------------------------------------------------------
// Remark: must respect RTOS IRQ threshold for IRS safe routines:
// 		Where value: configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY <  priority  < (2^configPRIO_BITS)

#define priority_UART_ISR				6		// character received from terminal
#define priority_DMA2_S7_ISR			7		// finished to print data to terminal

// -------------------------------------------------------------------------------------------------------------
// TASK PRIORITIES:	(higher number is higher priority)
// -------------------------------------------------------------------------------------------------------------
// Remark: priority 0 is Idle task thus its not allowed.

#define priority_CONSOLE_PROCESSING		3		// command processing priority
#define priority_CONSOLE_INPUT			2		// receive from input queue priority
#define priority_CONSOLE_OUTPUT			2		// write output to serial port priority

#define priority_HEARTBEAT				1		// system activity indicator priority


#endif
