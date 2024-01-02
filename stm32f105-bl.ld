/* Linker script for STM32F105RBT6, 128K flash, 64K RAM). */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 16K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

