/**
  ******************************************************************************
  * @file      stm32wlxx_ResetHandler_GCC.s
  * @author    MCD Application Team
  * @brief     STM32WLxx devices specific Reset handler for Sidewalk
  *            applications. GCC toolchain. This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set stack and heap watermarks for utilization monitoring,
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/* start address for the user stack section. defined in linker script */
.word   __start_STACK
/* end address for the user stack section. defined in linker script */
.word   __end_STACK
/* start address for the user stack guard section. defined in linker script */
.word   __start_STACK_GUARD
/* end address for the user stack guard section. defined in linker script */
.word   __end_STACK_GUARD
/* start address for the user heap section. defined in linker script */
.word   __start_HEAP
/* end address for the user heap section. defined in linker script */
.word   __end_HEAP

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .global Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
/* Fill entire stack with 0xAA55AA55 watermark */
  ldr r1, = __start_STACK
  ldr r2, = __end_STACK
  mov r3, #0xAA55
  movt r3, #0xAA55
FillStackWatermark:
  cmp r1, r2
  bge StackGuardInit
  str r3, [r1], #4
  b FillStackWatermark

StackGuardInit:
/* Fill stack guard area with 0xDEADBEEF watermark */
  ldr r1, = __start_STACK_GUARD
  ldr r2, = __end_STACK_GUARD
  mov r3, #0xBEEF
  movt r3, #0xDEAD
FillStackGuardWatermark:
  cmp r1, r2
  bge HeapWatermarkInit
  str r3, [r1], #4
  b FillStackGuardWatermark

HeapWatermarkInit:
/* Fill user heap area with 0x55AA55AA watermark */
  ldr r1, = __start_HEAP
  ldr r2, = __end_HEAP
  mov r3, #0x55AA
  movt r3, #0x55AA
FillHeapWatermark:
  cmp r1, r2
  bge InitStackPointer
  str r3, [r1], #4
  b FillHeapWatermark

InitStackPointer:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Call the clock system initialization function.*/
  bl  SystemInit

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
    b LoopForever
