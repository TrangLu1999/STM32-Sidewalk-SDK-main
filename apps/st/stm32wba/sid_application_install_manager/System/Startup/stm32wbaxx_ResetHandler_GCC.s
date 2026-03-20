/**
  ******************************************************************************
  * File Name          : stm32wbaxx_ResetHandler.s
  * Author             : MCD Application Team
  * Description        : STM32WBA5xx Ultra Low Power Devices specific
  *                      Reset handler for connectivity applications.
                         GCC toolchain.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * Cortex-M version
  *
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m33
  .fpu softvfp
  .thumb

  .extern  SystemInit
  .extern  AIM_BootUserApp

/* INIT_BSS macro is used to fill the specified region [start : end] with zeros */
.macro INIT_BSS start, end
  ldr r0, =\start
  ldr r1, =\end
  movs r3, #0
  bl LoopFillZerobss
.endm

/* INIT_DATA macro is used to copy data in the region [start : end] starting from 'src' */
.macro INIT_DATA start, end, src
  ldr r0, =\start
  ldr r1, =\end
  ldr r2, =\src
  movs r3, #0
  bl LoopCopyDataInit
.endm

.section  .text.data_initializers
CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc  CopyDataInit
  bx lr

FillZerobss:
  str  r3, [r0]
  adds r0, r0, #4

LoopFillZerobss:
  cmp r0, r1
  bcc FillZerobss
  bx lr

  .section .text.Reset_Handler
  .global Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  /* Check if this is a wake-up from Standby. It's not allowed to use stack for this to avoid interference with the app */
  /* Enable PWR peripheral clock in RCC */
  ldr r0, =0x46020C94 /* RCC->AHB4ENR register address */
  ldr r1, =0x00000004 /* PWREN bit mask in RCC->AHB4ENR */
  ldr r2, [r0]        /* Read RCC->AHB4ENR value */
  orr r2, r2, r1      /* r2 = r2 | r1  (sets PWREN bit) */
  str r2, [r0]        /* Write r2 back to RCC->AHB4ENR */
  ldr r1, [r0]        /* Read RCC->AHB4ENR again to introduce a delay after enabling PWR clock */

  /* Check SBF flag in PWR->SR register */
  ldr r0, =0x46020808 /* PWR->SR register address */
  ldr r1, =0x00000004 /* SBF bit mask in PWR->SR */
  ldr r2, [r0]        /* Read PWR->SR value */
  tst r2, r1
  beq EnterAIM        /* SBF bit is not set - proceed with Application Install Manager startup */

  /* Check RCC->CSR register */
  ldr r0, =0x46020CF4 /* RCC->CSR register address */
  ldr r1, [r0]        /* Read RCC->CSR value */
  cmp r1, #0
  bne EnterAIM        /* CSR is non-zero - proceed with Application Install Manager startup */

  /* Exit from Standby detected - jump to the application directly */
  bl AIM_BootUserApp
  b  LoopForever      /* Defensive programming measure if AIM_BootUserApp somehow returns */

  /* Regular startup of the Application Install Manager */
EnterAIM:
  /* Initialize AIM stack pointer */
  ldr   r0, =_estack
  mov   sp, r0        /* set stack pointer */

  /* Call the clock system initialization function.*/
  bl  SystemInit

  /* Copy the data segment initializers from flash to SRAM */
  INIT_DATA _sdata, _edata, _sidata

  /* Zero fill the bss segments. */
  INIT_BSS _sbss, _ebss

  /* Call static constructors */
  bl __libc_init_array
  /* Call the application s entry point.*/
  bl	main

LoopForever:
  b LoopForever

  /* end of specific code section for standby */
  .end
