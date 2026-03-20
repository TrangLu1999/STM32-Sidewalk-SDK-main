/*
******************************************************************************
**
** @file        : STM32WBA5xxG_Sidewalk_AIM.ld
**
** @author      : Application team
**
** @brief       : Linker script for STM32WBA5xxG Device from STM32WBA series
**                      1024 KBytes FLASH
**                      128 KBytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
******************************************************************************
** @attention
**
** Copyright (c) 2025 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

_Min_Heap_Size    = 0x0400;         /* required amount of heap  */
_Stack_Alignment  = 8;              /* Should equal to the stack frame size */
_Stack_Size       = 0x700;          /* required amount of stack */

_Entry_Point_Offset        = 0x4;   /* Offset of the Reset Handler ISR vector - application entry point is located at this offset */
_Magic_Word_Pointer_Offset = 0x188; /* Offset (relative to app image start) at which the address of the magic word is stored */
_Flash_Page_Size           = 8K;

/* Memories definition */
MEMORY
{
  RAM              (rw) : ORIGIN = 0x20000010, LENGTH = (128K - 0x10)
  AIM_FLASH        (rx) : ORIGIN = 0x08000000, LENGTH = 56K
  OTA_STATUS_FLASH (rw) : ORIGIN = 0x0800E000, LENGTH = 8K
  APP_FLASH        (rw) : ORIGIN = 0x08010000, LENGTH = 432K
  STAGING_FLASH    (rw) : ORIGIN = 0x0807C000, LENGTH = 440K
  NVM_STORAGE      (r)  : ORIGIN = 0x080EA000, LENGTH = 88K
}

__AIM_FLASH_region_start__ = ORIGIN(AIM_FLASH);
__AIM_FLASH_region_end__   = (ORIGIN(AIM_FLASH) + LENGTH(AIM_FLASH));
__AIM_FLASH_region_size__  = LENGTH(AIM_FLASH);

/* Sections */
SECTIONS
{
  /* The startup code into "AIM_FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >AIM_FLASH

  /* The program code and other data into "AIM_FLASH" Rom type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >AIM_FLASH

  /* Constant data into "AIM_FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >AIM_FLASH

  .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >AIM_FLASH

  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >AIM_FLASH

  .preinit_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >AIM_FLASH

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >AIM_FLASH

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >AIM_FLASH

  /* Put user mode stack at the very beginning of RAM, this ensures that stack overflow will immediately trigger hard fault due to access to non-implemented memory space */
  .stack ORIGIN(RAM) :
  {
    . = ALIGN(_Stack_Alignment);
    PROVIDE ( __start_STACK = . );
    . = . + _Stack_Size;
    . = ALIGN(_Stack_Alignment);
    PROVIDE ( _estack = . ); /* Highest address of the user mode stack */
    PROVIDE ( __end_STACK = . );
  } >RAM

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM AT> AIM_FLASH

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap section, used to check that there is enough "RAM" Ram  type memory left */
  ._user_heap (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( __start_HEAP = . );
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
    PROVIDE ( __end_HEAP = ORIGIN(RAM) + LENGTH(RAM) );
  } >RAM

  /* OTA status page */
  .ota_status_area (NOLOAD):
  {
    . = ALIGN(_Flash_Page_Size);
    PROVIDE_HIDDEN (__start_ota_status_area = .);
    PROVIDE_HIDDEN (__end_ota_status_area = ORIGIN(OTA_STATUS_FLASH) + LENGTH(OTA_STATUS_FLASH));
    *(.ota_status_area)
    *(.ota_status_area*)
  } >OTA_STATUS_FLASH

  /* Active application slot */
  .app_area (NOLOAD) :
  {
    . = ALIGN(_Flash_Page_Size);
    PROVIDE_HIDDEN (__start_app_active_area = .);
    PROVIDE_HIDDEN (__start_active_app_vtor = .);
    PROVIDE_HIDDEN (__active_app_estack = .);
    PROVIDE_HIDDEN (__active_app_entry = . + _Entry_Point_Offset);
    PROVIDE_HIDDEN (__start_aim_active_slot_header = . + _Magic_Word_Pointer_Offset);
    PROVIDE_HIDDEN (__offset_aim_active_slot_header = _Magic_Word_Pointer_Offset);
    PROVIDE_HIDDEN (__end_app_active_area = ORIGIN(APP_FLASH) + LENGTH(APP_FLASH));
  } >APP_FLASH

  /* App staging slot */
  .staging_area (NOLOAD) :
  {
    . = ALIGN(_Flash_Page_Size);
    PROVIDE_HIDDEN (__start_app_rollback_area = .);
    PROVIDE_HIDDEN (__start_aim_rollback_slot_header = . + _Magic_Word_Pointer_Offset);
    PROVIDE_HIDDEN (__end_app_rollback_area = ORIGIN(STAGING_FLASH) + LENGTH(STAGING_FLASH));
    . = . + _Flash_Page_Size;
    PROVIDE_HIDDEN (__start_app_staging_area = .);
    PROVIDE_HIDDEN (__start_aim_staging_slot_header = . + _Magic_Word_Pointer_Offset);
    PROVIDE_HIDDEN (__end_app_staging_area = ORIGIN(STAGING_FLASH) + LENGTH(STAGING_FLASH));
  } >STAGING_FLASH

  PROVIDE_HIDDEN (__total_flash_memory_size = LENGTH(AIM_FLASH) + LENGTH(OTA_STATUS_FLASH) + LENGTH(APP_FLASH) + LENGTH(STAGING_FLASH) + LENGTH(NVM_STORAGE));

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

