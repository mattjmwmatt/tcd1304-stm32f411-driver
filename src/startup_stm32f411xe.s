/* Minimal startup file for STM32F411xE */
    .syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

    .global g_pfnVectors
    .global Default_Handler

    .word  _estack
    .word  Reset_Handler
    .word  NMI_Handler
    .word  HardFault_Handler
    .word  MemManage_Handler
    .word  BusFault_Handler
    .word  UsageFault_Handler
    .word  0
    .word  0
    .word  0
    .word  0
    .word  SVC_Handler
    .word  DebugMon_Handler
    .word  0
    .word  PendSV_Handler
    .word  SysTick_Handler
    .rept 24
    .word Default_Handler
    .endr
    .word TIM2_IRQHandler
    .rept 27
    .word Default_Handler
    .endr
    .word DMA2_Stream0_IRQHandler
    .rept 20
    .word Default_Handler
    .endr

    .section .text.Reset_Handler
    .weak  Reset_Handler
    .type  Reset_Handler, %function
Reset_Handler:
    ldr   sp, =_estack
    ldr   r0, =_sdata
    ldr   r1, =_edata
    ldr   r2, =_etext
    movs  r3, #0
    b     LoopCopyData
CopyData:
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, r3, #4
LoopCopyData:
    adds  r4, r0, r3
    cmp   r4, r1
    bcc   CopyData
    ldr   r2, =_sbss
    ldr   r4, =_ebss
    movs  r3, #0
    b     LoopFillZero
FillZero:
    str   r3, [r2]
    adds  r2, r2, #4
LoopFillZero:
    cmp   r2, r4
    bcc   FillZero
    bl    main
LoopForever:
    b     LoopForever

    .weak      Default_Handler
    .type      Default_Handler, %function
Default_Handler:
    b Default_Handler

    .macro WEAK_IRQ name
    .weak \name
    .set  \name, Default_Handler
    .endm

    WEAK_IRQ NMI_Handler
    WEAK_IRQ HardFault_Handler
    WEAK_IRQ MemManage_Handler
    WEAK_IRQ BusFault_Handler
    WEAK_IRQ UsageFault_Handler
    WEAK_IRQ SVC_Handler
    WEAK_IRQ DebugMon_Handler
    WEAK_IRQ PendSV_Handler
    WEAK_IRQ SysTick_Handler
    WEAK_IRQ TIM2_IRQHandler
    WEAK_IRQ DMA2_Stream0_IRQHandler
