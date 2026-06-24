/* startup_bl.s — Cortex-M3 absolute minimum startup for STM32F103 bootloader
 * No IRQ handlers, no FPU, no SystemInit — runs at 8MHz HSI straight after reset.
 * Stack placed at end of 4KB RAM (0x20001000).
 */

    .syntax unified
    .cpu cortex-m3
    .thumb

/* -----------------------------------------------------------------------
 * Vector table — placed at 0x08000000 by the linker script (.isr_vector)
 * Only Initial_SP and Reset_Handler are real; everything else is 0.
 * ----------------------------------------------------------------------- */
    .section .isr_vector, "a", %progbits
    .type  _vectors, %object
_vectors:
    .word  _estack          /* Initial stack pointer (top of 4KB RAM)      */
    .word  Reset_Handler    /* Reset                                        */
    .word  0                /* NMI                                          */
    .word  0                /* HardFault                                    */
    .word  0                /* MemManage                                    */
    .word  0                /* BusFault                                     */
    .word  0                /* UsageFault                                   */
    .word  0                /* Reserved                                     */
    .word  0                /* Reserved                                     */
    .word  0                /* Reserved                                     */
    .word  0                /* Reserved                                     */
    .word  0                /* SVCall                                       */
    .word  0                /* DebugMon                                     */
    .word  0                /* Reserved                                     */
    .word  0                /* PendSV                                       */
    .word  0                /* SysTick                                      */
    /* No peripheral IRQ handlers — bootloader runs with interrupts off */
    .size  _vectors, . - _vectors

/* -----------------------------------------------------------------------
 * Reset_Handler
 * ----------------------------------------------------------------------- */
    .section .text.Reset_Handler, "ax", %progbits
    .type  Reset_Handler, %function
    .global Reset_Handler
Reset_Handler:
    /* Disable all interrupts at the core level just in case */
    cpsid  i

    /* Zero .bss */
    ldr    r0, =_sbss
    ldr    r1, =_ebss
    movs   r2, #0
bss_loop:
    cmp    r0, r1
    bge    bss_done
    str    r2, [r0], #4
    b      bss_loop
bss_done:

    /* Copy .data from flash (LMA) to RAM (VMA) */
    ldr    r0, =_sdata        /* destination in RAM  */
    ldr    r1, =_edata
    ldr    r2, =_ldata        /* source in flash     */
data_loop:
    cmp    r0, r1
    bge    data_done
    ldr    r3, [r2], #4
    str    r3, [r0], #4
    b      data_loop
data_done:

    /* Call main() — never returns in normal operation */
    bl     main

    /* Safety: spin if main ever returns */
spin:
    b      spin

    .size  Reset_Handler, . - Reset_Handler
