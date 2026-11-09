/* Cortex-M3 size-first AEABI zeroing helper: r0=destination, r1=byte count.
 * Byte stores support arbitrary alignment and do not touch adjacent bytes.
 * Zero length makes no memory access. AEABI specifies no return value;
 * only caller-saved r0/r1/r2 and flags are clobbered; no stack is used.
 */
.syntax unified
.thumb
.section .text.__aeabi_memclr,"ax",%progbits
.p2align 1
.global __aeabi_memclr
.global __aeabi_memclr4
.global __aeabi_memclr8
.type __aeabi_memclr,%function
.thumb_func
__aeabi_memclr:
    movs r2, #0
    cbz r1, 2f
1:
    strb r2, [r0], #1
    subs r1, r1, #1
    bne 1b
2:
    bx lr
.size __aeabi_memclr, .-__aeabi_memclr
.thumb_set __aeabi_memclr4, __aeabi_memclr
.thumb_set __aeabi_memclr8, __aeabi_memclr
