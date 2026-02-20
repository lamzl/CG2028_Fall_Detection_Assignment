/*
 * mov_avg.s
 *
 * Created on: 2/2/2026
 * Author: Hitesh B, Hou Linxin
 */
.syntax unified
 .cpu cortex-m4
 .thumb
 .global mov_avg
 .equ N_MAX, 8
 .bss
 .align 4

 .text
 .align 2
@ CG2028 Assignment, Sem 2, AY 2025/26
@ (c) ECE NUS, 2025
@ Write Student 1’s Name here: ABCD (A1234567R)
@ Write Student 2’s Name here: WXYZ (A0000007X)
@ You could create a look-up table of registers here:
@ R0 ...
@ R1 ...
@ write your program from here:



mov_avg:
 PUSH {r2-r11, lr}

	@ Inputs:
	@ R0 = N (N == 4 as defined in the main function to hold the size of the buffer)
	@ R1 = pointer to an int buffer containing the most recent samples.

	@ Outputs:
	@ R0 = Integer Average of the N samples

	@ Other registers:
	@ R0-R4 @ R5-R12 need to push the item on the stack first before returning

	@ 1. Setup Phase
	@ R2 will act as a running sum initialised to zero
	@ R3 will be the loop counter (you need to initialise with the value of N from R0)
	MOV R2, #0
 	MOV R3, R0

 	@ 2. Loop phase
 	@ Load the integer from the pointer from R1 to R4
 	@ increment the R1 to point to the next integer
 	@ then slowly increase the number after iterating the running sum
 	@ loop counter decrement with flag status on
 	@ then sign divide the running counter by N
 LOOP:
 	LDR R4, [R1]
 	ADD R1, R1, #4
 	ADD R2, R2, R4

 	SUBS R3, R3, #1 @ wait for N flag
 	BNE LOOP
 	SDIV R0, R2, R0
 	POP {r2-r11, pc}
