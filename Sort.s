@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@	Sort.s                                                                     @
@ -------------------------------------------------------------------------------- @
@	Author: Niklas Haderer, Pascal Kelbling, Christoph Hund, Moritz Fluechter  @
@	Target: Raspberri Pi, Raspbian						   @
@	Project: MM-Sorting-Machine						   @
@	Date: 27.02.2019							   @
@	Version: 0.01								   @
@ -------------------------------------------------------------------------------  @
@ This program controls the MM-Sorting-Machine, reading multiple input sensors     @
@ and controlling the motors accordingly.					   @
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

@ Constants for assembler
@ The following are defined in /usr/include/asm-generic/fcntl.h:
@ Note that the values are specified in octal.
        .equ      O_RDWR,00000002             @ open for read/write
        .equ      O_DSYNC,00010000            @ synchronize virtual memory
        .equ      __O_SYNC,04000000           @      programming changes with
        .equ      O_SYNC,__O_SYNC|O_DSYNC     @ I/O memory
@ The following are defined in /usr/include/asm-generic/mman-common.h:
        .equ      PROT_READ,0x1               @ page can be read
        .equ      PROT_WRITE,0x2              @ page can be written
        .equ      MAP_SHARED,0x01             @ share changes
@ The following are defined by me:
@        .equ      PERIPH,0x3f000000           @ RPi 2 & 3 peripherals
        .equ      PERIPH,0x20000000           @ RPi zero & 1 peripherals
        .equ      GPIO_OFFSET,0x200000        @ start of GPIO device
        .equ      TIMERIR_OFFSET,0xB000       @ start f´of IR and timer
        .equ      O_FLAGS,O_RDWR|O_SYNC       @ open file flags
        .equ      PROT_RDWR,PROT_READ|PROT_WRITE
        .equ      NO_PREF,0
        .equ      PAGE_SIZE,4096              @ Raspbian memory page
        .equ      FILE_DESCRP_ARG,0           @ file descriptor
        .equ      DEVICE_ARG,4                @ device address
        .equ      STACK_ARGS,8                @ sp already 8-byte aligned

TMPREG  .req      r5
RETREG  .req      r6
WAITREG .req      r8
RLDREG  .req      r9
GPIOREG .req      r10
COLREG  .req      r11



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		.data
gpiomem:
        .asciz    "/dev/gpiomem"
mem:
        .asciz    "/dev/mem"
fdMsg:
        .asciz    "File descriptor = %i\n"
memMsgGpio:
        .asciz    "(GPIO) Using memory at %p\n"
memMsgTimerIR:
        .asciz    "(Timer + IR) Using memory at %p\n"

        .balign   4
gpio_mmap_adr:
        .word     0               @ ...
gpio_mmap_fd:
        .word     0
timerir_mmap_adr:
        .word     0
timerir_mmap_fd:
        .word     0

@ - END OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF TEXT SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	.text
	@ externals for making use of std-functions
        .extern printf

        .balign   4
        .global   main
        .type     main, %function


@ -----------------------------------------------------------------------------
@ main entry point of the application
@   param:     none
@   return:    none
@ -----------------------------------------------------------------------------
main:
        ldr r0, =IntroMsg
        bl  printf

        @ GET GPIO VIRTUAL MEMORY ---------------------------------------------
        @ create backup and reserve stack space
        sub       sp, sp, #16                 @ space for saving regs
        str       r4, [sp, #0]                @ save r4
        str       r5, [sp, #4]                @      r5
        str       fp, [sp, #8]                @      fp
        str       lr, [sp, #12]               @      lr
        add       fp, sp, #12                 @ set our frame pointer
        sub       sp, sp, #STACK_ARGS         @ sp on 8-byte boundary

        @ open /dev/gpiomem for read/write and syncing
        ldr       r0, =gpiomem                 @ address of /dev/gpiomem
        ldr       r1, openMode                @ flags for accessing device
        bl        open
        mov       r4, r0                      @ use r4 for file descriptor

        @ display file descriptor
        ldr       r0, =fdMsg                  @ format for printf
        mov       r1, r4                      @ file descriptor
        bl        printf

        @ map the GPIO registers to a virtual memory location so we can access them
        str       r4, [sp, #FILE_DESCRP_ARG]  @ /dev/gpiomem file descriptor
        ldr       r0, gpio                    @ address of GPIO
        str       r0, [sp, #DEVICE_ARG]       @ location of GPIO
        mov       r0, #NO_PREF                @ let kernel pick memory
        mov       r1, #PAGE_SIZE              @ get 1 page of memory
        mov       r2, #PROT_RDWR              @ read/write this memory
        mov       r3, #MAP_SHARED             @ share with other processes
        bl        mmap

        @ save virtual memory address
        ldr       r1, =gpio_mmap_adr          @ store gpio mmap (virtual address)
        str       r0, [r1]
        ldr       r1, =gpio_mmap_fd           @ store the file descriptor
        str       r4, [r1]

        ldr       r6, [r1]
        mov       r1, r0                      @ display virtual address
        ldr       r0, =memMsgGpio
        bl        printf
        mov       r1, r6
        ldr       r0, =memMsgGpio
        bl        printf

        @ restore sp and free stack
        add       sp, sp, #STACK_ARGS         @ fix sp
        ldr       r4, [sp, #0]                @ restore r4
        ldr       r5, [sp, #4]                @      r5
        ldr       fp, [sp, #8]                @         fp
        ldr       lr, [sp, #12]               @         lr
        add       sp, sp, #16                 @ restore sp

        @ GET TIMER + IR VIRTUAL MEMORY ---------------------------------------
        @ create backup and reserve stack space
        sub       sp, sp, #16                 @ space for saving regs
        str       r4, [sp, #0]                @ save r4
        str       r5, [sp, #4]                @      r5
        str       fp, [sp, #8]                @      fp
        str       lr, [sp, #12]               @      lr
        add       fp, sp, #12                 @ set our frame pointer
        sub       sp, sp, #STACK_ARGS         @ sp on 8-byte boundary

        @ open /dev/gpiomem for read/write and syncing
        ldr       r0, =mem                    @ address of /dev/mem
        ldr       r1, openMode                @ flags for accessing device
        bl        open
        mov       r4, r0                      @ use r4 for file descriptor

        @ display file descriptor
        ldr       r0, =fdMsg                  @ format for printf
        mov       r1, r4                      @ file descriptor
        bl        printf

        @ map the GPIO registers to a virtual memory location so we can access them
        str       r4, [sp, #FILE_DESCRP_ARG]  @ /dev/mem file descriptor
        ldr       r0, timerIR                 @ address of timer + IR
        str       r0, [sp, #DEVICE_ARG]       @ location of timer +IR
        mov       r0, #NO_PREF                @ let kernel pick memory
        mov       r1, #PAGE_SIZE              @ get 1 page of memory
        mov       r2, #PROT_RDWR              @ read/write this memory
        mov       r3, #MAP_SHARED             @ share with other processes
        bl        mmap

        @ save virtual memory address
        ldr       r1, =timerir_mmap_adr       @ store timer + IR mmap (virtual address)
        str       r0, [r1]
        ldr       r1, =timerir_mmap_fd        @ store the file descriptor
        str       r4, [r1]

        ldr       r6, [r1]
        mov       r1, r0                      @ display virtual address
        ldr       r0, =memMsgTimerIR
        bl        printf
        mov       r1, r6
        ldr       r0, =memMsgTimerIR
        bl        printf

        @ restore sp and free stack
        add       sp, sp, #STACK_ARGS         @ fix sp
        ldr       r4, [sp, #0]                @ restore r4
        ldr       r5, [sp, #4]                @      r5
        ldr       fp, [sp, #8]                @         fp
        ldr       lr, [sp, #12]               @         lr
        add       sp, sp, #16                 @ restore sp

        @ initialize all other hardware
        b         hw_init


@ --------------------------------------------------------------------
@ Inititalize all needed GPIOs (see above), Set Input/Output mode, 
@ Set starting values of some Pins (11, 17)
@  param: none
@  return: none
@ -------------------------------------------------------------------
hw_init:

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@	Table of all needed GPIOs for init in _start:
@
@	GPIO	|	Signal		|	Hardware	
@	--------------------------------------------------------------
@	Signals to control 7-Segment Display:
@	2	|	Output		|	SER	7-Segment
@	3	|	Output		|	SRCLK	7-Segment
@	4	|	Output		|	nSRCLR  7-Segment
@	5	|	Output		|	RCLK	7-Segment
@	6	|	Output		|	A	7-Segment
@	7	|	Output		|	B	7-Segment
@	--------------------------------------------------------------
@	Input from Buttons:
@	8	|	Input		|	nBTN1	Taster
@	9	|	Input		|	nBTN2	Taster
@	10	|	Input		|	nBTN3	Taster
@	--------------------------------------------------------------
@	nRST of Outlet + Step with either Outlet or Color-Wheel:
@	11	|	Output		|	nRSTOut	Outlet
@	12	|	Output		|	StepOut	Outlet
@	13	|	Output		|	StepCW	Color-WHeel
@	--------------------------------------------------------------
@	Serial Communication:
@	14	|	TX		|	UART_TX	Serielle Com.
@	15	|	RX		|	UART_RX	Serielle Com.
@	--------------------------------------------------------------
@	Direction Control + RST of Color-Wheel:
@	16	|	Output		|	DirCW	Color-Wheel
@	17	|	Output		|	nRSTCW	Color-Wheel
@	--------------------------------------------------------------
@	Sorting LED control:
@	18	|	Output		|	ledSig	ColorLEDs
@	--------------------------------------------------------------
@	Start/Stop Feeder:
@	19	|	Output		|	GoStop	Feeder
@	--------------------------------------------------------------
@	Hallsensor to detect CW and Out position:
@	20	|	Input		|	nHallCW	Hallsensor ColorWheel
@	21	|	Input		|	nHallOut Hallensor Outlet
@	--------------------------------------------------------------
@	Output of Color Detection Sensor:
@	22	|	Input		|	colorBit0 Farberkennung
@	23	|	Input		|	colorBit1 Farberkennung
@	24	|	Input		|	colorBit2 Farberkennung
@	--------------------------------------------------------------
@	Objectsensor of CW (MM didnt fall) and Outlet Direction:
@	25	|	Input		|	objCW	  Objectsensor CW
@	26	|	Output		|	DirOut	  Outlet
@	--------------------------------------------------------------
@	Send/Dont Send Sleep Signal to Driver:
@	27	|	Output		|	nSLP	  Sleep Signal CoProzessor
@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

@ --------------------------------------------------------------------
@ Move Outlet Motor to starting position. The Hall sensor only returns 
@ true/false, so in order to find the center the motor turns until true
@ starts counting its steps, turns until false and then turns back half
@ of the steps counted to center itself
@  param: none
@  return: none
@ --------------------------------------------------------------------
startpos_outlet:

@ --------------------------------------------------------------------
@ Move Color-Wheel to starting position, same principle as with Outlet
@  param: none
@  return: none
@ --------------------------------------------------------------------
startpos_cw:

@ --------------------------------------------------------------------
@ Main Loop to control the sorting process
@  param: none
@  return: none
@ --------------------------------------------------------------------

@ --------------------------------------------------------------------------------------------------------------------
@
@ ADDRESSES: Further definitions.
@
@ --------------------------------------------------------------------------------------------------------------------
        .balign   4
@ addresses of messages
openMode:
        .word     O_FLAGS
gpio:
        .word     PERIPH+GPIO_OFFSET
timerIR:
        .word     PERIPH+TIMERIR_OFFSET


@ - END OF TEXT SECTION   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

