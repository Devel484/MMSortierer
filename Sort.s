@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Sort.s                                                                           @
@ -------------------------------------------------------------------------------- @
@ Author:   Niclas Haderer, Pascal Kelbling, Christoph Hund, Moritz Fluechter      @
@ Target:   Raspberri Pi, Raspbian                                                 @
@ Project:  MM-Sorting-Machine                                                     @
@ Date:     27.02.2019                                                             @
@ Version:  0.01                                                                   @
@ -------------------------------------------------------------------------------  @
@ This program controls the MM-Sorting-Machine, reading multiple input sensors     @
@ and controlling the motors accordingly.                                          @
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
@        .equ      PERIPH,0x3f000000          @ RPi 2 & 3 peripherals
        .equ      PERIPH,0x20000000           @ RPi zero & 1 peripherals
        .equ      GPIO_OFFSET,0x200000        @ start of GPIO device
        .equ      TIMERIR_OFFSET,0xB000       @ start f´of IR and timer
        .equ    SYSTIMER_OFFSET,0x3000        @ start of sys timer lower 32 bit
        .equ      O_FLAGS,O_RDWR|O_SYNC       @ open file flags
        .equ      PROT_RDWR,PROT_READ|PROT_WRITE
        .equ      NO_PREF,0
        .equ      PAGE_SIZE,4096              @ Raspbian memory page
        .equ      FILE_DESCRP_ARG,0           @ file descriptor
        .equ      DEVICE_ARG,4                @ device address
        .equ      STACK_ARGS,8                @ sp already 8-byte aligned
@ Center CW and Outlet Constants:
        .equ      CENTER_CWSPEED,10           @ wait in ms for centering CW
        .equ      CENTER_OUTSPEED,20          @ wait in ms for centering OUT
@ Constants for LEDs
        .equ      LED_OFFSET,0                @ Offset between outlet starting pos and red LED
        .equ      COLOR_RED,0xFF0000          @ RGB code for red
        .equ      COLOR_GREEN,0x00FF00        @ RGB code for green
        .equ      COLOR_BLUE,0x0000FF         @ RGB code for blue
        .equ      COLOR_BROWN,0x663300        @ RGB code for brown
        .equ      COLOR_ORANGE,0xFF9900       @ RGB code for orange
        .equ      COLOR_YELLOW,0xFFFF66       @ RGB code for yellow

@ Pin names:
        .equ      nBTN1,8                     @ Button 1
        .equ      nRSTOut,11                  @ toggle engine Outlet
        .equ      StepOut,12                  @ engine step Outlet
        .equ      StepCW,13                   @ engine step ColorWheel
        .equ      DirCW,16                    @ direction of ColorWheel
        .equ      nRSTCW,17                   @ toggle engine ColorWheel
        .equ      GoStop,19                   @ toggle feeder (subprocess)
        .equ      nHallCW,20                  @ hallsensor of ColorWheel
        .equ      nHallOutlet,21              @ hallsensor of Outlet
        .equ      colorBit0,22                @ colorBit 0
        .equ      colorBit1,23                @ colorBit 1
        .equ      colorBit2,24                @ colorBit 2
        .equ      objCW,25                    @ objectsensor of ColorWheel
        .equ      DirOut,26                   @ direction of Outlet
        .equ      nSLP,27                     @ toggle co-processor for engines

@ Register names:
POSREG  .req      r4
TMPREG  .req      r5
RETREG  .req      r6
WAITREG .req      r8
RLDREG  .req      r9
GPIOREG .req      r10
FLAGREG .req      r11



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
memMsgSysTimer:
        .asciz    "(SysTimer) Using memory at %p\n"
msgSysTimerDebug:
        .asciz    "(SysTimer) lower 32 bit systimer %i\n"
msgSysTimerWait:
        .asciz    "(SysTimer) wait %i us\n"
msgSysTimerSpread:
        .asciz    "(SysTimer) spread %i us\n"
IntroMsg:
        .asciz    "Hallo Christoph"


        .balign   4
gpio_mmap_adr:
        .word     0               @ ...
gpio_mmap_fd:
        .word     0
timerir_mmap_adr:
        .word     0
timerir_mmap_fd:
        .word     0
systimer_mmap_adr:
        .word     0
systimer_mmap_fd:
        .word     0

@ - END OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF TEXT SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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

        @ GET SYSTIMER VIRTUAL MEMORY ---------------------------------------
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

        @ map the lower 32bit sys timer to a virtual memory location so we can access them
        str       r4, [sp, #FILE_DESCRP_ARG]  @ /dev/mem file descriptor
        ldr       r0, sysTimer                @ address of lower 32 bit systimer
        str       r0, [sp, #DEVICE_ARG]       @ location of lower 32 bit systimer
        mov       r0, #NO_PREF                @ let kernel pick memory
        mov       r1, #PAGE_SIZE              @ get 1 page of memory
        mov       r2, #PROT_RDWR              @ read/write this memory
        mov       r3, #MAP_SHARED             @ share with other processes
        bl        mmap

        @ save virtual memory address
        ldr       r1, =systimer_mmap_adr      @ store timer + IR mmap (virtual address)
        str       r0, [r1]
        ldr       r1, =systimer_mmap_fd       @ store the file descriptor
        str       r4, [r1]

        ldr       r6, [r1]
        mov       r1, r0                      @ display virtual address
        ldr       r0, =memMsgSysTimer
        bl        printf
        mov       r1, r6
        ldr       r0, =memMsgSysTimer
        bl        printf

        @ restore sp and free stack
        add       sp, sp, #STACK_ARGS         @ fix sp
        ldr       r4, [sp, #0]                @ restore r4
        ldr       r5, [sp, #4]                @         r5
        ldr       fp, [sp, #8]                @         fp
        ldr       lr, [sp, #12]               @         lr
        add       sp, sp, #16                 @ restore sp



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@       main program                                @
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
sort:
        bl        hw_init

        @ start config for outlet and LEDs ---------------------------
        mov       POSREG,#1             @ make sure starting pos is 1
        push      {GPIOREG}
        bl        WS2812RPi_Init        @ initialise LEDs

        mov       r0,#100               @ Brightness for LEDs (0-100)
        bl        WS2812RPi_SetBrightness @ set Brightness for all LEDs
        bl        WS2812RPi_Show        @ Apply Brightness change
        pop       {GPIOREG}
        @ end of cfg ------------------------------------------------

        bl        startpos_out_init     @ outlet position init
        bl        startpos_cw_init      @ color wheel position init

        mov       r0, #GoStop           @ select Feeder StartStop pin
        bl        gp_set                @ start Feeder

        mov       WAITREG, #4000        @ wait 4s
        bl        wait

        mov       FLAGREG, #1           @ SET FLAG -> Secure at least one round

        @ if bedingungen
        @ turn cw wheel
        @ wait
        @ in / decrease (optional)
        @ outlet + lds
        @ b if bedingungen

check_flag:
        cmp       FLAGREG, #1
        bne       check_object_sensor     @ Flag = false

        mov       FLAGREG, #0             @ Flag = true -> UNSET
        b         start_process

check_object_sensor:
        mov       r0,#objCW               @ object sensor cw pin number
        bl        gp_read                 @ get object sensor value
        cmp       RLDREG, #1              @ Check if sensor value true
        bne       check_color_sensor      @ object sensor = false

        mov       FLAGREG, #1             @ object sensor = true -> SET FLAG
        b         start_process

check_color_sensor:
        bl        get_color               @ read color sensor
        cmp       RLDREG, #0
        beq       stop                    @ no M&M in color position

start_process:
        bl        turn_cw
        mov       WAITREG, #1000          @ Wait 1 s
        bl        wait                    @ Start wait

        mov       r0, RLDREG              @ Set param(color/position) for TURN OUTLET

        bl        set_outlet

        b         check_flag              @ back to the beginning

stop:
        mov       r0, #GoStop             @ select Feeder StartStop pin
        bl        gp_clear                @ stop Feeder

        mov       R7, #1
        svc       0

@ --------------------------------------------------------------------
@  Turns on the LED at the desired position. The Pos number is the 
@  same for LED and Outlet
@
@       
@       Pos     | Color
@       ----------------
@       1       | RED
@       2       | GREEN
@       3       | BLUE
@       4       | BROWN
@       5       | ORANGE
@       6       | YELLOW
@
@  param : r0 - LED pos (starting under Hall sensor)
@  return: none
@ --------------------------------------------------------------------
set_LED:
        puh     {lr}                    @ save lr
        mov     r1,#COLOR_RED           @ store RED color RGB val to pass
        cmp     r0,#1                   @ if position passed in r0 = 1
        beq     offset_LED              @ the color needed is red

        mov     r1,#COLOR_GREEN         @ store GREEN color RGB val to pass
        cmp     r0,#2                   @ if position passed in r0 = 2
        beq     offset_LED              @ the color needed is green

        mov     r1,#COLOR_BLUE          @ store BLUE color RGB val to pass
        cmp     r0,#3                   @ if position passed in r0 = 3
        beq     offset_LED              @ the color needed is blue

        mov     r1,#COLOR_BROWN         @ store BROWN color RGB val to pass
        cmp     r0,#4                   @ if position passed in r0 = 4
        beq     offset_LED              @ the color needed is brown

        mov     r1,#COLOR_ORANGE        @ store ORANGE color RGB val to pass
        cmp     r0,#5                   @ if position passed in r0 = 5
        beq     offset_LED              @ the color needed is orange

        mov     r1,#COLOR_YELLOW        @ store YELLOW color RGB val to pass
        cmp     r0,#6                   @ if position passed in r0 = 6
        b       offset_LED              @ the color needed is yellow

offset_LED:
        @ Hint: The Starting pos for LEDs (1) is the same as the starting pos
        @       of the outlet. But Because that may not be the LED with the 
        @       predefined number used in the WS2812RPi library, the offset
        @       corrects that by adding the offset and subtracting 6 if the 
        @       resulting value is >6 to continue the ring system
        add     r0,#LED_OFFSET          
        cmp     r0,#6                   @ check if out of ring
        bgt     set_LED_edge            @ correct pos value
        b       light_LED               @ continue with lighting LED

set_LED_edge:
        sub     r0,r0,#6                @ %6 is needed to stay inside ring
                                        @ however pos + offset cant be >11
                                        @ (offset max 5)

light_LED:
        @ r0 now contains the position with offset
        @ and r1 the needed color
        push    {GPIOREG}               @ GPIOREG may be altered by function
        push    {r0}                    @ r0 may be altered by function
        bl      WS2812RPi_SetSingle     @ Set which Led to light
        pop     {r0}                    @ retrieve LED pos for next call
        bl      WS2812RPi_SetOthersOff  @ Set other LEDs off

        bl      WS2812RPi_Show          @ Light LEDs
        pop     {GPIOREG}               @ restore GPIOREG
        pop     {lr}                    @ restore lr
        bx      lr                      @ return to caller

@ --------------------------------------------------------------------
@ Get Color from Color Sensor, then light LED first and Turn Outlet in the needed
@ direction.
@ Idea:
@       The postion number is a ring system from 1 to 6. These are translated into
@       positions adding 1 in clockwise direction.
@       -> 6 + 1 = 1 and 1 - 1 = 6
@       
@  param : none
@  return: none
@ --------------------------------------------------------------------
set_outlet:
        push    {lr}                    @ save link register
        bl      get_color               @ fetch current color at hall sensor

        cmp     RLDREG,#0               @ check if no color read
        beq     decision_equal          @ dont move outlet

        mov     r0,RLDREG               @ pass color val 
        bl      set_LED                 @ set led based on color

        cmp     RLDREG,POSREG           @ check if outlet already at pos
        beq     decision_equal          @ dont move outlet

        mov     r1,POSREG               @ Save starting position to manipulate
        bl      count_clockwise         @ count steps clockwise to color pos
        mov     TMPREG,r2               @ save result
        
        mov     r1,POSREG               @ Save starting position to manipulate
        bl      count_counterclock      @ count steps ccw to color pos

        cmp     TMPREG,r2               @ compare TMPREG: steps clockwise
                                        @ and     r2    : steps counterclockwise
        mov     r0,TMPREG               @ save steps clockwise
        blt     decision_clockwise      @ less steps clockwise

        mov     r0,r2                   @ save steps counterclockwise
        bge     decision_counterclock   @ less steps counterclockwise
                                        @ if same steps pick ccw by default


decision_clockwise:
        mov     r3,r0                   @ save steps needed to r3

        mov     r0,#DirOut              @ set pin number for DirOut
        bl      gp_set                  @ set direction of out to cw

decision_clockwise_step:
        bl      turn_out                @ turn outlet, r3 will always be gt 0
        sub     r3,r3,#1                @ decrement step counter
        
        cmp     r3,#0                   @ check if reached destination
        bne     decision_clockwise_step @ continue turning
        mov     POSREG,RLDREG           @ set position to destination
        pop     {lr}                    @ restore lr
        bx      lr                      @ return to caller    


decision_counterclock:
        mov     r3,r0                   @ save steps to r3

        mov     r0,#DirOut              @ pin of DirOut
        bl      gp_clear                @ set Direction to ccw

decision_counterclock_step:
        bl      turn_out                @ turn outlet
        sub     r3,r3,#1                @ decrement step counter

        cmp     r3,#0                   @ check if reached destination
        bne     decision_counterclock_step @ continue turning
        mov     POSREG,RLDREG           @ set position to destination
        pop     {lr}                    @ restore lr
        bx      lr                      @ return to caller

decision_equal:
        pop     {lr}                    @ restore lr
        bx      lr                      @ return to caller


count_clockwise:
        cmp     r1,#6                  @ if = 5: edge reached
        beq     clockwise_edge         @ set pos to 0 intsead of adding 1
                                       @ if not 5: continue incrementing

        add     r1,r1,#1               @ set position +1 step cw
        add     r2,r2,#1               @ increment step counter

        cmp     r1,RLDREG              @ if not at destination:
        bne     count_clockwise        @ repeat cycle
        bx      lr                     @ return steps counted


clockwise_edge: 
        mov     r1,#1                   @ set to min value in ring
        add     r2,r2,#1                @ increment step counter

        cmp     r1,RLDREG               @ check if reached destination
        bne     count_clockwise         @ if not: continue counting
        bx      lr                      @ else: return to caller

count_counterclock:
        
        cmp     r1,#1                  @ if = 1: edge reached
        beq     counterclock_edge      @ set pos to 6 intsead of subtracting 1
                                       @ if not 1: continue incrementing

        sub     r1,r1,#1               @ set position +1 step ccw
        add     r2,r2,#1               @ increment step counter

        cmp     r1,RLDREG              @ if not at desired pos:
        bne     count_counterclock     @ repeat cycle
        bx      lr                     @ return steps counted

counterclock_edge:
        mov     r1,#6                  @ set r1 to max number in ring
        add     r2,r2,#1               @ increment step counter

        cmp     r1,RLDREG              @ check if destination reached
        bne     count_counterclock     @ if not continue counting
        bx      lr                     @ else: return to caller

@ --------------------------------------------------------------------
@ Move Outlet engine to starting position. The hall sensor only returns
@ true/false, so in order to find the center the motor turns until true
@ starts counting its steps, turns until false and then turns back half
@ of the steps counted to center itself.
@  param : none
@  return: none
@ --------------------------------------------------------------------
startpos_out_init:
        push      {lr}
        mov       r0,#DirOut              @ number of Outlet Dir pin
        bl        gp_set                  @ Set Outlet motor direction

startpos_out:
        @ Idea:
        @ Move until Hall sensor activates.
        @ Once activates count the steps until it deactivates.
        @ Go back half the amount of steps (right shift numer of steps)

        mov       r0,#1                   @ param: turn 1 step
        mov       r1,#CENTER_OUTSPEED     @ param: wait 20ms
        bl        turn_out_step           @ turn outlet

        mov       r0,#nHallOutlet         @ hall sensor pin number
        bl        gp_read                 @ get hall sensor value
        cmp       RLDREG,#0               @ if != 0: out of FoV, repeat
                                          @ if = 0: inside FoV, start counting
        mov       r1,#0                   @ reset counter r1
        bne       startpos_out            @ start counting steps

        mov       r0,#DirOut              @ number of Outlet Dir pin
        bl        gp_clear                @ invert outlet turn direction

startpos_out_inside:
        mov       r0,#1                   @ param: turn 1 step
        mov       r1,#CENTER_OUTSPEED     @ param: wait 20ms
        bl        turn_out_step           @ turn outlet

        add       r1,#1                   @ increment counter
        mov       r0,#nHallOutlet         @ hall sensor pin number
        bl        gp_read                 @ check hall sensor, r0 unchanged

        cmp       RLDREG,#0               @ if = 0: inside FoV, repeat
        beq       startpos_out_inside     @ if != 0: outside FoV, stop counting
        lsr       r1,#1                   @ divide steps counted by 2

startpos_out_center:
        mov       r0,#1                   @ param: turn 1 step
        mov       r1,#CENTER_OUTSPEED                  @ param: wait 20ms
        bl        turn_out_step           @ turn outlet

        sub       r1,#1                   @ reduce steps counter
        cmp       r1,#0                   @ if != 0: still moving, repeat
        bne       startpos_out_center     @ if  = 0: in center, stop

        pop       {lr}                    @ Jump back
        bx        lr



@ --------------------------------------------------------------------
@ Move Color-Wheel to starting position, same principle as with Outlet.
@  param : none
@  return: none
@ --------------------------------------------------------------------
startpos_cw_init:
        push      {lr}
        mov       r0,#DirCW             @ number of Outlet Dir pin
        bl        gp_set                @ Set Outlet motor direction

startpos_cw:
        @ Idea:
        @ Move until Hall sensor activates.
        @ Once activates count the steps until it deactivates.
        @ Go back half the amount of steps (right shift numer of steps)

        mov       r0,#1                   @ param: turn 1 step
        mov       r1,#CENTER_CWSPEED      @ param: wait 20ms
        bl        turn_cw_step            @ turn cw

        mov       r0,#nHallCW             @ hall sensor pin number
        bl        gp_read                 @ get hall sensor value
        cmp       RLDREG,#0               @ if != 0: out of FoV, repeat
                                          @ if = 0: inside FoV, start counting
        mov       r1,#0                   @ reset counter r1
        bne       startpos_cw             @ start counting steps

startpos_cw_inside:
        mov       r0,#1                   @ param: turn 1 step
        mov       r1,#CENTER_CWSPEED      @ param: wait 20ms
        bl        turn_cw_step            @ turn cw

        add       r1,#1                   @ increment counter
        mov       r0,#nHallCW             @ hall sensor pin number
        bl        gp_read                 @ check hall sensor, r0 unchanged
        cmp       RLDREG,#0               @ if = 0: inside FoV, repeat
        beq       startpos_cw_inside      @ if != 0: outside FoV, stop counting
        lsr       r1,#1                   @ divide steps counted by 2

        mov       r0,#DirCW               @ number of CW Dir pin
        bl        gp_clear                @ invert outlet turn direction

startpos_cw_center:
        mov       r0,#1                   @ param: turn 1 step
        mov       r1,#CENTER_CWSPEED      @ param: wait 20ms
        bl        turn_cw_step            @ turn outlet

        sub       r1,#1                   @ reduce steps counter
        cmp       r1,#0                   @ if != 0: still moving, repeat
        bne       startpos_cw_center      @ if  = 0: in center, stop

        pop       {lr}
        bx        lr


@ --------------------------------------------------------------------
@ Move Outlet with given parameters.
@  param : r0 = steps, r1 = wait ms
@  return: none
@ --------------------------------------------------------------------
turn_out_step:
        mov       TMPREG, r0
        push      {lr}
        mov       r0, #nSLP           @ activate co process
        bl        gp_set              @ call gp_set
        mov       r0, #nRSTOut        @ activate outlet engine
        bl        gp_set              @ call gp_set

turn_out_step_sub:
        mov       r0, #StepOut        @ set step high
        bl        gp_set              @ call gp_set
        mov       WAITREG, r1         @ Wait r1 ms
        bl        wait                @ Start wait

        mov       r0, #StepOut        @ set step high
        bl        gp_clear            @ set step low
        mov       WAITREG, r1         @ Wait r1 ms
        bl        wait                @ Start wait

        sub       TMPREG, TMPREG, #1  @ Decrease step counter
        cmp       TMPREG, #0          @ left steps == 0?
        bne       turn_out_step_sub   @ if not again

        mov       r0, #nRSTOut        @ deactivate outlet engine
        bl        gp_clear            @ clear output pin
        mov       r0, #nSLP           @ deactivate co processor
        bl        gp_clear            @ clear output pin
        pop       {lr}
        bx        lr                  @ close branch


@ --------------------------------------------------------------------
@ Move Outlet 67°.
@  param : none
@  return: none
@ --------------------------------------------------------------------
turn_out:
        mov       TMPREG, #67         @ do 67 steps ~ 60°
        push      {lr}
        mov       r0, #nSLP           @ activate co process
        bl        gp_set              @ call gp_set
        mov       r0, #nRSTOut        @ activate outlet engine
        bl        gp_set              @ call gp_set

turn_out_sub:
        mov       r0, #StepOut        @ set step high
        bl        gp_set              @ call gp_set
        mov       WAITREG, #5         @ Wait 5 ms
        bl        wait                @ Start wait
        mov       r0, #StepOut        @ set step high
        bl        gp_clear            @ set step low
        mov       WAITREG, #5         @ Wait 5 ms
        bl        wait                @ Start wait
        sub       TMPREG, TMPREG, #1  @ Decrease step counter
        cmp       TMPREG, #0          @ left steps == 0?
        bne       turn_out_sub        @ if not again

        mov       r0, #nRSTOut        @ deactivate outlet engine
        bl        gp_clear            @ clear output pin
        mov       r0, #nSLP           @ deactivate co processor
        bl        gp_clear            @ clear output pin
        pop       {lr}
        bx        lr                  @ close branch


@ --------------------------------------------------------------------
@ Move Color-Wheel with given parameters.
@  param : r0 = steps, r1 = wait_ms
@  return: none
@ --------------------------------------------------------------------
turn_cw_step:
        mov       TMPREG, r0          @ Store amount steps
        push      {lr}
        mov       r0, #nSLP           @ activate co process
        bl        gp_set              @ call gp_set
        mov       r0, #nRSTCW         @ activate color wheel engine
        bl        gp_set              @ call gp_set

turn_cw_step_sub:
        mov       r0, #StepCW         @ define pin
        bl        gp_set              @ set StepCW high
        mov       WAITREG, r1         @ Wait r1 ms
        bl        wait                @ Start wait
        mov       r0, #StepCW         @ define pin
        bl        gp_clear            @ set StepCW low
        mov       WAITREG, r1         @ Wait r1 ms
        bl        wait                @ Start wait
        sub       TMPREG, TMPREG, #1  @ Decrease step counter
        cmp       TMPREG, #0          @ left steps == 0?
        bne       turn_cw_step_sub    @ if not again

        mov       r0, #nRSTCW         @ deactivate color wheel engine
        bl        gp_clear            @ clear output pin
        mov       r0, #nSLP           @ deactivate co processor
        bl        gp_clear            @ clear output pin
        pop       {lr}
        bx        lr                  @ close branch

@ --------------------------------------------------------------------
@ Move Color-Wheel 90°.
@  param : none
@  return: none
@ --------------------------------------------------------------------
turn_cw:
        push      {lr}
        mov       TMPREG, #400        @ Store 400 steps
        mov       r0, #DirCW          @ rotate clockwise
        bl        gp_clear            @ call gp_clear
        mov       r0, #nSLP           @ activate co process
        bl        gp_set              @ call gp_set
        mov       r0, #nRSTCW         @ activate color wheel engine
        bl        gp_set              @ call gp_set

turn_cw_sub:
        mov       r0, #StepCW             @ set step high
        bl        gp_set                  @ call gp_set
        mov       WAITREG, #3             @ Wait 3 ms
        bl        wait                    @ Start wait
        mov       r0, #StepCW             @ set step high
        bl        gp_clear                @ set step low
        mov       WAITREG, #3             @ Wait 3 ms
        bl        wait                    @ Start wait
        sub       TMPREG, TMPREG, #1      @ Decrease step counter
        cmp       TMPREG, #0              @ left steps == 0?
        bne       turn_cw_sub             @ if not again

        mov       r0, #nRSTCW             @ deactivate color wheel engine
        bl        gp_clear                @ clear output pin
        mov       r0, #nSLP               @ deactivate co processor
        bl        gp_clear                @ clear output pin
        pop       {lr}
        bx        lr                      @ close branch


@ --------------------------------------------------------------------
@ Wait for the given time (ms). Count down 485378 steps for 1ms.
@  param : WAITREG in microseconds
@  return: none
@ --------------------------------------------------------------------
wait:
        mov       r0, #237
        lsl       r0, #11               @ Save 485376 in r0

wait_do:
        subs      r0, #1                @ Count down r0
        cmp       r0, #0
        bne       wait_do               @ Go again if r0 is not 0

        subs      WAITREG, #1           @ Count down WAITREG (given ms to wait)
        cmp       WAITREG, #0
        bne       wait                  @ Go again for another 1ms

        bx        lr                    @ close branch

@ --------------------------------------------------------------------
@ Fetch Color from Sensor. Will return pins 22, 23, 24 as one binary number.
@  0 | 0 | 0 <-> Bit2 | Bit1 | Bit0
@ The reult is a decimal number between 0 and 7, because this 0 and 7 are undefined,
@ the number can be read as a position for the outlet (1-6)
@  param : none
@  return: RLDREG - color position
@ --------------------------------------------------------------------
get_color:
        mov       r0,#colorBit2         @ first color Bit
        bl        gp_read               @ read pin23 level
        mov       r1,RLDREG             @ store return val to r1
        lsl       r1,#1                 @ make room for next pin
        mov       r0,#colorBit1         @ second Color Bit
        bl        gp_read               @ read pin23 level
        orr       r1,r1,RLDREG          @ store return val to rightmost bit
        lsl       r1,#1                 @ make room for next pin
        mov       r0,#colorBit0         @ last color Bit
        bl        gp_read               @ read pin24 level
        orr       r1,r1,RLDREG          @ store return val to rightmost bit
        mov       RLDREG, r1            @ return r1

        bx        lr                    @ close branch


@ --------------------------------------------------------------------
@  Set the pin with number saved in r0 to 1 (Pull Up)
@  param : r0 - PinNumber
@  return: none
@ --------------------------------------------------------------------
gp_set:
        mov       r1,#1                  @ prepare bitmask
        lsl       r1,r0                  @ shift 1 to position of pin passed in r0
        str       r1,[GPIOREG,#0x1C]     @ Write mask to GPSET0
        bx        lr                     @ return

@ --------------------------------------------------------------------
@  Clears the pin with number saved in r0 (Pull Down)
@  param : r0 - PinNumber
@  return: none
@ --------------------------------------------------------------------
gp_clear:
        mov       r1,#1                @ prepare bitmask
        lsl       r1,r0                @ shift to position of pin
        str       r1,[GPIOREG,#0x28]   @ Write to GPCLR0
        bx        lr                   @ return

@ --------------------------------------------------------------------
@  Reads the level at the pin number passed in r0. To do so it ands a bit
@       mask with the register (GPLEV0).
@  Example:
@   Selected Pin:    00001000        00001000
@   GPLEV0:      AND 10011001        10010001
@                  = 00001000        00000000
@   after rsl     -> 00000001        00000000
@
@  param : r0 - PinNumber
@  return: RLDREG - 0 or 1 (unset or set)
@ --------------------------------------------------------------------
gp_read:
        mov       r1,#1                 @ prepare bitmask
        lsl       r1,r0                 @ shift to position of pin
        ldr       r2,[GPIOREG,#0x34]
        and       r1,r1,r2              @ r1 is now either 1 or 0
        mov       RLDREG, r1            @ mov result to return register

        bx        lr                    @ return


@ --------------------------------------------------------------------
@ Inititalize all needed GPIOs (see above), Set Input/Output mode,
@ Set starting values of some Pins (11, 17)
@  param : none
@  return: none
@ -------------------------------------------------------------------
hw_init:

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Table of all needed GPIOs for init in _start:
@
@ GPIO  | Signal    | Hardware
@ --------------------------------------------------------------
@ Signals to control 7-Segment Display:
@ 2 | Output    | SER 7-Segment
@ 3 | Output    | SRCLK 7-Segment
@ 4 | Output    | nSRCLR  7-Segment
@ 5 | Output    | RCLK  7-Segment
@ 6 | Output    | A 7-Segment
@ 7 | Output    | B 7-Segment
@ --------------------------------------------------------------
@ Input from Buttons:
@ 8 | Input   | nBTN1 Taster
@ 9 | Input   | nBTN2 Taster
@ 10  | Input   | nBTN3 Taster
@ --------------------------------------------------------------
@ nRST of Outlet + Step with either Outlet or Color-Wheel:
@ 11  | Output    | nRSTOut Outlet
@ 12  | Output    | StepOut Outlet
@ 13  | Output    | StepCW  Color-WHeel
@ --------------------------------------------------------------
@ Serial Communication:
@ 14  | TX    | UART_TX Serielle Com.
@ 15  | RX    | UART_RX Serielle Com.
@ --------------------------------------------------------------
@ Direction Control + RST of Color-Wheel:
@ 16  | Output    | DirCW Color-Wheel
@ 17  | Output    | nRSTCW  Color-Wheel
@ --------------------------------------------------------------
@ Sorting LED control:
@ 18  | Output    | ledSig  ColorLEDs
@ --------------------------------------------------------------
@ Start/Stop Feeder:
@ 19  | Output    | GoStop  Feeder
@ --------------------------------------------------------------
@ Hallsensor to detect CW and Out position:
@ 20  | Input   | nHallCW Hallsensor ColorWheel
@ 21  | Input   | nHallOut Hallensor Outlet
@ --------------------------------------------------------------
@ Output of Color Detection Sensor:
@ 22  | Input   | colorBit0 Farberkennung
@ 23  | Input   | colorBit1 Farberkennung
@ 24  | Input   | colorBit2 Farberkennung
@ --------------------------------------------------------------
@ Objectsensor of CW (MM didnt fall) and Outlet Direction:
@ 25  | Input   | objCW   Objectsensor CW
@ 26  | Output    | DirOut    Outlet
@ --------------------------------------------------------------
@ Send/Dont Send Sleep Signal to Driver:
@ 27  | Output    | nSLP    Sleep Signal CoProzessor
@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



        ldr       r1, =gpio_mmap_adr          @ reload the addr for accessing the GPIOs
        ldr       GPIOREG, [r1]               @ load GPIO bas addr into GPIOREG



@Configure Output pins
@ Each pin has a 3 bit config mask stored in GPFSEL1 to GPFSEL5 (32bit per register)
@ 000 -> input, 001 -> output
@ thus only output pins need to be configured

@ GPFSEL0, GPIOs 0-9 (Pins 2, 3, 4, 5, 6, 7 needed as Output):

        mov       r1,#0                         @ make sure r1 is 0 -> bit mask

        mov       r2,#1                         @ GPIO2: prepare r2 with 1 for shift
        lsl       r2,#6                         @  left shift 1 to bit pos 6 (FSEL2)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO3: prepare r2 with 1 for shift
        lsl       r2,#9                         @  left shift 1 to bit pos 9 (FSEL3)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO4: prepare r2 with 1 for shift
        lsl       r2,#12                        @  left shift 1 to bit pos 12 (FSEL4)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO5: prepare r2 with 1 for shift
        lsl       r2,#15                        @  left shift 1 to bit pos 15 (FSEL5)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO6: prepare r2 with 1 for shift
        lsl       r2,#18                        @  left shift 1 to bit pos 18 (FSEL6)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO7: prepare r2 with 1 for shift
        lsl       r2,#21                        @  left shift 1 to bit pos 21 (FSEL7)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        str       r1,[GPIOREG]                  @ Store config to GPFSEL0 (base + 0)

@ GPFSEL1, GPIOs 10-19 (11, 12, 13, 16, 17, 18, 19 needed):

        mov       r1,#0                         @ make sure r1 is 0 -> bit mask

        mov       r2,#1                         @ GPIO11: prepare r2 with 1 for shift
        lsl       r2,#3                         @  left shift 1 to bit pos 3 (FSEL11)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO12: prepare r2 with 1 for shift
        lsl       r2,#6                         @  left shift 1 to bit pos 6 (FSEL12)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO13: prepare r2 with 1 for shift
        lsl       r2,#9                         @  left shift 1 to bit pos 9 (FSEL13)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO16: prepare r2 with 1 for shift
        lsl       r2,#18                         @  left shift 1 to bit pos 18 (FSEL16)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO17: prepare r2 with 1 for shift
        lsl       r2,#21                        @  left shift 1 to bit pos 21 (FSEL17)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO18: prepare r2 with 1 for shift
        lsl       r2,#24                         @  left shift 1 to bit pos 24 (FSEL18)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO19: prepare r2 with 1 for shift
        lsl       r2,#27                        @  left shift 1 to bit pos 27 (FSEL19)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        str       r1,[GPIOREG,#4]               @ Store config to GPFSEL1 (base + 4)

@ GPFSEL2, GPIOs 20-29 (26, 27 needed)

        mov       r1,#0                         @ make sure r1 is 0 -> bit mask

        mov       r2,#1                         @ GPIO26: prepare r2 with 1 for shift
        lsl       r2,#18                        @  left shift 1 to bit pos 18 (FSEL26)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        mov       r2,#1                         @ GPIO27: prepare r2 with 1 for shift
        lsl       r2,#21                        @  left shift 1 to bit pos 21 (FSEL27)
        orr       r1,r1,r2                      @  add config to bit mask in r1

        str       r1,[GPIOREG,#8]               @ Store config to GPFSEL2 (base + 8)

        bx        lr

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
sysTimer:
        .word     PERIPH+SYSTIMER_OFFSET


@ - END OF TEXT SECTION   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
