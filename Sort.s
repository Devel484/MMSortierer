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


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	.data
@ - END OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF TEXT SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	.text
	.balign 4
	.global _start

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
@ Inititalize all needed GPIOs (see above), Set Input/Output mode, 
@ Set starting values of some Pins (11, 17)
@  param: none
@  return: none
@ -------------------------------------------------------------------
_start:


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
main:

@ - END OF TEXT SECTION   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

