EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ESP8266
LIBS:stepper drivers
LIBS:mpu-6050
LIBS:MCP23008
LIBS:microchip_adcdac
LIBS:ESP_multi_board-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L LD1117S33TR U1
U 1 1 57126C2F
P 4700 2800
F 0 "U1" H 4700 3050 50  0000 C CNN
F 1 "LD1117S33TR" H 4700 3000 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 4700 2900 50  0000 C CNN
F 3 "" H 4700 2800 50  0000 C CNN
	1    4700 2800
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 57126C5D
P 4100 3050
F 0 "C2" H 4125 3150 50  0000 L CNN
F 1 "100n" H 4125 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4138 2900 50  0001 C CNN
F 3 "" H 4100 3050 50  0000 C CNN
	1    4100 3050
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 57126C91
P 5350 3000
F 0 "C1" H 5375 3100 50  0000 L CNN
F 1 "10u" H 5375 2900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5388 2850 50  0001 C CNN
F 3 "" H 5350 3000 50  0000 C CNN
	1    5350 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3150 5350 3200
Wire Wire Line
	5350 3200 4100 3200
Wire Wire Line
	4700 3050 4700 3450
Connection ~ 4700 3200
Text HLabel 4700 3450 3    60   BiDi ~ 0
GND
Wire Wire Line
	3850 2750 4300 2750
Wire Wire Line
	4100 2750 4100 2900
Connection ~ 4100 2750
Text HLabel 3850 2750 0    60   BiDi ~ 0
5V
Wire Wire Line
	5100 2750 5600 2750
Wire Wire Line
	5350 2750 5350 2850
Connection ~ 5350 2750
Text HLabel 5600 2750 2    60   BiDi ~ 0
3_3V
$EndSCHEMATC
