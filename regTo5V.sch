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
Sheet 3 3
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
L LD1117S50TR U?
U 1 1 57169709
P 5150 2950
F 0 "U?" H 5150 3200 50  0000 C CNN
F 1 "LD1117S50TR" H 5150 3150 50  0000 C CNN
F 2 "SOT-223" H 5150 3050 50  0000 C CNN
F 3 "" H 5150 2950 50  0000 C CNN
	1    5150 2950
	1    0    0    -1  
$EndComp
Text HLabel 5150 3450 3    60   BiDi ~ 0
GND
$Comp
L C C?
U 1 1 5716975B
P 5700 3150
F 0 "C?" H 5725 3250 50  0000 L CNN
F 1 "10u" H 5725 3050 50  0000 L CNN
F 2 "" H 5738 3000 50  0000 C CNN
F 3 "" H 5700 3150 50  0000 C CNN
	1    5700 3150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5716977C
P 4600 3150
F 0 "C?" H 4625 3250 50  0000 L CNN
F 1 "100n" H 4625 3050 50  0000 L CNN
F 2 "" H 4638 3000 50  0000 C CNN
F 3 "" H 4600 3150 50  0000 C CNN
	1    4600 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3200 5150 3450
Wire Wire Line
	4750 2900 4600 2900
Wire Wire Line
	4600 2900 4600 3000
Wire Wire Line
	4600 3300 5700 3300
Connection ~ 5150 3300
Wire Wire Line
	5700 3000 5700 2900
Wire Wire Line
	5700 2900 5550 2900
Text HLabel 5700 2900 2    60   BiDi ~ 0
5V
Text HLabel 4600 2900 0    60   BiDi ~ 0
VCC
$EndSCHEMATC
