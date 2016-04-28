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
Sheet 1 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 1450 950  2    60   BiDi ~ 0
VDD
$Comp
L GND #PWR01
U 1 1 570ED483
P 2300 3800
F 0 "#PWR01" H 2300 3550 50  0001 C CNN
F 1 "GND" H 2300 3650 50  0000 C CNN
F 2 "" H 2300 3800 50  0000 C CNN
F 3 "" H 2300 3800 50  0000 C CNN
	1    2300 3800
	-1   0    0    1   
$EndComp
$Comp
L MCP23008 U104
U 1 1 57125BD4
P 8450 1700
F 0 "U104" H 8450 2150 70  0000 C CNN
F 1 "MCP23008" H 8450 1100 70  0000 C CNN
F 2 "Housings_SOIC:SOIC-18_7.5x11.6mm_Pitch1.27mm" H 8450 1700 60  0001 C CNN
F 3 "" H 8450 1700 60  0000 C CNN
	1    8450 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 57125BD6
P 1450 1200
F 0 "#PWR02" H 1450 950 50  0001 C CNN
F 1 "GND" H 1450 1050 50  0000 C CNN
F 2 "" H 1450 1200 50  0000 C CNN
F 3 "" H 1450 1200 50  0000 C CNN
	1    1450 1200
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR03
U 1 1 57125BD7
P 4250 3150
F 0 "#PWR03" H 4250 2900 50  0001 C CNN
F 1 "GND" H 4250 3000 50  0000 C CNN
F 2 "" H 4250 3150 50  0000 C CNN
F 3 "" H 4250 3150 50  0000 C CNN
	1    4250 3150
	1    0    0    -1  
$EndComp
$Comp
L MAX11605EEE+ U103
U 1 1 57125BDB
P 5350 5250
F 0 "U103" H 5450 5550 60  0000 C CNN
F 1 "MAX11605EEE+" H 5450 5700 60  0000 C CNN
F 2 "Housings_SSOP:SSOP-16_5.3x6.2mm_Pitch0.65mm" H 2750 6350 60  0001 C CNN
F 3 "" H 2750 6350 60  0000 C CNN
	1    5350 5250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 57125BDC
P 6200 4650
F 0 "#PWR04" H 6200 4400 50  0001 C CNN
F 1 "GND" H 6200 4500 50  0000 C CNN
F 2 "" H 6200 4650 50  0000 C CNN
F 3 "" H 6200 4650 50  0000 C CNN
	1    6200 4650
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X06 P104
U 1 1 57125BDD
P 2350 4000
F 0 "P104" H 2350 4350 50  0000 C CNN
F 1 "CONN_01X06" V 2450 4000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 2350 4000 50  0001 C CNN
F 3 "" H 2350 4000 50  0000 C CNN
	1    2350 4000
	0    1    1    0   
$EndComp
$Comp
L CONN_01X06 P102
U 1 1 57125BDE
P 1550 4000
F 0 "P102" H 1550 4350 50  0000 C CNN
F 1 "CONN_01X06" V 1650 4000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 1550 4000 50  0001 C CNN
F 3 "" H 1550 4000 50  0000 C CNN
	1    1550 4000
	0    1    1    0   
$EndComp
Text GLabel 6200 4450 2    60   BiDi ~ 0
VDD
Text GLabel 4250 1300 1    60   BiDi ~ 0
VDD
Text GLabel 9150 1350 2    60   BiDi ~ 0
VDD
Text GLabel 7750 1450 0    60   BiDi ~ 0
SDA
Text GLabel 7750 1350 0    60   BiDi ~ 0
SCL
Text GLabel 5150 2200 2    60   BiDi ~ 0
SDA
Text GLabel 6200 4850 2    60   BiDi ~ 0
SDA
Text GLabel 6200 5050 2    60   BiDi ~ 0
SCL
$Comp
L CONN_01X12 P105
U 1 1 57125BDF
P 3650 5100
F 0 "P105" H 3650 5750 50  0000 C CNN
F 1 "CONN_01X12" V 3750 5100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x12" H 3650 5100 50  0001 C CNN
F 3 "" H 3650 5100 50  0000 C CNN
	1    3650 5100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR05
U 1 1 57125BE2
P 7700 1700
F 0 "#PWR05" H 7700 1450 50  0001 C CNN
F 1 "GND" H 7700 1550 50  0000 C CNN
F 2 "" H 7700 1700 50  0000 C CNN
F 3 "" H 7700 1700 50  0000 C CNN
	1    7700 1700
	0    1    1    0   
$EndComp
Text GLabel 7750 1550 0    60   BiDi ~ 0
VDD
$Comp
L VCC #PWR06
U 1 1 57125BE7
P 900 1200
F 0 "#PWR06" H 900 1050 50  0001 C CNN
F 1 "VCC" H 900 1350 50  0000 C CNN
F 2 "" H 900 1200 50  0000 C CNN
F 3 "" H 900 1200 50  0000 C CNN
	1    900  1200
	0    -1   -1   0   
$EndComp
Text GLabel 2600 3800 1    60   Output ~ 0
M2+
Text GLabel 2500 3800 1    60   Output ~ 0
M2-
Text GLabel 2400 3800 1    60   BiDi ~ 0
VDD
$Comp
L GND #PWR07
U 1 1 57125BE8
P 2300 3800
F 0 "#PWR07" H 2300 3550 50  0001 C CNN
F 1 "GND" H 2300 3650 50  0000 C CNN
F 2 "" H 2300 3800 50  0000 C CNN
F 3 "" H 2300 3800 50  0000 C CNN
	1    2300 3800
	-1   0    0    1   
$EndComp
Text GLabel 2200 3800 1    60   Input ~ 0
ENC2_A
Text GLabel 2100 3800 1    60   Input ~ 0
ENC2_B
Text GLabel 1700 3800 1    60   Output ~ 0
M1-
Text GLabel 1600 3800 1    60   BiDi ~ 0
VDD
Text GLabel 1400 3800 1    60   Input ~ 0
ENC1_A
Text GLabel 1300 3800 1    60   Input ~ 0
ENC1_B
$Comp
L GND #PWR08
U 1 1 57125BE9
P 1500 3750
F 0 "#PWR08" H 1500 3500 50  0001 C CNN
F 1 "GND" H 1500 3600 50  0000 C CNN
F 2 "" H 1500 3750 50  0000 C CNN
F 3 "" H 1500 3750 50  0000 C CNN
	1    1500 3750
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P103
U 1 1 57125BEB
P 2050 2550
F 0 "P103" H 2050 2700 50  0000 C CNN
F 1 "CONN_01X02" V 2150 2550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x02" H 2050 2550 50  0001 C CNN
F 3 "" H 2050 2550 50  0000 C CNN
	1    2050 2550
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 P101
U 1 1 57125BEC
P 750 2650
F 0 "P101" H 750 2800 50  0000 C CNN
F 1 "CONN_01X02" V 850 2650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x02" H 750 2650 50  0001 C CNN
F 3 "" H 750 2650 50  0000 C CNN
	1    750  2650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR09
U 1 1 57125BEE
P 700 2850
F 0 "#PWR09" H 700 2600 50  0001 C CNN
F 1 "GND" H 700 2700 50  0000 C CNN
F 2 "" H 700 2850 50  0000 C CNN
F 3 "" H 700 2850 50  0000 C CNN
	1    700  2850
	1    0    0    -1  
$EndComp
$Sheet
S 900  800  550  600 
U 57126BEE
F0 "reg5to3_3V" 60
F1 "reg5to3_3V.sch" 60
F2 "GND" B R 1450 1200 60 
F3 "3_3V" B R 1450 950 60 
F4 "VCC" B L 900 1200 60 
$EndSheet
$Comp
L R R102
U 1 1 57127B6C
P 3250 1550
F 0 "R102" V 3330 1550 50  0000 C CNN
F 1 "10k" V 3250 1550 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 3180 1550 50  0001 C CNN
F 3 "" H 3250 1550 50  0000 C CNN
	1    3250 1550
	1    0    0    -1  
$EndComp
$Comp
L R R101
U 1 1 57127BE5
P 2600 1950
F 0 "R101" V 2680 1950 50  0000 C CNN
F 1 "10k" V 2600 1950 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 2530 1950 50  0001 C CNN
F 3 "" H 2600 1950 50  0000 C CNN
	1    2600 1950
	1    0    0    -1  
$EndComp
Text GLabel 3250 1400 1    60   BiDi ~ 0
VDD
Text GLabel 2600 1800 1    60   BiDi ~ 0
VDD
Text GLabel 5250 2000 2    60   Input ~ 0
RX
Text GLabel 5250 1900 2    60   Output ~ 0
TX
$Comp
L R R105
U 1 1 5712A472
P 6100 1500
F 0 "R105" V 6180 1500 50  0000 C CNN
F 1 "47k" V 6100 1500 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 6030 1500 50  0001 C CNN
F 3 "" H 6100 1500 50  0000 C CNN
	1    6100 1500
	1    0    0    -1  
$EndComp
Text GLabel 6100 1650 3    60   BiDi ~ 0
VDD
Text GLabel 5900 1600 3    60   BiDi ~ 0
VDD
$Comp
L R R104
U 1 1 5712A576
P 5900 1450
F 0 "R104" V 5980 1450 50  0000 C CNN
F 1 "47k" V 5900 1450 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 5830 1450 50  0001 C CNN
F 3 "" H 5900 1450 50  0000 C CNN
	1    5900 1450
	1    0    0    -1  
$EndComp
Text GLabel 5700 1250 0    60   BiDi ~ 0
SDA
Text GLabel 5700 1150 0    60   BiDi ~ 0
SCL
$Comp
L CONN_01X08 P107
U 1 1 5713D03F
P 7250 3350
F 0 "P107" H 7250 3800 50  0000 C CNN
F 1 "CONN_I2C_MPU6050" V 7350 3350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08" H 7250 3350 50  0001 C CNN
F 3 "" H 7250 3350 50  0000 C CNN
	1    7250 3350
	0    1    1    0   
$EndComp
Text GLabel 6900 3150 1    60   BiDi ~ 0
VDD
Text GLabel 7100 3150 1    60   BiDi ~ 0
SCL
Text GLabel 7200 3150 1    60   BiDi ~ 0
SDA
Text GLabel 7750 1850 0    60   BiDi ~ 0
VDD
Text GLabel 3350 2300 0    60   Input ~ 0
ENC2_A
Text GLabel 3350 2200 0    60   Input ~ 0
ENC1_A
Text GLabel 10200 1350 0    60   BiDi ~ 0
VDD
Text GLabel 1800 3800 1    60   Output ~ 0
M1+
$Comp
L GND #PWR010
U 1 1 57143D77
P 10200 1250
F 0 "#PWR010" H 10200 1000 50  0001 C CNN
F 1 "GND" H 10200 1100 50  0000 C CNN
F 2 "" H 10200 1250 50  0000 C CNN
F 3 "" H 10200 1250 50  0000 C CNN
	1    10200 1250
	0    1    1    0   
$EndComp
$Comp
L GND #PWR011
U 1 1 57144505
P 7500 3050
F 0 "#PWR011" H 7500 2800 50  0001 C CNN
F 1 "GND" H 7500 2900 50  0000 C CNN
F 2 "" H 7500 3050 50  0000 C CNN
F 3 "" H 7500 3050 50  0000 C CNN
	1    7500 3050
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR012
U 1 1 571445C1
P 7000 3100
F 0 "#PWR012" H 7000 2850 50  0001 C CNN
F 1 "GND" H 7000 2950 50  0000 C CNN
F 2 "" H 7000 3100 50  0000 C CNN
F 3 "" H 7000 3100 50  0000 C CNN
	1    7000 3100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR013
U 1 1 5714473B
P 7750 2150
F 0 "#PWR013" H 7750 1900 50  0001 C CNN
F 1 "GND" H 7750 2000 50  0000 C CNN
F 2 "" H 7750 2150 50  0000 C CNN
F 3 "" H 7750 2150 50  0000 C CNN
	1    7750 2150
	0    1    1    0   
$EndComp
$Comp
L CONN_01X03 P106
U 1 1 571460B6
P 4650 1050
F 0 "P106" H 4650 1250 50  0000 C CNN
F 1 "CONN_01X03" V 4750 1050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 4650 1050 50  0001 C CNN
F 3 "" H 4650 1050 50  0000 C CNN
	1    4650 1050
	0    -1   -1   0   
$EndComp
Text GLabel 4750 1250 3    60   Input ~ 0
RX
Text GLabel 4550 1250 3    60   Output ~ 0
TX
$Comp
L GND #PWR014
U 1 1 57146199
P 4650 1250
F 0 "#PWR014" H 4650 1000 50  0001 C CNN
F 1 "GND" H 4650 1100 50  0000 C CNN
F 2 "" H 4650 1250 50  0000 C CNN
F 3 "" H 4650 1250 50  0000 C CNN
	1    4650 1250
	1    0    0    -1  
$EndComp
Text GLabel 2500 4550 2    60   Output ~ 0
M2+
Text GLabel 2500 5050 2    60   Output ~ 0
M2-
Text GLabel 2500 5150 2    60   BiDi ~ 0
M2_B
$Comp
L GND #PWR015
U 1 1 5714055F
P 2750 4800
F 0 "#PWR015" H 2750 4550 50  0001 C CNN
F 1 "GND" H 2750 4650 50  0000 C CNN
F 2 "" H 2750 4800 50  0000 C CNN
F 3 "" H 2750 4800 50  0000 C CNN
	1    2750 4800
	0    -1   -1   0   
$EndComp
Text GLabel 2500 4350 2    60   BiDi ~ 0
VDD
Text GLabel 2500 4450 2    60   BiDi ~ 0
M2_A
Text GLabel 2500 5250 2    60   Input ~ 0
EN2
$Comp
L L293DD U101
U 1 1 5714092A
P 1950 4850
F 0 "U101" H 1950 4850 60  0000 C CNN
F 1 "L293DD" H 1950 4950 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-20_7.5x12.8mm_Pitch1.27mm" H 1950 4850 60  0001 C CNN
F 3 "" H 1950 4850 60  0000 C CNN
	1    1950 4850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 57140F36
P 950 4750
F 0 "#PWR016" H 950 4500 50  0001 C CNN
F 1 "GND" H 950 4600 50  0000 C CNN
F 2 "" H 950 4750 50  0000 C CNN
F 3 "" H 950 4750 50  0000 C CNN
	1    950  4750
	0    1    1    0   
$EndComp
Text GLabel 1400 4450 0    60   BiDi ~ 0
M1_A
Text GLabel 1400 5150 0    60   BiDi ~ 0
M1_B
Text GLabel 1400 5050 0    60   Output ~ 0
M1-
Text GLabel 1400 4550 0    60   Output ~ 0
M1+
$Comp
L VCC #PWR017
U 1 1 57140F40
P 1400 5250
F 0 "#PWR017" H 1400 5100 50  0001 C CNN
F 1 "VCC" H 1400 5400 50  0000 C CNN
F 2 "" H 1400 5250 50  0000 C CNN
F 3 "" H 1400 5250 50  0000 C CNN
	1    1400 5250
	0    -1   -1   0   
$EndComp
Text GLabel 1400 4350 0    60   Input ~ 0
EN1
$Comp
L PWR_FLAG #FLG018
U 1 1 57154A7C
P 2100 2750
F 0 "#FLG018" H 2100 2845 50  0001 C CNN
F 1 "PWR_FLAG" H 2100 2930 50  0000 C CNN
F 2 "" H 2100 2750 50  0000 C CNN
F 3 "" H 2100 2750 50  0000 C CNN
	1    2100 2750
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR019
U 1 1 57154E13
P 2100 2950
F 0 "#PWR019" H 2100 2800 50  0001 C CNN
F 1 "VCC" H 2100 3100 50  0000 C CNN
F 2 "" H 2100 2950 50  0000 C CNN
F 3 "" H 2100 2950 50  0000 C CNN
	1    2100 2950
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG020
U 1 1 571554D1
P 4250 3100
F 0 "#FLG020" H 4250 3195 50  0001 C CNN
F 1 "PWR_FLAG" H 4250 3280 50  0000 C CNN
F 2 "" H 4250 3100 50  0000 C CNN
F 3 "" H 4250 3100 50  0000 C CNN
	1    4250 3100
	0    1    1    0   
$EndComp
NoConn ~ 3350 2000
NoConn ~ 7300 3150
NoConn ~ 7400 3150
Text GLabel 5150 2400 2    60   Output ~ 0
EN1
Text GLabel 5150 2100 2    60   Output ~ 0
EN2
Text GLabel 3350 2400 0    60   Input ~ 0
ENC1_B
Text GLabel 3350 2500 0    60   Input ~ 0
ENC2_B
Text GLabel 9300 2150 2    60   Output ~ 0
M1_A
Text GLabel 9500 2050 2    60   Output ~ 0
M1_B
Text GLabel 9700 1950 2    60   Output ~ 0
M2_A
Text GLabel 9900 1850 2    60   Output ~ 0
M2_B
$Comp
L CONN_01X06 P108
U 1 1 571568CA
P 10400 1500
F 0 "P108" H 10400 1850 50  0000 C CNN
F 1 "CONN_PINES_DIG" V 10500 1500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 10400 1500 50  0001 C CNN
F 3 "" H 10400 1500 50  0000 C CNN
	1    10400 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 57154834
P 6750 1800
F 0 "#PWR021" H 6750 1550 50  0001 C CNN
F 1 "GND" H 6750 1650 50  0000 C CNN
F 2 "" H 6750 1800 50  0000 C CNN
F 3 "" H 6750 1800 50  0000 C CNN
	1    6750 1800
	1    0    0    -1  
$EndComp
Text GLabel 6450 1150 1    60   BiDi ~ 0
VDD
$Comp
L CP C101
U 1 1 5715447A
P 6600 1400
F 0 "C101" H 6625 1500 50  0000 L CNN
F 1 "10u" H 6625 1300 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:TantalC_SizeC_EIA-6032_HandSoldering" H 6638 1250 50  0001 C CNN
F 3 "" H 6600 1400 50  0000 C CNN
	1    6600 1400
	0    -1   -1   0   
$EndComp
$Comp
L C C102
U 1 1 57154411
P 6600 1650
F 0 "C102" H 6625 1750 50  0000 L CNN
F 1 "0.1u" H 6625 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6638 1500 50  0001 C CNN
F 3 "" H 6600 1650 50  0000 C CNN
	1    6600 1650
	0    1    1    0   
$EndComp
$Comp
L SW_PUSH_SMALL SW101
U 1 1 5719131F
P 5900 2400
F 0 "SW101" H 6050 2510 50  0000 C CNN
F 1 "SW_PUSH_SMALL" H 5900 2321 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVPBF" H 5900 2400 50  0001 C CNN
F 3 "" H 5900 2400 50  0000 C CNN
	1    5900 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 571913A6
P 6000 2500
F 0 "#PWR022" H 6000 2250 50  0001 C CNN
F 1 "GND" H 6000 2350 50  0000 C CNN
F 2 "" H 6000 2500 50  0000 C CNN
F 3 "" H 6000 2500 50  0000 C CNN
	1    6000 2500
	1    0    0    -1  
$EndComp
$Comp
L R R103
U 1 1 57191663
P 5150 2650
F 0 "R103" V 5230 2650 50  0000 C CNN
F 1 "10k" V 5150 2650 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 5080 2650 50  0001 C CNN
F 3 "" H 5150 2650 50  0000 C CNN
	1    5150 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 571916BE
P 5150 2800
F 0 "#PWR023" H 5150 2550 50  0001 C CNN
F 1 "GND" H 5150 2650 50  0000 C CNN
F 2 "" H 5150 2800 50  0000 C CNN
F 3 "" H 5150 2800 50  0000 C CNN
	1    5150 2800
	1    0    0    -1  
$EndComp
$Comp
L ESP-12 U102
U 1 1 57125BD1
P 4250 2200
F 0 "U102" H 4250 2100 50  0000 C CNN
F 1 "ESP-12" H 4250 2300 50  0000 C CNN
F 2 "ESP8266:ESP-12" H 4250 2200 50  0001 C CNN
F 3 "" H 4250 2200 50  0001 C CNN
	1    4250 2200
	1    0    0    -1  
$EndComp
Text GLabel 5950 2300 2    60   BiDi ~ 0
SCL
NoConn ~ 7600 3150
NoConn ~ 7750 2050
$Comp
L SWITCH_INV SW102
U 1 1 57214298
P 1500 2850
F 0 "SW102" H 1300 3000 50  0000 C CNN
F 1 "SWITCH_INV" H 1350 2700 50  0000 C CNN
F 2 "Buttons_Switches_ThroughHole:SW_Micro_SPST" H 1500 2850 50  0001 C CNN
F 3 "" H 1500 2850 50  0000 C CNN
	1    1500 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4450 4450 4450
Wire Wire Line
	3950 4450 3950 4550
Wire Wire Line
	3850 4650 4450 4650
Wire Wire Line
	4450 4850 4350 4850
Wire Wire Line
	4350 4850 4350 4750
Wire Wire Line
	4350 4750 3850 4750
Wire Wire Line
	4450 5050 4300 5050
Wire Wire Line
	4300 5050 4300 4850
Wire Wire Line
	4300 4850 3850 4850
Wire Wire Line
	3850 4950 4250 4950
Wire Wire Line
	4250 4950 4250 5250
Wire Wire Line
	4250 5250 4450 5250
Wire Wire Line
	3850 5050 4200 5050
Wire Wire Line
	4200 5050 4200 5850
Wire Wire Line
	3850 5150 4150 5150
Wire Wire Line
	4150 5150 4150 5650
Wire Wire Line
	3850 5250 4100 5250
Wire Wire Line
	4100 5250 4100 5450
Wire Wire Line
	3850 5350 4050 5350
Wire Wire Line
	4050 5350 4050 6200
Wire Wire Line
	4050 6200 6350 6200
Wire Wire Line
	6350 6200 6350 5850
Wire Wire Line
	3850 5450 4000 5450
Wire Wire Line
	4000 5450 4000 6250
Wire Wire Line
	4000 6250 6400 6250
Wire Wire Line
	6400 6250 6400 5650
Wire Wire Line
	3950 5550 3950 6300
Wire Wire Line
	3950 6300 6450 6300
Wire Wire Line
	6450 6300 6450 5450
Wire Wire Line
	6500 5250 6500 6350
Wire Wire Line
	6500 6350 3900 6350
Wire Wire Line
	3900 6350 3900 5650
Wire Wire Line
	3900 5650 3850 5650
Wire Wire Line
	3950 5550 3850 5550
Wire Wire Line
	3950 4550 3850 4550
Wire Wire Line
	9150 1950 9700 1950
Wire Wire Line
	9900 1850 9150 1850
Wire Wire Line
	9150 1750 10200 1750
Wire Wire Line
	10200 1650 9150 1650
Wire Wire Line
	9150 1550 10200 1550
Wire Wire Line
	10200 1450 9150 1450
Wire Wire Line
	800  2850 1000 2850
Wire Wire Line
	3250 1700 3250 1900
Wire Wire Line
	3250 1900 3350 1900
Wire Wire Line
	2600 2100 3350 2100
Wire Wire Line
	5700 1250 6100 1250
Wire Wire Line
	6100 1250 6100 1350
Wire Wire Line
	5700 1150 5900 1150
Wire Wire Line
	5900 1150 5900 1300
Connection ~ 5900 1150
Connection ~ 6100 1250
Wire Wire Line
	7500 3050 7500 3150
Wire Wire Line
	1500 3750 1500 3800
Wire Wire Line
	4250 3100 4250 3150
Wire Wire Line
	7750 1650 7700 1650
Wire Wire Line
	7700 1650 7700 1750
Wire Wire Line
	7700 1750 7750 1750
Connection ~ 7700 1700
Wire Wire Line
	7000 3100 7000 3150
Wire Wire Line
	9150 2150 9300 2150
Wire Wire Line
	9150 2050 9500 2050
Wire Wire Line
	5250 2000 5150 2000
Wire Wire Line
	5250 1900 5150 1900
Wire Wire Line
	2500 4650 2650 4650
Wire Wire Line
	2650 4650 2650 4950
Wire Wire Line
	2650 4950 2500 4950
Wire Wire Line
	2500 4850 2650 4850
Connection ~ 2650 4850
Wire Wire Line
	2500 4750 2650 4750
Connection ~ 2650 4750
Wire Wire Line
	2650 4800 2750 4800
Connection ~ 2650 4800
Wire Wire Line
	1250 4950 1400 4950
Wire Wire Line
	1250 4650 1250 4950
Wire Wire Line
	1250 4650 1400 4650
Wire Wire Line
	950  4750 1400 4750
Connection ~ 1250 4750
Wire Wire Line
	1400 4850 1250 4850
Connection ~ 1250 4850
Wire Wire Line
	2100 2750 2100 2950
Connection ~ 6750 1650
Wire Wire Line
	6750 1400 6750 1800
Connection ~ 6450 1400
Connection ~ 6450 1650
Wire Wire Line
	6450 1150 6450 1650
Wire Wire Line
	5150 2300 5950 2300
Connection ~ 5800 2300
Wire Wire Line
	6500 5250 6200 5250
Wire Wire Line
	6450 5450 6200 5450
Wire Wire Line
	6400 5650 6200 5650
Wire Wire Line
	6350 5850 6200 5850
Wire Wire Line
	4100 5450 4450 5450
Wire Wire Line
	4150 5650 4450 5650
Wire Wire Line
	4200 5850 4450 5850
$EndSCHEMATC
