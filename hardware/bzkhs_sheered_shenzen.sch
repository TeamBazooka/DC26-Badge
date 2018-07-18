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
LIBS:bzkhs_sheered_shenzen-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L STM32L031x4/6 U1
U 1 1 5B4191AA
P 2100 2100
F 0 "U1" H 2100 2650 60  0000 C CNN
F 1 "STM32L031F6" H 2050 1550 60  0000 C CNN
F 2 "Housings_SSOP:TSSOP-20_4.4x6.5mm_Pitch0.65mm" H 1450 2300 60  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/stm32l031f6.pdf" H 1450 2300 60  0001 C CNN
F 4 "497-17484-ND" H 2100 2100 60  0001 C CNN "DK-PN"
	1    2100 2100
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5B4191CE
P 1300 1050
F 0 "C2" H 1325 1150 50  0000 L CNN
F 1 "C" H 1325 950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1338 900 50  0001 C CNN
F 3 "" H 1300 1050 50  0001 C CNN
	1    1300 1050
	0    -1   -1   0   
$EndComp
$Comp
L C C1
U 1 1 5B41920D
P 1300 800
F 0 "C1" H 1325 900 50  0000 L CNN
F 1 "C" H 1325 700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1338 650 50  0001 C CNN
F 3 "" H 1300 800 50  0001 C CNN
	1    1300 800 
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR01
U 1 1 5B419453
P 3450 1950
F 0 "#PWR01" H 3450 1800 50  0001 C CNN
F 1 "+3.3V" H 3450 2090 50  0000 C CNN
F 2 "" H 3450 1950 50  0001 C CNN
F 3 "" H 3450 1950 50  0001 C CNN
	1    3450 1950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5B419479
P 3450 2200
F 0 "#PWR02" H 3450 1950 50  0001 C CNN
F 1 "GND" H 3450 2050 50  0000 C CNN
F 2 "" H 3450 2200 50  0001 C CNN
F 3 "" H 3450 2200 50  0001 C CNN
	1    3450 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2050 3450 2050
Wire Wire Line
	3450 2050 3450 1950
Wire Wire Line
	3150 2150 3450 2150
Wire Wire Line
	3450 2150 3450 2200
$Comp
L +3.3V #PWR03
U 1 1 5B419517
P 700 2000
F 0 "#PWR03" H 700 1850 50  0001 C CNN
F 1 "+3.3V" H 700 2140 50  0000 C CNN
F 2 "" H 700 2000 50  0001 C CNN
F 3 "" H 700 2000 50  0001 C CNN
	1    700  2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 2050 700  2050
Wire Wire Line
	700  2050 700  2000
$Comp
L +3.3V #PWR04
U 1 1 5B419573
P 950 850
F 0 "#PWR04" H 950 700 50  0001 C CNN
F 1 "+3.3V" H 950 990 50  0000 C CNN
F 2 "" H 950 850 50  0001 C CNN
F 3 "" H 950 850 50  0001 C CNN
	1    950  850 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1050 1050 1150 1050
Wire Wire Line
	1050 800  1050 1050
Wire Wire Line
	1050 850  950  850 
Wire Wire Line
	1050 800  1150 800 
Connection ~ 1050 850 
$Comp
L GND #PWR05
U 1 1 5B4195A3
P 1550 950
F 0 "#PWR05" H 1550 700 50  0001 C CNN
F 1 "GND" H 1550 800 50  0000 C CNN
F 2 "" H 1550 950 50  0001 C CNN
F 3 "" H 1550 950 50  0001 C CNN
	1    1550 950 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1500 1050 1450 1050
Wire Wire Line
	1500 800  1500 1050
Wire Wire Line
	1500 950  1550 950 
Wire Wire Line
	1500 800  1450 800 
Connection ~ 1500 950 
$Comp
L LED D1
U 1 1 5B419679
P 4200 2200
F 0 "D1" H 4200 2300 50  0000 C CNN
F 1 "LED" H 4200 2100 50  0000 C CNN
F 2 "BZKHS:LED" H 4200 2200 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 4200 2200 50  0001 C CNN
F 4 "475-1145-1-ND" H 4200 2200 60  0001 C CNN "DK-PN"
	1    4200 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 3250 4200 3450
Wire Wire Line
	4200 2450 4200 2350
Wire Wire Line
	5000 2450 5000 2350
Wire Wire Line
	5800 2450 5800 2350
Wire Wire Line
	6600 2450 6600 2350
Wire Wire Line
	7400 2450 7400 2350
Wire Wire Line
	5000 3250 5000 3450
Wire Wire Line
	5800 3250 5800 3550
Wire Wire Line
	6600 3250 6600 3450
Wire Wire Line
	7400 3250 7400 3450
$Comp
L GND #PWR06
U 1 1 5B419DCD
P 5800 1100
F 0 "#PWR06" H 5800 850 50  0001 C CNN
F 1 "GND" H 5800 950 50  0000 C CNN
F 2 "" H 5800 1100 50  0001 C CNN
F 3 "" H 5800 1100 50  0001 C CNN
	1    5800 1100
	-1   0    0    1   
$EndComp
Text Label 4800 1750 0    60   ~ 0
PA1
Text Label 5600 1450 0    60   ~ 0
PA2
Text Label 6400 1750 0    60   ~ 0
PA3
Text Label 7200 1450 0    60   ~ 0
PA4
Text Label 8000 1750 0    60   ~ 0
PA5
Text Label 1000 2250 2    60   ~ 0
PA1
Text Label 1000 2350 2    60   ~ 0
PA2
Text Label 1000 2450 2    60   ~ 0
PA3
Text Label 1000 2550 2    60   ~ 0
PA4
Text Label 3150 2550 0    60   ~ 0
PA5
$Comp
L GND #PWR07
U 1 1 5B41A565
P 2200 1200
F 0 "#PWR07" H 2200 950 50  0001 C CNN
F 1 "GND" H 2200 1050 50  0000 C CNN
F 2 "" H 2200 1200 50  0001 C CNN
F 3 "" H 2200 1200 50  0001 C CNN
	1    2200 1200
	0    1    1    0   
$EndComp
$Comp
L Conn_01x05 ICSP1
U 1 1 5B41A6E0
P 2750 1000
F 0 "ICSP1" H 2750 1300 50  0000 C CNN
F 1 "Conn_01x05" H 2750 700 50  0000 C CNN
F 2 "SF_Connectors:1X05_LOCK_LONGPADS" H 2750 1000 50  0001 C CNN
F 3 "" H 2750 1000 50  0001 C CNN
	1    2750 1000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR08
U 1 1 5B41A730
P 2250 800
F 0 "#PWR08" H 2250 650 50  0001 C CNN
F 1 "+3.3V" H 2250 940 50  0000 C CNN
F 2 "" H 2250 800 50  0001 C CNN
F 3 "" H 2250 800 50  0001 C CNN
	1    2250 800 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2550 800  2250 800 
Wire Wire Line
	2200 1200 2550 1200
Wire Wire Line
	2550 900  2400 900 
Wire Wire Line
	2550 1000 2400 1000
Wire Wire Line
	2550 1100 2400 1100
Text Label 2400 1100 2    60   ~ 0
RST
Text Label 2400 1000 2    60   ~ 0
SWDAT
Text Label 2400 900  2    60   ~ 0
SWCLK
Text Label 1000 1950 2    60   ~ 0
RST
Text Label 3150 1650 0    60   ~ 0
SWCLK
Text Label 3150 1750 0    60   ~ 0
SWDAT
Wire Wire Line
	3450 900  3500 900 
$Comp
L Battery_Cell BT1
U 1 1 5B41BEC4
P 3700 900
F 0 "BT1" V 3850 950 50  0000 L CNN
F 1 "Battery_Cell" V 3600 450 50  0000 L CNN
F 2 "BZKHS:aaa" V 3700 960 50  0001 C CNN
F 3 "http://www.memoryprotectiondevices.com/datasheets/BC3AAAPC-datasheet.pdf" V 3700 960 50  0001 C CNN
F 4 "BC3AAAPC-ND" V 3700 900 60  0001 C CNN "DK-PN"
	1    3700 900 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR09
U 1 1 5B41BF62
P 4250 900
F 0 "#PWR09" H 4250 650 50  0001 C CNN
F 1 "GND" H 4250 750 50  0000 C CNN
F 2 "" H 4250 900 50  0001 C CNN
F 3 "" H 4250 900 50  0001 C CNN
	1    4250 900 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3800 900  3950 900 
$Comp
L +3.3V #PWR010
U 1 1 5B424315
P 3450 900
F 0 "#PWR010" H 3450 750 50  0001 C CNN
F 1 "+3.3V" H 3450 1040 50  0000 C CNN
F 2 "" H 3450 900 50  0001 C CNN
F 3 "" H 3450 900 50  0001 C CNN
	1    3450 900 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4200 2050 4200 1950
Wire Wire Line
	5000 2050 5000 1650
Wire Wire Line
	5800 2050 5800 1950
Wire Wire Line
	6600 2050 6600 1650
Wire Wire Line
	7400 2050 7400 1950
$Comp
L +3.3V #PWR011
U 1 1 5B42BA55
P 5800 3550
F 0 "#PWR011" H 5800 3400 50  0001 C CNN
F 1 "+3.3V" H 5800 3690 50  0000 C CNN
F 2 "" H 5800 3550 50  0001 C CNN
F 3 "" H 5800 3550 50  0001 C CNN
	1    5800 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	7400 3450 4200 3450
Connection ~ 5000 3450
Connection ~ 5800 3450
Connection ~ 6600 3450
Wire Wire Line
	4200 1550 4200 1200
Wire Wire Line
	4200 1200 7400 1200
Wire Wire Line
	5000 1200 5000 1250
Wire Wire Line
	5800 1100 5800 1550
Connection ~ 5000 1200
Wire Wire Line
	6600 1200 6600 1250
Connection ~ 5800 1200
Wire Wire Line
	7400 1200 7400 1550
Connection ~ 6600 1200
Wire Wire Line
	4050 900  4250 900 
$Comp
L Conn_02x02_Counter_Clockwise SAO1
U 1 1 5B464504
P 1550 3100
F 0 "SAO1" H 1600 3200 50  0000 C CNN
F 1 "2X2" H 1600 2900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x02_Pitch2.54mm" H 1550 3100 50  0001 C CNN
F 3 "" H 1550 3100 50  0001 C CNN
	1    1550 3100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR012
U 1 1 5B46461F
P 1350 3100
F 0 "#PWR012" H 1350 2950 50  0001 C CNN
F 1 "+3.3V" H 1350 3240 50  0000 C CNN
F 2 "" H 1350 3100 50  0001 C CNN
F 3 "" H 1350 3100 50  0001 C CNN
	1    1350 3100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR013
U 1 1 5B46467B
P 1300 3750
F 0 "#PWR013" H 1300 3500 50  0001 C CNN
F 1 "GND" H 1300 3600 50  0000 C CNN
F 2 "" H 1300 3750 50  0001 C CNN
F 3 "" H 1300 3750 50  0001 C CNN
	1    1300 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 3200 1300 3350
Wire Wire Line
	1300 3200 1350 3200
Text Label 700  3550 2    60   ~ 0
PA6
Text Label 3150 2450 0    60   ~ 0
PA6
Text Label 1850 3100 0    60   ~ 0
SDA
Text Label 1850 3200 0    60   ~ 0
SCL
Text Label 3150 1950 0    60   ~ 0
SCL
Text Label 3150 1850 0    60   ~ 0
SDA
$Comp
L +3.3V #PWR014
U 1 1 5B465E51
P 2450 2900
F 0 "#PWR014" H 2450 2750 50  0001 C CNN
F 1 "+3.3V" H 2450 3040 50  0000 C CNN
F 2 "" H 2450 2900 50  0001 C CNN
F 3 "" H 2450 2900 50  0001 C CNN
	1    2450 2900
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR015
U 1 1 5B465EB3
P 2450 3200
F 0 "#PWR015" H 2450 2950 50  0001 C CNN
F 1 "GND" H 2450 3050 50  0000 C CNN
F 2 "" H 2450 3200 50  0001 C CNN
F 3 "" H 2450 3200 50  0001 C CNN
	1    2450 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 2900 2800 2900
Wire Wire Line
	2450 3500 2800 3500
Text Label 1000 1650 2    60   ~ 0
BOOT0
Text Label 1000 1750 2    60   ~ 0
PC14
Text Label 1000 1850 2    60   ~ 0
PC15
Text Label 1000 2150 2    60   ~ 0
PA0
Text Label 3150 2250 0    60   ~ 0
PB1
Text Label 3150 2350 0    60   ~ 0
PA7
Text Label 2800 3000 2    60   ~ 0
PC14
Text Label 2800 3100 2    60   ~ 0
PC15
Text Label 2800 3600 2    60   ~ 0
PA0
Text Label 2800 4300 2    60   ~ 0
PB1
Text Label 2800 3700 2    60   ~ 0
PA7
$Comp
L Conn_01x04 J1
U 1 1 5B46698B
P 3000 3000
F 0 "J1" H 3000 3200 50  0000 C CNN
F 1 "Conn_01x04" H 3000 2700 50  0000 C CNN
F 2 "SF_Connectors:1X04_LONGPADS" H 3000 3000 50  0001 C CNN
F 3 "" H 3000 3000 50  0001 C CNN
	1    3000 3000
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J2
U 1 1 5B466AB1
P 3000 3600
F 0 "J2" H 3000 3800 50  0000 C CNN
F 1 "Conn_01x04" H 3000 3300 50  0000 C CNN
F 2 "SF_Connectors:1X04_LONGPADS" H 3000 3600 50  0001 C CNN
F 3 "" H 3000 3600 50  0001 C CNN
	1    3000 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3200 2800 3200
$Comp
L +3.3V #PWR016
U 1 1 5B466F8D
P 2450 3500
F 0 "#PWR016" H 2450 3350 50  0001 C CNN
F 1 "+3.3V" H 2450 3640 50  0000 C CNN
F 2 "" H 2450 3500 50  0001 C CNN
F 3 "" H 2450 3500 50  0001 C CNN
	1    2450 3500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR017
U 1 1 5B467106
P 2450 3800
F 0 "#PWR017" H 2450 3550 50  0001 C CNN
F 1 "GND" H 2450 3650 50  0000 C CNN
F 2 "" H 2450 3800 50  0001 C CNN
F 3 "" H 2450 3800 50  0001 C CNN
	1    2450 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 3800 2800 3800
$Comp
L Conn_01x04 J3
U 1 1 5B4671C9
P 3000 4200
F 0 "J3" H 3000 4400 50  0000 C CNN
F 1 "Conn_01x04" H 3000 3900 50  0000 C CNN
F 2 "SF_Connectors:1X04_LONGPADS" H 3000 4200 50  0001 C CNN
F 3 "" H 3000 4200 50  0001 C CNN
	1    3000 4200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR018
U 1 1 5B467240
P 2450 4100
F 0 "#PWR018" H 2450 3950 50  0001 C CNN
F 1 "+3.3V" H 2450 4240 50  0000 C CNN
F 2 "" H 2450 4100 50  0001 C CNN
F 3 "" H 2450 4100 50  0001 C CNN
	1    2450 4100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR019
U 1 1 5B4672A8
P 2450 4400
F 0 "#PWR019" H 2450 4150 50  0001 C CNN
F 1 "GND" H 2450 4250 50  0000 C CNN
F 2 "" H 2450 4400 50  0001 C CNN
F 3 "" H 2450 4400 50  0001 C CNN
	1    2450 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 4100 2800 4100
Wire Wire Line
	2450 4400 2800 4400
Text Label 2800 4200 2    60   ~ 0
BOOT0
$Comp
L Conn_01x02 J5
U 1 1 5B46773B
P 3000 5100
F 0 "J5" H 3000 5200 50  0000 C CNN
F 1 "Conn_01x02" H 3000 4900 50  0000 C CNN
F 2 "SF_Connectors:1X02_LONGPADS" H 3000 5100 50  0001 C CNN
F 3 "" H 3000 5100 50  0001 C CNN
	1    3000 5100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR020
U 1 1 5B4677CB
P 2800 5100
F 0 "#PWR020" H 2800 4950 50  0001 C CNN
F 1 "+3.3V" H 2800 5240 50  0000 C CNN
F 2 "" H 2800 5100 50  0001 C CNN
F 3 "" H 2800 5100 50  0001 C CNN
	1    2800 5100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR021
U 1 1 5B467836
P 2800 5200
F 0 "#PWR021" H 2800 4950 50  0001 C CNN
F 1 "GND" H 2800 5050 50  0000 C CNN
F 2 "" H 2800 5200 50  0001 C CNN
F 3 "" H 2800 5200 50  0001 C CNN
	1    2800 5200
	0    1    1    0   
$EndComp
$Comp
L Conn_01x02 J4
U 1 1 5B4679C7
P 3000 4700
F 0 "J4" H 3000 4800 50  0000 C CNN
F 1 "Conn_01x02" H 3000 4500 50  0000 C CNN
F 2 "SF_Connectors:1X02_LONGPADS" H 3000 4700 50  0001 C CNN
F 3 "" H 3000 4700 50  0001 C CNN
	1    3000 4700
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR022
U 1 1 5B4679CD
P 2800 4700
F 0 "#PWR022" H 2800 4550 50  0001 C CNN
F 1 "+3.3V" H 2800 4840 50  0000 C CNN
F 2 "" H 2800 4700 50  0001 C CNN
F 3 "" H 2800 4700 50  0001 C CNN
	1    2800 4700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR023
U 1 1 5B4679D3
P 2800 4800
F 0 "#PWR023" H 2800 4550 50  0001 C CNN
F 1 "GND" H 2800 4650 50  0000 C CNN
F 2 "" H 2800 4800 50  0001 C CNN
F 3 "" H 2800 4800 50  0001 C CNN
	1    2800 4800
	0    1    1    0   
$EndComp
$Comp
L Conn_02x02_Counter_Clockwise SAO2
U 1 1 5B465135
P 1550 4150
F 0 "SAO2" H 1600 4250 50  0000 C CNN
F 1 "2X2" H 1600 3950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x02_Pitch2.54mm" H 1550 4150 50  0001 C CNN
F 3 "" H 1550 4150 50  0001 C CNN
	1    1550 4150
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR024
U 1 1 5B46513B
P 1350 4150
F 0 "#PWR024" H 1350 4000 50  0001 C CNN
F 1 "+3.3V" H 1350 4290 50  0000 C CNN
F 2 "" H 1350 4150 50  0001 C CNN
F 3 "" H 1350 4150 50  0001 C CNN
	1    1350 4150
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR025
U 1 1 5B465141
P 1300 4800
F 0 "#PWR025" H 1300 4550 50  0001 C CNN
F 1 "GND" H 1300 4650 50  0000 C CNN
F 2 "" H 1300 4800 50  0001 C CNN
F 3 "" H 1300 4800 50  0001 C CNN
	1    1300 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 4250 1300 4400
Wire Wire Line
	1300 4250 1350 4250
Text Label 700  4600 2    60   ~ 0
PA6
Text Label 1850 4150 0    60   ~ 0
SDA
Text Label 1850 4250 0    60   ~ 0
SCL
$Comp
L R R3
U 1 1 5B465F0B
P 4650 1750
F 0 "R3" V 4730 1750 50  0000 C CNN
F 1 "1kR" V 4650 1750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 4580 1750 50  0001 C CNN
F 3 "" H 4650 1750 50  0001 C CNN
	1    4650 1750
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 5B46602D
P 5450 1450
F 0 "R4" V 5530 1450 50  0000 C CNN
F 1 "1kR" V 5450 1450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5380 1450 50  0001 C CNN
F 3 "" H 5450 1450 50  0001 C CNN
	1    5450 1450
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 5B466123
P 6250 1750
F 0 "R5" V 6330 1750 50  0000 C CNN
F 1 "1kR" V 6250 1750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6180 1750 50  0001 C CNN
F 3 "" H 6250 1750 50  0001 C CNN
	1    6250 1750
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 5B46621D
P 7050 1450
F 0 "R6" V 7130 1450 50  0000 C CNN
F 1 "1kR" V 7050 1450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6980 1450 50  0001 C CNN
F 3 "" H 7050 1450 50  0001 C CNN
	1    7050 1450
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 5B466321
P 7850 1750
F 0 "R7" V 7930 1750 50  0000 C CNN
F 1 "1kR" V 7850 1750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7780 1750 50  0001 C CNN
F 3 "" H 7850 1750 50  0001 C CNN
	1    7850 1750
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5B46724D
P 850 4600
F 0 "R2" V 930 4600 50  0000 C CNN
F 1 "1kR" V 850 4600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 780 4600 50  0001 C CNN
F 3 "" H 850 4600 50  0001 C CNN
	1    850  4600
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 5B467349
P 850 3550
F 0 "R1" V 930 3550 50  0000 C CNN
F 1 "1kR" V 850 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 780 3550 50  0001 C CNN
F 3 "" H 850 3550 50  0001 C CNN
	1    850  3550
	0    1    1    0   
$EndComp
$Comp
L LED D2
U 1 1 5B468024
P 4200 2600
F 0 "D2" H 4200 2700 50  0000 C CNN
F 1 "LED" H 4200 2500 50  0000 C CNN
F 2 "BZKHS:LED" H 4200 2600 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 4200 2600 50  0001 C CNN
F 4 "475-1145-1-ND" H 4200 2600 60  0001 C CNN "DK-PN"
	1    4200 2600
	0    1    1    0   
$EndComp
$Comp
L LED D4
U 1 1 5B468090
P 5000 2600
F 0 "D4" H 5000 2700 50  0000 C CNN
F 1 "LED" H 5000 2500 50  0000 C CNN
F 2 "BZKHS:LED" H 5000 2600 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 5000 2600 50  0001 C CNN
F 4 "475-1145-1-ND" H 5000 2600 60  0001 C CNN "DK-PN"
	1    5000 2600
	0    1    1    0   
$EndComp
$Comp
L LED D3
U 1 1 5B4680FD
P 5000 2200
F 0 "D3" H 5000 2300 50  0000 C CNN
F 1 "LED" H 5000 2100 50  0000 C CNN
F 2 "BZKHS:LED" H 5000 2200 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 5000 2200 50  0001 C CNN
F 4 "475-1145-1-ND" H 5000 2200 60  0001 C CNN "DK-PN"
	1    5000 2200
	0    1    1    0   
$EndComp
$Comp
L LED D5
U 1 1 5B46816B
P 5800 2200
F 0 "D5" H 5800 2300 50  0000 C CNN
F 1 "LED" H 5800 2100 50  0000 C CNN
F 2 "BZKHS:LED" H 5800 2200 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 5800 2200 50  0001 C CNN
F 4 "475-1145-1-ND" H 5800 2200 60  0001 C CNN "DK-PN"
	1    5800 2200
	0    1    1    0   
$EndComp
$Comp
L LED D6
U 1 1 5B4681DB
P 5800 2600
F 0 "D6" H 5800 2700 50  0000 C CNN
F 1 "LED" H 5800 2500 50  0000 C CNN
F 2 "BZKHS:LED" H 5800 2600 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 5800 2600 50  0001 C CNN
F 4 "475-1145-1-ND" H 5800 2600 60  0001 C CNN "DK-PN"
	1    5800 2600
	0    1    1    0   
$EndComp
$Comp
L LED D7
U 1 1 5B468250
P 6600 2200
F 0 "D7" H 6600 2300 50  0000 C CNN
F 1 "LED" H 6600 2100 50  0000 C CNN
F 2 "BZKHS:LED" H 6600 2200 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 6600 2200 50  0001 C CNN
F 4 "475-1145-1-ND" H 6600 2200 60  0001 C CNN "DK-PN"
	1    6600 2200
	0    1    1    0   
$EndComp
$Comp
L LED D8
U 1 1 5B46839A
P 6600 2600
F 0 "D8" H 6600 2700 50  0000 C CNN
F 1 "LED" H 6600 2500 50  0000 C CNN
F 2 "BZKHS:LED" H 6600 2600 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 6600 2600 50  0001 C CNN
F 4 "475-1145-1-ND" H 6600 2600 60  0001 C CNN "DK-PN"
	1    6600 2600
	0    1    1    0   
$EndComp
$Comp
L LED D9
U 1 1 5B468417
P 7400 2200
F 0 "D9" H 7400 2300 50  0000 C CNN
F 1 "LED" H 7400 2100 50  0000 C CNN
F 2 "BZKHS:LED" H 7400 2200 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 7400 2200 50  0001 C CNN
F 4 "475-1145-1-ND" H 7400 2200 60  0001 C CNN "DK-PN"
	1    7400 2200
	0    1    1    0   
$EndComp
$Comp
L LED D10
U 1 1 5B468495
P 7400 2600
F 0 "D10" H 7400 2700 50  0000 C CNN
F 1 "LED" H 7400 2500 50  0000 C CNN
F 2 "BZKHS:LED" H 7400 2600 50  0001 C CNN
F 3 "https://dammedia.osram.info/media/resource/hires/osram-dam-2494571/LA%20T776.pdf" H 7400 2600 50  0001 C CNN
F 4 "475-1145-1-ND" H 7400 2600 60  0001 C CNN "DK-PN"
	1    7400 2600
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 5B468CDF
P 4200 3100
F 0 "R8" V 4280 3100 50  0000 C CNN
F 1 "50R" V 4200 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 4130 3100 50  0001 C CNN
F 3 "" H 4200 3100 50  0001 C CNN
	1    4200 3100
	-1   0    0    1   
$EndComp
$Comp
L R R9
U 1 1 5B468DED
P 5000 3100
F 0 "R9" V 5080 3100 50  0000 C CNN
F 1 "50R" V 5000 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 4930 3100 50  0001 C CNN
F 3 "" H 5000 3100 50  0001 C CNN
	1    5000 3100
	-1   0    0    1   
$EndComp
$Comp
L R R10
U 1 1 5B468E79
P 5800 3100
F 0 "R10" V 5880 3100 50  0000 C CNN
F 1 "50R" V 5800 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5730 3100 50  0001 C CNN
F 3 "" H 5800 3100 50  0001 C CNN
	1    5800 3100
	-1   0    0    1   
$EndComp
$Comp
L R R12
U 1 1 5B468F07
P 7400 3100
F 0 "R12" V 7480 3100 50  0000 C CNN
F 1 "50R" V 7400 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7330 3100 50  0001 C CNN
F 3 "" H 7400 3100 50  0001 C CNN
	1    7400 3100
	-1   0    0    1   
$EndComp
$Comp
L R R11
U 1 1 5B469137
P 6600 3100
F 0 "R11" V 6680 3100 50  0000 C CNN
F 1 "50R" V 6600 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6530 3100 50  0001 C CNN
F 3 "" H 6600 3100 50  0001 C CNN
	1    6600 3100
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 2950 4200 2750
Wire Wire Line
	5000 2750 5000 2950
Wire Wire Line
	5800 2950 5800 2750
Wire Wire Line
	6600 2750 6600 2950
Wire Wire Line
	7400 2950 7400 2750
$Comp
L Q_NPN_BEC Q2
U 1 1 5B469600
P 1200 4600
F 0 "Q2" H 1400 4650 50  0000 L CNN
F 1 "Q_NPN_BEC" H 1400 4550 50  0000 L CNN
F 2 "SF_Silicon-Standard:SOT23-3" H 1400 4700 50  0001 C CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 1200 4600 50  0001 C CNN
F 4 "BC846BLT3GOSCT-ND" H 1200 4600 60  0001 C CNN "DK-PN"
	1    1200 4600
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BEC Q3
U 1 1 5B469BF8
P 4300 1750
F 0 "Q3" H 4500 1800 50  0000 L CNN
F 1 "Q_NPN_BEC" H 4500 1700 50  0000 L CNN
F 2 "SF_Silicon-Standard:SOT23-3" H 4500 1850 50  0001 C CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 4300 1750 50  0001 C CNN
F 4 "BC846BLT3GOSCT-ND" H 4300 1750 60  0001 C CNN "DK-PN"
	1    4300 1750
	-1   0    0    1   
$EndComp
$Comp
L Q_NPN_BEC Q4
U 1 1 5B469CD6
P 5100 1450
F 0 "Q4" H 5300 1500 50  0000 L CNN
F 1 "Q_NPN_BEC" H 5300 1400 50  0000 L CNN
F 2 "SF_Silicon-Standard:SOT23-3" H 5300 1550 50  0001 C CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 5100 1450 50  0001 C CNN
F 4 "BC846BLT3GOSCT-ND" H 5100 1450 60  0001 C CNN "DK-PN"
	1    5100 1450
	-1   0    0    1   
$EndComp
$Comp
L Q_NPN_BEC Q5
U 1 1 5B469D69
P 5900 1750
F 0 "Q5" H 6100 1800 50  0000 L CNN
F 1 "Q_NPN_BEC" H 6100 1700 50  0000 L CNN
F 2 "SF_Silicon-Standard:SOT23-3" H 6100 1850 50  0001 C CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 5900 1750 50  0001 C CNN
F 4 "BC846BLT3GOSCT-ND" H 5900 1750 60  0001 C CNN "DK-PN"
	1    5900 1750
	-1   0    0    1   
$EndComp
$Comp
L Q_NPN_BEC Q6
U 1 1 5B469DF8
P 6700 1450
F 0 "Q6" H 6900 1500 50  0000 L CNN
F 1 "Q_NPN_BEC" H 6900 1400 50  0000 L CNN
F 2 "SF_Silicon-Standard:SOT23-3" H 6900 1550 50  0001 C CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 6700 1450 50  0001 C CNN
F 4 "BC846BLT3GOSCT-ND" H 6700 1450 60  0001 C CNN "DK-PN"
	1    6700 1450
	-1   0    0    1   
$EndComp
$Comp
L Q_NPN_BEC Q7
U 1 1 5B469E8F
P 7500 1750
F 0 "Q7" H 7700 1800 50  0000 L CNN
F 1 "Q_NPN_BEC" H 7700 1700 50  0000 L CNN
F 2 "SF_Silicon-Standard:SOT23-3" H 7700 1850 50  0001 C CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 7500 1750 50  0001 C CNN
F 4 "BC846BLT3GOSCT-ND" H 7500 1750 60  0001 C CNN "DK-PN"
	1    7500 1750
	-1   0    0    1   
$EndComp
$Comp
L Q_NPN_BEC Q1
U 1 1 5B469FC8
P 1200 3550
F 0 "Q1" H 1400 3600 50  0000 L CNN
F 1 "Q_NPN_BEC" H 1400 3500 50  0000 L CNN
F 2 "SF_Silicon-Standard:SOT23-3" H 1400 3650 50  0001 C CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 1200 3550 50  0001 C CNN
F 4 "BC846BLT3GOSCT-ND" H 1200 3550 60  0001 C CNN "DK-PN"
	1    1200 3550
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 SW1
U 1 1 5B46B39D
P 4050 650
F 0 "SW1" H 4050 850 50  0000 C CNN
F 1 "DPDT" H 4050 450 50  0000 C CNN
F 2 "BZKHS:JS202011JAQN" H 4050 650 50  0001 C CNN
F 3 "https://www.ckswitches.com/media/1422/js.pdf" H 4050 650 50  0001 C CNN
F 4 "CKN10722CT-ND" H 4050 650 60  0001 C CNN "DK-PN"
F 5 "JS202011JAQN" H 4050 650 60  0001 C CNN "PN"
	1    4050 650 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3950 900  3950 850 
Wire Wire Line
	4050 900  4050 850 
NoConn ~ 4150 850 
$EndSCHEMATC
