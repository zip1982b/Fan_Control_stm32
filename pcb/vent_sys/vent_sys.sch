EESchema Schematic File Version 4
LIBS:vent_sys-cache
EELAYER 26 0
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
L stm32f103c8t6-module-china:stm32f103c8t6-module-china U?
U 1 1 5C5569F8
P 5700 3100
F 0 "U?" H 5700 4837 60  0000 C CNN
F 1 "stm32f103c8t6-module-china" H 5700 4731 60  0000 C CNN
F 2 "myelin-kicad:stm32f103c8t6-module-china" H 5700 1400 60  0001 C CNN
F 3 "" H 5300 3350 60  0000 C CNN
	1    5700 3100
	1    0    0    -1  
$EndComp
$Comp
L Isolator:PC817 U?
U 1 1 5C59C113
P 2150 3150
F 0 "U?" H 2150 3475 50  0000 C CNN
F 1 "PC817" H 2150 3384 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 1950 2950 50  0001 L CIN
F 3 "http://www.soselectronic.cz/a_info/resource/d/pc817.pdf" H 2150 3150 50  0001 L CNN
	1    2150 3150
	1    0    0    -1  
$EndComp
$Comp
L Converter_ACDC:HLK-PM01(03) PS?
U 1 1 5C59C6AB
P 2850 5700
F 0 "PS?" H 2850 6315 50  0000 C CNN
F 1 "HLK-PM01(03)" H 2850 6224 50  0000 C CNN
F 2 "" H 2850 5700 50  0001 C CNN
F 3 "" H 2850 5700 50  0001 C CNN
	1    2850 5700
	1    0    0    -1  
$EndComp
$Comp
L RF_Module:RA-02 U?
U 1 1 5C5AE3F2
P 9000 1200
F 0 "U?" H 9000 1725 50  0000 C CNN
F 1 "RA-02" H 9000 1634 50  0000 C CNN
F 2 "" H 9000 1200 50  0001 C CNN
F 3 "" H 9000 1200 50  0001 C CNN
	1    9000 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5C5AEB4D
P 2800 2800
F 0 "R?" H 2870 2846 50  0000 L CNN
F 1 "R" H 2870 2755 50  0000 L CNN
F 2 "" V 2730 2800 50  0001 C CNN
F 3 "~" H 2800 2800 50  0001 C CNN
	1    2800 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C5AFA20
P 3650 5400
F 0 "#PWR?" H 3650 5150 50  0001 C CNN
F 1 "GND" H 3655 5227 50  0000 C CNN
F 2 "" H 3650 5400 50  0001 C CNN
F 3 "" H 3650 5400 50  0001 C CNN
	1    3650 5400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5C5AFAD4
P 3650 6000
F 0 "#PWR?" H 3650 5850 50  0001 C CNN
F 1 "+3.3V" H 3665 6173 50  0000 C CNN
F 2 "" H 3650 6000 50  0001 C CNN
F 3 "" H 3650 6000 50  0001 C CNN
	1    3650 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 6000 3650 6000
Wire Wire Line
	3400 5400 3650 5400
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5C5B0053
P 950 5700
F 0 "J?" H 844 5375 50  0000 C CNN
F 1 "Conn_01x02_Female" H 844 5466 50  0000 C CNN
F 2 "" H 950 5700 50  0001 C CNN
F 3 "~" H 950 5700 50  0001 C CNN
	1    950  5700
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 5C5B04F2
P 1350 3350
F 0 "R?" H 1420 3396 50  0000 L CNN
F 1 "R" H 1420 3305 50  0000 L CNN
F 2 "" V 1280 3350 50  0001 C CNN
F 3 "~" H 1350 3350 50  0001 C CNN
	1    1350 3350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5C5B058A
P 1350 2900
F 0 "R?" H 1420 2946 50  0000 L CNN
F 1 "R" H 1420 2855 50  0000 L CNN
F 2 "" V 1280 2900 50  0001 C CNN
F 3 "~" H 1350 2900 50  0001 C CNN
	1    1350 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	1150 5600 1500 5600
Wire Wire Line
	1500 5600 1500 5400
Wire Wire Line
	1500 5400 1650 5400
Wire Wire Line
	1150 5700 1500 5700
Wire Wire Line
	1500 5700 1500 6000
Wire Wire Line
	1500 6000 2000 6000
Wire Wire Line
	4650 4550 4300 4550
Wire Wire Line
	4300 4550 4300 6000
Wire Wire Line
	4300 6000 4000 6000
Connection ~ 3650 6000
$Comp
L power:GND #PWR?
U 1 1 5C5B0D6D
P 7250 1500
F 0 "#PWR?" H 7250 1250 50  0001 C CNN
F 1 "GND" H 7255 1327 50  0000 C CNN
F 2 "" H 7250 1500 50  0001 C CNN
F 3 "" H 7250 1500 50  0001 C CNN
	1    7250 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 1700 6900 1700
Wire Wire Line
	6900 1700 6900 1400
Wire Wire Line
	6900 1400 7250 1400
Wire Wire Line
	7250 1400 7250 1500
Wire Wire Line
	6750 1850 6900 1850
Wire Wire Line
	6900 1850 6900 1700
Connection ~ 6900 1700
Wire Wire Line
	4650 4400 3900 4400
Wire Wire Line
	3900 4400 3900 5100
Wire Wire Line
	3900 5400 3650 5400
Connection ~ 3650 5400
Wire Wire Line
	1500 2900 1800 2900
Wire Wire Line
	1800 2900 1800 3050
Wire Wire Line
	1800 3050 1850 3050
Wire Wire Line
	1500 3350 1800 3350
Wire Wire Line
	1800 3350 1800 3250
Wire Wire Line
	1800 3250 1850 3250
$Comp
L power:GND #PWR?
U 1 1 5C5B39D8
P 2650 3350
F 0 "#PWR?" H 2650 3100 50  0001 C CNN
F 1 "GND" H 2655 3177 50  0000 C CNN
F 2 "" H 2650 3350 50  0001 C CNN
F 3 "" H 2650 3350 50  0001 C CNN
	1    2650 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3250 2650 3250
Wire Wire Line
	2650 3250 2650 3350
Wire Wire Line
	2450 3050 2800 3050
Wire Wire Line
	2800 3050 2800 2950
$Comp
L power:+3.3V #PWR?
U 1 1 5C5B4551
P 2800 2500
F 0 "#PWR?" H 2800 2350 50  0001 C CNN
F 1 "+3.3V" H 2815 2673 50  0000 C CNN
F 2 "" H 2800 2500 50  0001 C CNN
F 3 "" H 2800 2500 50  0001 C CNN
	1    2800 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 2650 2800 2500
$Comp
L power:+3.3V #PWR?
U 1 1 5C5B5B03
P 7500 1800
F 0 "#PWR?" H 7500 1650 50  0001 C CNN
F 1 "+3.3V" H 7515 1973 50  0000 C CNN
F 2 "" H 7500 1800 50  0001 C CNN
F 3 "" H 7500 1800 50  0001 C CNN
	1    7500 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2000 7500 2000
Wire Wire Line
	7500 2000 7500 1800
Wire Wire Line
	1200 3350 1150 3350
Wire Wire Line
	1150 3350 1150 5150
Wire Wire Line
	1150 5150 2000 5150
Wire Wire Line
	2000 5150 2000 6000
Connection ~ 2000 6000
Wire Wire Line
	2000 6000 2300 6000
Wire Wire Line
	1200 2900 950  2900
Wire Wire Line
	950  2900 950  5300
Wire Wire Line
	950  5300 1650 5300
Wire Wire Line
	1650 5300 1650 5400
Connection ~ 1650 5400
Wire Wire Line
	1650 5400 2300 5400
$Comp
L power:GND #PWR?
U 1 1 5C5D7125
P 7750 850
F 0 "#PWR?" H 7750 600 50  0001 C CNN
F 1 "GND" H 7755 677 50  0000 C CNN
F 2 "" H 7750 850 50  0001 C CNN
F 3 "" H 7750 850 50  0001 C CNN
	1    7750 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 900  8400 900 
Wire Wire Line
	8400 900  8400 750 
Wire Wire Line
	8400 750  7750 750 
Wire Wire Line
	7750 750  7750 850 
Wire Wire Line
	8500 1000 8400 1000
Wire Wire Line
	8400 1000 8400 900 
Connection ~ 8400 900 
$Comp
L power:+3.3V #PWR?
U 1 1 5C5DA605
P 8000 1100
F 0 "#PWR?" H 8000 950 50  0001 C CNN
F 1 "+3.3V" H 8015 1273 50  0000 C CNN
F 2 "" H 8000 1100 50  0001 C CNN
F 3 "" H 8000 1100 50  0001 C CNN
	1    8000 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 1100 8500 1100
Text Notes 850  5600 0    50   ~ 0
L
Text Notes 850  5750 0    50   ~ 0
N
Wire Wire Line
	6750 3350 9000 3350
Wire Wire Line
	9000 3350 9000 2200
Entry Wire Line
	8900 2100 9000 2200
Entry Wire Line
	8800 2100 8900 2200
Entry Wire Line
	8700 2100 8800 2200
Entry Wire Line
	8600 2100 8700 2200
Wire Wire Line
	6750 3200 8900 3200
Wire Wire Line
	8900 3200 8900 2200
Wire Wire Line
	6750 3050 8800 3050
Wire Wire Line
	8800 3050 8800 2200
Wire Wire Line
	6750 2900 8700 2900
Wire Wire Line
	8700 2900 8700 2200
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5C8AB50F
P 4000 5800
F 0 "#FLG?" H 4000 5875 50  0001 C CNN
F 1 "PWR_FLAG" H 4000 5974 50  0000 C CNN
F 2 "" H 4000 5800 50  0001 C CNN
F 3 "~" H 4000 5800 50  0001 C CNN
	1    4000 5800
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5C8AB681
P 3650 5200
F 0 "#FLG?" H 3650 5275 50  0001 C CNN
F 1 "PWR_FLAG" H 3650 5374 50  0000 C CNN
F 2 "" H 3650 5200 50  0001 C CNN
F 3 "~" H 3650 5200 50  0001 C CNN
	1    3650 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 5800 4000 6000
Connection ~ 4000 6000
Wire Wire Line
	4000 6000 3650 6000
Wire Wire Line
	3650 5200 3650 5400
Text Label 7300 2900 0    50   ~ 0
MOSI
Text Label 7300 3050 0    50   ~ 0
MISO
Text Label 7300 3200 0    50   ~ 0
SCK
Text Label 7300 3350 0    50   ~ 0
NSS
Wire Wire Line
	9500 1000 10700 1000
Wire Wire Line
	9500 1100 10700 1100
Wire Wire Line
	9500 1200 10700 1200
Wire Wire Line
	9500 1300 10700 1300
Entry Wire Line
	10700 1000 10800 1100
Entry Wire Line
	10700 1100 10800 1200
Entry Wire Line
	10700 1200 10800 1300
Entry Wire Line
	10700 1300 10800 1400
Text Label 9850 1000 0    50   ~ 0
NSS
Text Label 9850 1100 0    50   ~ 0
MOSI
Text Label 9850 1200 0    50   ~ 0
MISO
Text Label 9850 1300 0    50   ~ 0
SCK
$Comp
L power:GND #PWR?
U 1 1 5C8D193A
P 9600 1700
F 0 "#PWR?" H 9600 1450 50  0001 C CNN
F 1 "GND" H 9605 1527 50  0000 C CNN
F 2 "" H 9600 1700 50  0001 C CNN
F 3 "" H 9600 1700 50  0001 C CNN
	1    9600 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 1600 9600 1600
Wire Wire Line
	9600 1600 9600 1700
$Comp
L power:GND #PWR?
U 1 1 5C8D5889
P 9750 700
F 0 "#PWR?" H 9750 450 50  0001 C CNN
F 1 "GND" H 9755 527 50  0000 C CNN
F 2 "" H 9750 700 50  0001 C CNN
F 3 "" H 9750 700 50  0001 C CNN
	1    9750 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 900  9500 600 
Wire Wire Line
	9500 600  9750 600 
Wire Wire Line
	9750 600  9750 700 
Wire Wire Line
	6750 3500 9100 3500
Wire Wire Line
	9100 3500 9100 2200
Entry Wire Line
	9000 2100 9100 2200
Text Label 7300 3500 0    50   ~ 0
DIO1
Wire Wire Line
	8500 1400 8300 1400
Wire Wire Line
	8300 1400 8300 2000
Entry Wire Line
	8300 2000 8400 2100
Text Label 8300 1400 0    50   ~ 0
DIO1
$Comp
L Device:C C?
U 1 1 5C65C4D9
P 10150 5300
F 0 "C?" H 10265 5346 50  0000 L CNN
F 1 "0.01uF - 300V" H 10265 5255 50  0000 L CNN
F 2 "" H 10188 5150 50  0001 C CNN
F 3 "~" H 10150 5300 50  0001 C CNN
	1    10150 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5C65C42F
P 10150 4850
F 0 "R?" H 10220 4896 50  0000 L CNN
F 1 "39" H 10220 4805 50  0000 L CNN
F 2 "" V 10080 4850 50  0001 C CNN
F 3 "~" H 10150 4850 50  0001 C CNN
	1    10150 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 5300 8100 5300
Wire Wire Line
	8150 5050 8150 5300
Wire Wire Line
	8200 5050 8150 5050
Wire Wire Line
	8650 5050 8500 5050
Wire Wire Line
	8650 5050 8650 5150
Connection ~ 8650 5050
Wire Wire Line
	8750 5050 8650 5050
$Comp
L Device:R R?
U 1 1 5C6375D4
P 8350 5050
F 0 "R?" V 8557 5050 50  0000 C CNN
F 1 "470" V 8466 5050 50  0000 C CNN
F 2 "" V 8280 5050 50  0001 C CNN
F 3 "~" H 8350 5050 50  0001 C CNN
	1    8350 5050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5C5F3D32
P 8900 5050
F 0 "R?" V 9107 5050 50  0000 C CNN
F 1 "360" V 9016 5050 50  0000 C CNN
F 2 "" V 8830 5050 50  0001 C CNN
F 3 "~" H 8900 5050 50  0001 C CNN
	1    8900 5050
	0    -1   -1   0   
$EndComp
Text Notes 10950 5650 0    50   ~ 0
FAN
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5C5DC3CB
P 10900 5550
F 0 "J?" H 10794 5225 50  0000 C CNN
F 1 "Conn_01x02_Female" H 10794 5316 50  0000 C CNN
F 2 "" H 10900 5550 50  0001 C CNN
F 3 "~" H 10900 5550 50  0001 C CNN
	1    10900 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C5B07FB
P 8650 5300
F 0 "C?" H 8765 5346 50  0000 L CNN
F 1 "0.05uF - 300V" H 8765 5255 50  0000 L CNN
F 2 "" H 8688 5150 50  0001 C CNN
F 3 "~" H 8650 5300 50  0001 C CNN
	1    8650 5300
	1    0    0    -1  
$EndComp
$Comp
L Triac_Thyristor:BT138-600 Q?
U 1 1 5C59C0A7
P 9500 5100
F 0 "Q?" H 9629 5146 50  0000 L CNN
F 1 "BT138-600" H 9629 5055 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9700 5025 50  0001 L CIN
F 3 "http://www.nxp.com/documents/data_sheet/BT138_SER_D_E.pdf" H 9500 5100 50  0001 L CNN
	1    9500 5100
	1    0    0    -1  
$EndComp
$Comp
L Relay_SolidState:MOC3021M U?
U 1 1 5C59BFB3
P 7800 5400
F 0 "U?" H 7800 5725 50  0000 C CNN
F 1 "MOC3021M" H 7800 5634 50  0000 C CNN
F 2 "" H 7600 5200 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/MO/MOC3020M.pdf" H 7775 5400 50  0001 L CNN
	1    7800 5400
	1    0    0    -1  
$EndComp
Text Label 2200 5400 0    50   ~ 0
L
Text Label 2200 6000 0    50   ~ 0
N
Wire Wire Line
	10700 5550 10150 5550
Wire Wire Line
	10150 5550 10150 5450
Wire Wire Line
	10150 5150 10150 5000
Wire Wire Line
	10150 4700 9500 4700
Wire Wire Line
	9100 4700 9100 5050
Wire Wire Line
	9100 5050 9050 5050
Wire Wire Line
	9500 4950 9500 4700
Connection ~ 9500 4700
Wire Wire Line
	9500 4700 9100 4700
Wire Wire Line
	8650 5450 8650 5550
Wire Wire Line
	8650 5550 9500 5550
Connection ~ 10150 5550
Wire Wire Line
	9500 5250 9500 5550
Connection ~ 9500 5550
Wire Wire Line
	9500 5550 10150 5550
Wire Wire Line
	8100 5500 9100 5500
Wire Wire Line
	9100 5500 9100 5200
Wire Wire Line
	9100 5200 9350 5200
Wire Wire Line
	2000 6000 2000 6350
Wire Wire Line
	2000 6350 9500 6350
Wire Wire Line
	9500 6350 9500 5650
Wire Wire Line
	9500 5650 10700 5650
Wire Wire Line
	1650 5400 1650 6650
Wire Wire Line
	1650 6650 6450 6650
Wire Wire Line
	6450 6650 6450 4900
Wire Wire Line
	6450 4900 8100 4900
Wire Wire Line
	8100 4900 8100 4500
Wire Wire Line
	8100 4500 10150 4500
Wire Wire Line
	10150 4500 10150 4700
Connection ~ 10150 4700
$Comp
L Device:R R?
U 1 1 5C906C4B
P 7200 5300
F 0 "R?" V 7407 5300 50  0000 C CNN
F 1 "510" V 7316 5300 50  0000 C CNN
F 2 "" V 7130 5300 50  0001 C CNN
F 3 "~" H 7200 5300 50  0001 C CNN
	1    7200 5300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7350 5300 7500 5300
Text Label 6900 5300 0    50   ~ 0
FAN
Text Label 7100 2450 0    50   ~ 0
FAN
Wire Wire Line
	7050 5300 6900 5300
Wire Wire Line
	6750 2450 7100 2450
Wire Wire Line
	7500 5500 4500 5500
Wire Wire Line
	4500 5500 4500 5100
Wire Wire Line
	4500 5100 3900 5100
Connection ~ 3900 5100
Wire Wire Line
	3900 5100 3900 5400
Wire Wire Line
	3300 3050 2800 3050
Connection ~ 2800 3050
Text Label 3300 3050 0    50   ~ 0
Herz_couner
Wire Wire Line
	4650 1700 4200 1700
Text Label 4200 1700 0    50   ~ 0
Herz_counter
Wire Bus Line
	10800 750  10800 2100
Wire Bus Line
	8000 2100 10800 2100
$Comp
L maxim:DS2482-100 U?
U 1 1 5C5AE47D
P 2600 1300
F 0 "U?" H 2600 1978 50  0000 C CNN
F 1 "DS2482-100" H 2600 1887 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2650 900 50  0001 L CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/DS2482-100.pdf" H 2870 1550 50  0001 C CNN
	1    2600 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Variable R?
U 1 1 5C5AE99F
P 3400 1150
F 0 "R?" H 3528 1196 50  0000 L CNN
F 1 "R_Variable" H 3528 1105 50  0000 L CNN
F 2 "" V 3330 1150 50  0001 C CNN
F 3 "~" H 3400 1150 50  0001 C CNN
	1    3400 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5C5AEA85
P 800 1000
F 0 "R?" H 870 1046 50  0000 L CNN
F 1 "R" H 870 955 50  0000 L CNN
F 2 "" V 730 1000 50  0001 C CNN
F 3 "~" H 800 1000 50  0001 C CNN
	1    800  1000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Female J?
U 1 1 5C5B0332
P 4000 1400
F 0 "J?" H 4027 1426 50  0000 L CNN
F 1 "Conn_01x03_Female" H 4027 1335 50  0000 L CNN
F 2 "" H 4000 1400 50  0001 C CNN
F 3 "~" H 4000 1400 50  0001 C CNN
	1    4000 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5C5B7267
P 2950 700
F 0 "#PWR?" H 2950 550 50  0001 C CNN
F 1 "+3.3V" H 2965 873 50  0000 C CNN
F 2 "" H 2950 700 50  0001 C CNN
F 3 "" H 2950 700 50  0001 C CNN
	1    2950 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 800  2950 800 
Wire Wire Line
	2950 800  2950 750 
$Comp
L power:GND #PWR?
U 1 1 5C5B77FB
P 2600 1950
F 0 "#PWR?" H 2600 1700 50  0001 C CNN
F 1 "GND" H 2605 1777 50  0000 C CNN
F 2 "" H 2600 1950 50  0001 C CNN
F 3 "" H 2600 1950 50  0001 C CNN
	1    2600 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1800 2600 1850
NoConn ~ 3100 1100
$Comp
L Device:R R?
U 1 1 5C5B9E9D
P 1400 1000
F 0 "R?" H 1470 1046 50  0000 L CNN
F 1 "R" H 1470 955 50  0000 L CNN
F 2 "" V 1330 1000 50  0001 C CNN
F 3 "~" H 1400 1000 50  0001 C CNN
	1    1400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 750  2950 750 
Connection ~ 2950 750 
Wire Wire Line
	3400 750  2950 750 
Wire Wire Line
	2950 750  2950 700 
Wire Wire Line
	1400 850  1400 750 
Connection ~ 1400 750 
Wire Wire Line
	800  750  1400 750 
Wire Wire Line
	2100 1100 1700 1100
Wire Wire Line
	1700 1100 1700 1150
Wire Wire Line
	1700 1150 1400 1150
Wire Wire Line
	800  850  800  750 
Wire Wire Line
	800  1150 800  1250
Wire Wire Line
	800  1250 1900 1250
Wire Wire Line
	1900 1250 1900 1200
Wire Wire Line
	1900 1200 2100 1200
$Comp
L power:GND #PWR?
U 1 1 5C5C0A5C
P 1600 1550
F 0 "#PWR?" H 1600 1300 50  0001 C CNN
F 1 "GND" H 1605 1377 50  0000 C CNN
F 2 "" H 1600 1550 50  0001 C CNN
F 3 "" H 1600 1550 50  0001 C CNN
	1    1600 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1400 1600 1400
Wire Wire Line
	1600 1400 1600 1500
Wire Wire Line
	2100 1500 1600 1500
Connection ~ 1600 1500
Wire Wire Line
	1600 1500 1600 1550
Wire Wire Line
	3400 1000 3400 750 
Connection ~ 3400 750 
Wire Wire Line
	3700 750  3400 750 
Wire Wire Line
	3100 1400 3400 1400
Wire Wire Line
	3400 1400 3400 1300
Connection ~ 3400 1400
Wire Wire Line
	3800 1400 3400 1400
Wire Wire Line
	3800 1500 3800 1850
Wire Wire Line
	3800 1850 2600 1850
Connection ~ 2600 1850
Wire Wire Line
	2600 1850 2600 1950
Wire Wire Line
	3800 1300 3700 1300
Wire Wire Line
	3700 1300 3700 750 
$EndSCHEMATC
