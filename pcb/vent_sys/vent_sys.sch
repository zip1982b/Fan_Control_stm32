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
L vent_sys-rescue:stm32f103c8t6-module-china-stm32f103c8t6-module-china U2
U 1 1 5C5569F8
P 5700 3100
F 0 "U2" H 5700 4837 60  0000 C CNN
F 1 "stm32f103c8t6-module-china" H 5700 4731 60  0000 C CNN
F 2 "Module:Maple_Mini" H 5700 1400 60  0001 C CNN
F 3 "" H 5300 3350 60  0000 C CNN
	1    5700 3100
	1    0    0    -1  
$EndComp
$Comp
L Isolator:PC817 U1
U 1 1 5C59C113
P 2150 3150
F 0 "U1" H 2150 3475 50  0000 C CNN
F 1 "PC817" H 2150 3384 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 1950 2950 50  0001 L CIN
F 3 "http://www.soselectronic.cz/a_info/resource/d/pc817.pdf" H 2150 3150 50  0001 L CNN
	1    2150 3150
	1    0    0    -1  
$EndComp
$Comp
L Converter_ACDC:HLK-PM01(03) PS1
U 1 1 5C59C6AB
P 2850 5700
F 0 "PS1" H 2850 6315 50  0000 C CNN
F 1 "HLK-PM01(03)" H 2850 6224 50  0000 C CNN
F 2 "Converter_ACDC:HLK-PM01" H 2850 5700 50  0001 C CNN
F 3 "" H 2850 5700 50  0001 C CNN
	1    2850 5700
	1    0    0    -1  
$EndComp
$Comp
L RF_Module:RA-02 U4
U 1 1 5C5AE3F2
P 9000 1200
F 0 "U4" H 9000 1725 50  0000 C CNN
F 1 "RA-02" H 9000 1634 50  0000 C CNN
F 2 "RF_Module:Ai-Thinker-Ra-01-LoRa" H 9000 1200 50  0001 C CNN
F 3 "" H 9000 1200 50  0001 C CNN
	1    9000 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5C5AEB4D
P 2800 2800
F 0 "R5" H 2870 2846 50  0000 L CNN
F 1 "5K" H 2870 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2730 2800 50  0001 C CNN
F 3 "~" H 2800 2800 50  0001 C CNN
	1    2800 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5C5B04F2
P 1350 3350
F 0 "R3" H 1420 3396 50  0000 L CNN
F 1 "82K" H 1420 3305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1280 3350 50  0001 C CNN
F 3 "~" H 1350 3350 50  0001 C CNN
	1    1350 3350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 5C5B058A
P 1350 2900
F 0 "R2" H 1420 2946 50  0000 L CNN
F 1 "82K" H 1420 2855 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1280 2900 50  0001 C CNN
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
L power:GND #PWR01
U 1 1 5C5B39D8
P 2650 3350
F 0 "#PWR01" H 2650 3100 50  0001 C CNN
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
Wire Wire Line
	1200 3350 1150 3350
Wire Wire Line
	1150 3350 1150 3750
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
Wire Wire Line
	8500 900  8400 900 
Wire Wire Line
	8500 1000 8400 1000
Wire Wire Line
	8400 1000 8400 900 
Text Notes 1200 5600 0    50   ~ 0
L
Text Notes 1200 5700 0    50   ~ 0
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
Text Label 10500 1100 0    50   ~ 0
MOSI
Text Label 10500 1200 0    50   ~ 0
MISO
Text Label 9850 1300 0    50   ~ 0
SCK
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
L Device:C C2
U 1 1 5C65C4D9
P 10150 5300
F 0 "C2" H 10265 5346 50  0000 L CNN
F 1 "0.01uF - 300V" H 10265 5255 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L9.0mm_W9.8mm_P7.50mm_MKT" H 10188 5150 50  0001 C CNN
F 3 "~" H 10150 5300 50  0001 C CNN
	1    10150 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5C65C42F
P 10150 4850
F 0 "R9" H 10220 4896 50  0000 L CNN
F 1 "39" H 10220 4805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10080 4850 50  0001 C CNN
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
L Device:R R7
U 1 1 5C6375D4
P 8350 5050
F 0 "R7" V 8557 5050 50  0000 C CNN
F 1 "470" V 8466 5050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8280 5050 50  0001 C CNN
F 3 "~" H 8350 5050 50  0001 C CNN
	1    8350 5050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 5C5F3D32
P 8900 5050
F 0 "R8" V 9107 5050 50  0000 C CNN
F 1 "360" V 9016 5050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8830 5050 50  0001 C CNN
F 3 "~" H 8900 5050 50  0001 C CNN
	1    8900 5050
	0    -1   -1   0   
$EndComp
Text Notes 10950 5650 0    50   ~ 0
FAN
$Comp
L Device:C C1
U 1 1 5C5B07FB
P 8650 5300
F 0 "C1" H 8765 5346 50  0000 L CNN
F 1 "0.05uF - 300V" H 8765 5255 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L10.0mm_W2.5mm_P7.50mm_MKS4" H 8688 5150 50  0001 C CNN
F 3 "~" H 8650 5300 50  0001 C CNN
	1    8650 5300
	1    0    0    -1  
$EndComp
$Comp
L Triac_Thyristor:BT138-600 Q1
U 1 1 5C59C0A7
P 9500 5100
F 0 "Q1" H 9629 5146 50  0000 L CNN
F 1 "BT138-600" H 9629 5055 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9700 5025 50  0001 L CIN
F 3 "http://www.nxp.com/documents/data_sheet/BT138_SER_D_E.pdf" H 9500 5100 50  0001 L CNN
	1    9500 5100
	1    0    0    -1  
$EndComp
$Comp
L Relay_SolidState:MOC3021M U3
U 1 1 5C59BFB3
P 7800 5400
F 0 "U3" H 7800 5725 50  0000 C CNN
F 1 "MOC3021M" H 7800 5634 50  0000 C CNN
F 2 "Package_DIP:DIP-6_W10.16mm" H 7600 5200 50  0001 L CIN
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
L Device:R R6
U 1 1 5C906C4B
P 7200 5300
F 0 "R6" V 7407 5300 50  0000 C CNN
F 1 "510" V 7316 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7130 5300 50  0001 C CNN
F 3 "~" H 7200 5300 50  0001 C CNN
	1    7200 5300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7350 5300 7500 5300
Connection ~ 2800 3050
Entry Wire Line
	9200 2100 9300 2200
Wire Wire Line
	8500 1200 8000 1200
Wire Wire Line
	8000 1200 8000 2000
Entry Wire Line
	8000 2000 8100 2100
Text Label 7300 3800 0    50   ~ 0
Reset
Text Label 8050 1200 0    50   ~ 0
Reset
Wire Wire Line
	6750 3800 9300 3800
Wire Wire Line
	9300 3800 9300 2200
Entry Wire Line
	9100 2100 9200 2200
Wire Wire Line
	9200 2200 9200 3650
Wire Wire Line
	9200 3650 6750 3650
Text Label 7300 3650 0    50   ~ 0
nIRQ
Wire Wire Line
	8500 1300 8150 1300
Wire Wire Line
	8150 1300 8150 2000
Entry Wire Line
	8150 2000 8250 2100
Text Label 8200 1300 0    50   ~ 0
nIRQ
$Comp
L Device:R R1
U 1 1 5C931572
P 1350 2650
F 0 "R1" V 1143 2650 50  0000 C CNN
F 1 "82K" V 1234 2650 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1280 2650 50  0001 C CNN
F 3 "~" H 1350 2650 50  0001 C CNN
	1    1350 2650
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5C9315D2
P 1350 3750
F 0 "R4" V 1143 3750 50  0000 C CNN
F 1 "82K" V 1234 3750 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1280 3750 50  0001 C CNN
F 3 "~" H 1350 3750 50  0001 C CNN
	1    1350 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	1200 2650 950  2650
Wire Wire Line
	950  2650 950  2900
Connection ~ 950  2900
Wire Wire Line
	1500 3750 1800 3750
Wire Wire Line
	1800 3750 1800 3350
Connection ~ 1800 3350
Wire Wire Line
	1200 3750 1150 3750
Connection ~ 1150 3750
Wire Wire Line
	1150 3750 1150 5150
Wire Wire Line
	1500 2900 1800 2900
Wire Wire Line
	1500 2650 1800 2650
Wire Wire Line
	1800 2650 1800 2900
Connection ~ 1800 2900
NoConn ~ 6750 4550
NoConn ~ 6750 4400
NoConn ~ 6750 4250
NoConn ~ 6750 3950
NoConn ~ 4650 1850
NoConn ~ 4650 2000
NoConn ~ 4650 2150
NoConn ~ 4650 2300
NoConn ~ 4650 2450
NoConn ~ 4650 2600
NoConn ~ 4650 2750
NoConn ~ 4650 2900
NoConn ~ 4650 3050
NoConn ~ 4650 3200
NoConn ~ 4650 3350
NoConn ~ 4650 3500
NoConn ~ 4650 3650
NoConn ~ 4650 3800
NoConn ~ 4650 3950
NoConn ~ 4650 4100
NoConn ~ 4650 4250
NoConn ~ 6750 2750
NoConn ~ 6750 2600
NoConn ~ 6750 2300
NoConn ~ 6750 2150
NoConn ~ 10900 3300
NoConn ~ 9500 1400
NoConn ~ 9500 1500
NoConn ~ 8500 1500
NoConn ~ 8500 1600
Wire Wire Line
	4650 1700 3500 1700
Wire Wire Line
	3500 1700 3500 3050
Wire Wire Line
	2800 3050 3500 3050
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 5C92A326
P 10900 5550
F 0 "J2" H 10980 5542 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 10980 5451 50  0000 L CNN
F 2 "Connector_Phoenix_MC_HighVoltage:PhoenixContact_MCV_1,5_2-G-5.08_1x02_P5.08mm_Vertical" H 10900 5550 50  0001 C CNN
F 3 "~" H 10900 5550 50  0001 C CNN
	1    10900 5550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 5C92A382
P 950 5700
F 0 "J1" H 870 5375 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 870 5466 50  0000 C CNN
F 2 "Connector_Phoenix_MC_HighVoltage:PhoenixContact_MCV_1,5_2-G-5.08_1x02_P5.08mm_Vertical" H 950 5700 50  0001 C CNN
F 3 "~" H 950 5700 50  0001 C CNN
	1    950  5700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5C986FC8
P 7150 5600
F 0 "#PWR0103" H 7150 5350 50  0001 C CNN
F 1 "GND" H 7155 5427 50  0000 C CNN
F 2 "" H 7150 5600 50  0001 C CNN
F 3 "" H 7150 5600 50  0001 C CNN
	1    7150 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 5500 7150 5600
Wire Wire Line
	3400 6000 3950 6000
Wire Wire Line
	3400 5400 3700 5400
$Comp
L power:GND #PWR0101
U 1 1 5C9AEA36
P 3700 5500
F 0 "#PWR0101" H 3700 5250 50  0001 C CNN
F 1 "GND" H 3705 5327 50  0000 C CNN
F 2 "" H 3700 5500 50  0001 C CNN
F 3 "" H 3700 5500 50  0001 C CNN
	1    3700 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5500 3700 5400
Wire Wire Line
	7150 5500 7500 5500
Wire Wire Line
	9500 900  9500 600 
Connection ~ 8400 900 
Wire Wire Line
	8400 600  8400 900 
Wire Wire Line
	9500 600  8400 600 
Wire Wire Line
	6900 1850 6900 1700
Wire Wire Line
	4650 4400 4000 4400
Wire Wire Line
	6750 1850 6900 1850
Wire Wire Line
	6750 1700 6900 1700
Connection ~ 6900 1700
$Comp
L power:VCC #PWR0107
U 1 1 5C98C187
P 3950 5900
F 0 "#PWR0107" H 3950 5750 50  0001 C CNN
F 1 "VCC" H 3967 6073 50  0000 C CNN
F 2 "" H 3950 5900 50  0001 C CNN
F 3 "" H 3950 5900 50  0001 C CNN
	1    3950 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 5900 3950 6000
Connection ~ 3950 6000
Wire Wire Line
	3950 6000 4300 6000
Wire Wire Line
	7700 1100 8500 1100
Wire Wire Line
	4000 4400 4000 5400
Wire Wire Line
	4000 5400 3700 5400
Connection ~ 3700 5400
Wire Wire Line
	6900 600  8400 600 
Wire Wire Line
	6900 600  6900 1700
Connection ~ 8400 600 
Wire Wire Line
	9500 1600 9800 1600
Wire Wire Line
	9800 1600 9800 600 
Wire Wire Line
	9800 600  9500 600 
Connection ~ 9500 600 
Wire Wire Line
	7700 2000 7700 1100
Wire Wire Line
	6750 2000 7700 2000
Wire Wire Line
	2800 2650 2800 1100
Wire Wire Line
	2800 1100 7700 1100
Connection ~ 7700 1100
NoConn ~ 6750 2450
Wire Wire Line
	6750 4100 6900 4100
Wire Wire Line
	6900 4100 6900 5300
Wire Wire Line
	6900 5300 7050 5300
$Comp
L Device:R R10
U 1 1 5C980FE3
P 10250 1100
F 0 "R10" H 10320 1146 50  0000 L CNN
F 1 "5K" H 10320 1055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 10180 1100 50  0001 C CNN
F 3 "~" H 10250 1100 50  0001 C CNN
	1    10250 1100
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R11
U 1 1 5C9A026E
P 10250 1200
F 0 "R11" H 10320 1246 50  0000 L CNN
F 1 "5K" H 10320 1155 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 10180 1200 50  0001 C CNN
F 3 "~" H 10250 1200 50  0001 C CNN
	1    10250 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	10400 1100 10700 1100
Wire Wire Line
	10700 1200 10400 1200
Wire Wire Line
	9500 1100 10100 1100
Wire Wire Line
	10100 1200 9500 1200
Wire Bus Line
	10800 750  10800 2100
Wire Bus Line
	8000 2100 10800 2100
$EndSCHEMATC
