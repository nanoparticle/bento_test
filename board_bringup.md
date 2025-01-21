# Board Bringup:
## Power Rails:
- **VBUS** - Connected directly to the USB C Connector VBUS pins, goes only through the power-path/reverse-polarity-protection diode to the +5V rail
- **VAUX** - External high-voltage power input, goes only through the eFuse IC to the VMOT rail
- **VMOT** - Protected motor power supply, connects primarily to the DRV8316C, but is also measured through a voltage divider by the MCU
- **VBUCK** - Output from the DRV8316C internal buck regulator, starts at 3.3V but is immediately configured to output 5V after MCU boots. Goes only through an ideal diode IC to the +5V rail
- **+5V** - Nominally 5V (though briefly ~3.3V when powered only by VAUX), this rail powers only the CAN transciever and 3.3V LDO
- **+3V3** - Nominally 3.3V (though briefly 3.3V minus the regulator dropout voltage when powered only by VAUX), this rail powers all the remaining logic on the board
	
## Equipment:
- Fluke 115 Multimeter
- BK Precision 9111 Adjustable PSU
- U96PB USB Tester

## Test Procedure:
### Initial state:
- MCU erased from factory
- All DIP switches off from factory
- Nothing plugged into any of the connectors except as stated 
### Visual inspection under microscope:
- <font color="green">PCB looks very good, and all components appear to be soldered correctly</font>
### Check resistance to GND of all power rails:
- First number that shows up is recorded, after that the resistance changes a lot as capacitors charge and ICs start waking up
- VBUS:	5MΩ
- VAUX:	10MΩ
- VMOT:	2MΩ
- VBUCK:	3MΩ
- +5V:	3MΩ
- +3V3:	50kΩ
- <font color="green">Result: No shorts, looks good</font>
### Plug only into 5V USB PSU through USB tester:
- Tester reading: 4.965V 0.042A
- VBUS:	4.97V
- VAUX:	0.00V
- VMOT:	0.00V
- VBUCK:	0.00V
- +5V:	4.66V
- +3V3:	3.30V
- <font color="green">Result: Looks good</font>
### Plug only into Vaux, DC PSU set to 5V with 0.1A current limit:
- PSU reading: 5.00V 0.045A
- VBUS:	3.19V (Likely from diode leakage current)
- VAUX:	4.99V
- VMOT:	4.99V
- VBUCK:	3.22V (Expected since DRV8316C buck regulator defaults to 3.3V)
- +5V:	3.22V
- +3V3:	3.20V
- <font color="green">Result: Looks good</font>
### Plug only into Vaux, DC PSU set to 24V with 0.1A current limit:
- PSU reading: 24.00V 0.031A
- VBUS:	3.20V (Likely from diode leakage current)
- VAUX:	23.99V
- VMOT:	23.99V
- VBUCK:	3.26V (Expected since DRV8316C buck regulator defaults to 3.3V)
- +5V:	3.25V
- +3V3:	3.23V
- <font color="green">Result: Looks good</font>
### Plug only into Vaux, DC PSU set to 36V with 0.1A current limit:
- PSU reading: 30.64V 0.099A (CC mode)
- TVS diode was getting hot so detailed measurement was skipped, but lowest absolute maximum voltage of all ICs on VAUX/VMOT is at least 40V so this is fine
- <font color="green">Result: Looks good</font>
### Plug only into Vaux, DC PSU set to -5V with 0.1A current limit:
- PSU reading: -0.91V 0.100A (CC mode)
- <font color="red">Result: reverse polarity protection test failed</font>
### Plug only into 5V USB PSU through USB tester and Vaux, DC PSU set to 5V with 0.1A current limit:
- Tester reading: 4.965V 0.042A
- PSU reading: 5.00 0.014A
- VBUS:	4.97V
- VAUX:	5.00V
- VMOT:	5.00V
- VBUCK:	3.23V
- +5V:	4.66V
- +3V3:	3.30V
- <font color="green">Result: Looks good</font>
### Plug only into 5V USB PSU through USB tester and Vaux, DC PSU set to 6V with 0.1A current limit:
- Tester reading: 4.965V 0.042A
- PSU reading: 6.00 0.014A
- VBUS:	4.97V
- VAUX:	6.00V
- VMOT:	5.99V
- VBUCK:	3.23V
- +5V:	4.67V
- +3V3:	3.30V
- <font color="green">Result: Looks good</font>
### Plug only into 5V USB PSU through USB tester and Vaux, DC PSU set to 24V with 0.1A current limit:
- Tester reading: 4.965V 0.042A
- PSU reading: 24.00 0.025A
- VBUS:	4.97V
- VAUX:	23.99V
- VMOT:	23.99V
- VBUCK:	3.25V
- +5V:	4.67V
- +3V3:	3.30V
- <font color="green">Result: Looks good</font>