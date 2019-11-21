# CE514Project-3




<h2> Hardware</h2>

The follows are all controlled by the Spark Core using a common I2C bus where pin D0 is SDA and pin D1 is SCL. The displays (from left to right) are on I2C addresses 0x70, 0x71, and 0x72.

<b>1. Temperature and humidity sensor BME280 </b>
Only four legs will be connected: VCC  (3V3), GND (GND), SCL (D1), SDA (D0). The latter two legs are for data exchange. it uses I2C protocol to store data of temperature and humidity.  We can find it needs a power supply (3.3 VDC), which can be provided by photon. And SCL and SDA should connect to the D1 and D0 in the photon.A reference to the temperature sensor: https://github.com/finitespace/BME280

<b>2. Adafruit Mini 8x8 LED Matrix w/I2C Backpack </b>
Mini 8x8 LED Matrix is used to show the set system parameters. This LED Matrix also supports I2C protocol, which means it should connect to the D0 and D1 of Photon. It also needs a power supply. The 3.3VDC could be provided by Photon. Figure 20 shows the layout of LED.
In addition, the default I2C address of Mini 8x8 LED Matrix is 0x70. Therefore, if we are going to use two LEDs in one bus, we should change the address of it. Figure 21 shows the back of the LED. We can change the address of a backpack very easily. Look on the back to find the two A0, A1 solder jumpers.

Library of Adafruit Mini 8x8 LED Matrix w/I2C Backpack can be found: https://github.com/adafruit/Adafruit_LED_Backpack

<b>3. Adafruit CCS811 Air Quality Sensor</b>
CCS811 sensor measures eCO2 (equivalent calculated carbon-dioxide) concentration within a range of 400 to 8192 parts per million (ppm), and TVOC (Total Volatile Organic Compound) concentration within a range of 0 to 1187 parts per billion (ppb). The default address is 0X5A.

Pins :
Vin: This is the power pin, the sensor uses 3.3V;
3V3: This is power output;
GND: Common ground for power and logic;
SCL: This is I2C protocol pin;
SDA: This is I2C protocol pin;
WAKE: It needs to be pulled to ground in order to communicate with the sensor;
RST: This is reset pin;	
INT: This is the interrupt-output pin (seldom use);
The recommended wire connection shows Fig23, WAKE should be connected to the ground.

Library and related application for CO2 sensor can be found:
https://learn.adafruit.com/adafruit-ccs811-air-quality-sensor/arduino-wiring-test


<h2>Software for the Particle</h2>
Online IDE
The detailed instruction can be found on this website. Figure26 shows the interface of IDE.  
https://docs.particle.io/guide/getting-started/build/photon/
There are three functions for the Web IDE. They are Flash, Verify and Save.
1	Hardware 
1.1	Photon Introduction
Photon is a cloud-based micro-controller platform. It is composed of a powerful ARM Cortex M3 microcontroller and a Broadcom Wi-Fi chip in a tiny thumbnail-sized module called the PØ (P-zero). The micro-controller is the brain for the device and run the application as you desired. It connects to sensors or devices through the IO board. Fig. 1 shows the outlook of the Photon and the diagram for the pins.
           
Fig. 1 Photon Structure
•	D0-D7 are digital input/output points. The type of input or output can be defined in the program.
•	A0-A5 are analog points of Photon. They can be defined by input or output in the program. 
•	DAC, WKP can also be used as analog points. 
•	The analog points can be converted to digital points in the program.
The link to https://docs.particle.io/datasheets/wi-fi/photon-datasheet/ is the comprehensive description of Photon. Photon can also read data through SPI/I2C telecom protocols.  In Fig. 2, we can see that D0 and D1 can be used to read I2C protocol, and D2-D5 can be used to read SPI protocol. In Fig. 3, we can see that A2 – A5 can be used to read SPI protocol.
 
Fig. 2 the Pins at the right side of the photon
 
Fig. 3 the Pins at the left side of the photon
Besides the microcontroller and WIFI-chip, Particle adds a rock solid 3.3VDC SMPS power supply, RF, and user interface components to the PØ on the Photon. The design is open source, so the users can easily integrate the Photon and sensors into their application.
Connect the photon with Internet
Follow the video: https://vimeo.com/178282058

1.2	Breadboard
An electronics breadboard is referring to a solderless breadboard. The backside of the board to show the circuit connections, each metal stripe will have the same current. Central portions, separated, are horizontally connected. Both sides are vertically connected, are supposed to be connected to power supply. When using the Photon, we can get 5V and 3.3 V output from the module. The “-“ is the ground voltage for reference. Please click the link here https://www.youtube.com/watch?v=fq6U5Y14oM4&t=3s to learn more.
 
Fig. 4 Breadboard structure
1.3	LED
Led light connection: longer leg in Fig. 5 is linked to positive of 3.3 V and the short leg to ground. 
 
Fig. 5 LED light
1.4	The 1st round Test procedures
Step 1: Power the device
Plug the USB cable into your power source: the power source can be power brick, your computer or another power source wired to the port on the module. Here we used the computer for the power supply. Then use the Photon to supply 3.3V (3V3) 5V (VIN) voltage to the breadboard, as shown in Fig. 5. 

      
Fig. 6 Power Photon
 
Figure 6 Power LED
Debugging:
•	If your Photon is not blinking blue, hold down the SETUP button.
•	If your Photon is not blinking at all, or if the LED is burning a dull orange color, it may not be getting enough power. Try changing your power source or USB cable.

Step 2: Connect the Photon to the internet using the smartphone
1.	Download the Particle App through the application store, as shown in Fig 6.
2.	Sign up an account with Particle, as shown in Fig 6.
3.	Press the plus icon and select the device you'd like to add. Then follow the instructions on the screen to connect your device to Wi-Fi, as shown in Fig 7.
Debugging: 
•	If this is your Photon's first time connecting, it will blink purple and may take 6-12 minutes for the updates, with the Photon restarting a few times in the process. Please do not restart or unplug your Photon during this time.
•	If the device cannot be connected to WIFI via mobile apps, the Photon can connect with the WIFI over USB. This method requires to install the Particle CLI for your computer.
 	 	 
       Fig. 6 Particle APP to be installed on your smartphone
 	 	 
		
 
Fig. 7 Connecting WiFi 
Step 3: Test the LED
A LED is wired to D0. The wire connection is shown in Fig. 8., and the user interface on the mobile app is shown in Fig. 9. We set it as digitalWrite as an output. When the voltage sensed was on high, the LED is lighted. If using the analog write, scroll left to right for different level of lighting level.
  
Fig. 8 Connection of a LED

  
Fig. 9 set the LED as digitalWrite as an output
[OR: Blink the Photon on-board LED (D7) with the Particle App—Tinker screen. Tap D7 and make it in digitalWrite mode. Then tap the D7 circle, and the tiny blue LED on Photon should turn off or on.]
1.5	Overview of the other hardware
1)	Two Adafruit Mini 8x8 LED Matrix boards are the monitor screens.
2)	A Bosch BME 280 is the temperature and humidity sensor.
3)	Panasonic EKMB1101111 is the motion sensor.
4)	Others include Potentiometer, wires, LEDs, and resistors 
5)	A USB cable is used to connect with computer or power brick to provide power 
Fig. 10 is an assembly of a control system. 
 
 
Fig. 10 Connected devices in a Photon project

1.6	Potentiometer 
The potentiometer has three legs: two legs for power connection and one is for output sending to the input board, as shown in Fig.11.
 
Fig. 11 Potentiometer
Test Potentiometer
Try to link the potentiometer to the LED light directly and adjust the light level by switching the potentiometer without passing through Photon.
 
Fig. 12 Test Potentiometer
•	The potentiometer will be the input board A0, assign analog read, iPhone interface shows the voltage corresponding to the different position of the potentiometer.
•	DO could be used to control light level through photon.
  
Fig. 13 connect Potentiometer
1.7	Panasonic EKMB1101111 Motion sensor
The motion sensor also has three legs. One for power in and one for ground. The other one is linked to the board as input A1 and led light D7 as an indicator, as shown in Fig.14.
 
Fig. 14 Motion sensor
The motion sensor has a digital output, which means when it senses someone or something is moving, it will send a digital signal. According to the table1 from motion sensor instruction, the output is a Voltage signal. When it senses moving objects, it will output Vdd voltage, and it has a 0.5VDC output when it detects nothing. The useful link to the motion sensor is 
http://www1.futureelectronics.com/Mailing/etechs/Panasonic/etech_Panasonic_Sensors/Images/PanasonicPIRSensorsCatalog-Update.pdf.
Table 1 Specification of the motion sensor
 
Therefore, the motion sensor should connect to the Analog point of Photon.  And make a program to define the operation signal of motion sensor. Fig.15 is the wire connection of the motion sensor.
 
Fig. 15 motion sensor
   
Fig. 16 Testing the motion sensor
Fig. 16 shows the connection to test the motion sensor, the output of the motion sensor connected to the A1, and on the APP,  it is set as an analog read. The LED is used to implicate the motion detected.
1.8	BME280 temperature and humidity sensor
Only four legs will be connected: VCC  (3V3), GND (GND), SCL (D1), SDA (D0). The latter two legs are for data exchange. Fig. 17 shows the layout of BME280 sensor; it uses I2C protocol to store data of temperature and humidity.  We can find it needs a power supply (3.3 VDC), which can be provided by photon. And SCL and SDA should connect to the D1 and D0 in the photon.
https://www.youtube.com/watch?v=xA-vExF6ChI
  
Fig. 17 Connection of temperature sensors.
Fig. 18 shows the temperature and humidity address of the BME280 sensor. According to this address and bit information, we can make program in the photon and read temperature and humidity data from sensor. (The address of the sensor can be changed by connecting DO to VCC to have the second address.)
 
Fig. 18 BME280 sensor instruction
A reference to the temperature sensor: https://github.com/finitespace/BME280
1.9	Adafruit Mini 8x8 LED Matrix w/I2C Backpack 
Mini 8x8 LED Matrix is used to show the set system parameters. This LED Matrix also supports I2C protocol, which means it should connect to the D0 and D1 of Photon. It also needs a power supply. The 3.3 VDC could be provided by Photon. Fig.19 shows the layout of the LED.	
 
Fig. 19 Monitor
Also, the default I2C address of Mini 8x8 LED Matrix is 0x70. Therefore, if we are going to use two LEDs in one bus, we should change the address of it. Fig. 20 shows the back of the LED. We can change the address of a backpack very easily. Look on the back to find the two A0, A1 solder jumpers. Each one of these is used to hardcode in the address. If a jumper is shorted with solder, that sets the address. A0 sets the lowest bit with a value of 1, A1 sets the middle bit with a value of 2. The final address is 0x70 + A1 + A0. Therefore, if A1 is shorted, the address is 0x70 + 2 = 0x72.
 
Fig. 20 Temperature sensor back look
Library of Adafruit Mini 8x8 LED Matrix w/I2C Backpack can be found: https://github.com/adafruit/Adafruit_LED_Backpack
1.10	Adafruit CCS811 Air Quality Sensor
CCS811 sensor measures eCO2 (equivalent calculated carbon-dioxide) concentration within a range of 400 to 8192 parts per million (ppm), and TVOC (Total Volatile Organic Compound) concentration within a range of 0 to 1187 parts per billion (ppb). The default address is 0X5A.
The layout shows below,
 
Fig. 21 IAQ sensor
Pins :
Vin: This is the power pin, the sensor uses 3.3V;
3V3: This is power output;
GND: Common ground for power and logic;
SCL: This is I2C protocol pin;
SDA: This is I2C protocol pin;
WAKE: It needs to be pulled to ground in order to communicate with the sensor;
RST: This is reset pin;	
INT: This is the interrupt-output pin (seldom use);
The recommended wire connection shows Fig23, WAKE should be connected to the ground.
 
Fig. 22 IAQ sensor links
Library and related application for CO2 sensor can be found:
https://learn.adafruit.com/adafruit-ccs811-air-quality-sensor/arduino-wiring-test

2	Software for the Particle
a.	Online IDE
The detailed instruction can be found on this website. Fig. 23 shows the interface of IDE.  https://docs.particle.io/guide/getting-started/build/photon/
There are three functions for the Web IDE. They are Flash, Verify, and Save.
•	Flash: Flashes the current code to the device. It initiates an over-the-air firmware update and loads the new software onto your device.
•	Verify: This compiles your code without actually flashing it to the device; if there are any errors in your code, they will be shown in the debug console on the bottom of the screen.
•	Save: Saves any changes you've made to your code.

  
Fig. 23 The interface of IDE
b.	 Standalone Particle Dev
Standalone Particle Dev requires to install the software in your computer. It provides more advanced features that make managing large and complicated firmware projects fast and easy. The detailed direction can be found on the website below. Fig. 24 shows the interface of Local Particle Dev.
https://www.particle.io/products/development-tools/particle-desktop-ide
 
Fig. 24 Particle Dev

c.	Mobile App – visualize your data collected.
The detailed direction can be found on the website below. 
https://docs.particle.io/guide/getting-started/start/photon/
 
Fig. 25 visualize your data collected
digitalWrite: Sets the pin to HIGH or LOW, which either connects it to 3.3V (the maximum voltage of the system) or to GND (ground). 
analogWrite: Sets the pin to a value between 0 and 255, where 0 is the same as LOW and 255 is the same as HIGH. This is sort of like sending a voltage between 0 and 3.3V, but since this is a digital system, it uses a mechanism called Pulse Width Modulation, or PWM. 
digitalRead: This will read the digital value of a pin, which can be read as either HIGH or LOW. If you were to connect the pin to 3.3V, it would read HIGH; if you connec t it to GND, it would read LOW. Anywhere in between, it'll probably read whichever one it's closer to, but it gets dicey in the middle.
analogRead: This will read the analog value of a pin, which is a value from 0 to 4095, where 0 is LOW (GND) and 4095 is HIGH (3.3V). All of the analog pins (A0 to A7) can handle this. analogRead is great for reading data from sensors.

d.	Coding  (Some comments have been added on the BME280 code)
The below figures show some basic used codes in the photon program.
 
Fig. 26 Temperature displays on the Mini 8x8 LED Matrix boards
 
Fig. 27 Reading temperature from BME280 sensor
 
Fig. 28 Part of thermostat control logic
e.	Run the code – 
The order of running code is Save, Flash, and Check. Before you check your code, you need to select your particle device in the pulldown tab – particle.
 




Fig. 29 Run and debug
6.	Monitor the system
We can use the software platform of Photon to monitor the operation parameters. The console will be used. It can be opened through the online IDE or local Particle DEV, see Fig. 30 below.
           
Fig. 30 Monitoring the system
In console, we can find a lot of operating parameters of system. See Fig. 31 below.
 
Fig. 31 Monitoring program
On the left side of the console screen, it shows event logs. Temperature and humidity will update every 10 seconds, and motion sensor will send signal once it detects moving objects.
On the right side of the console screen, we can see the functions and variables defined in the photon program. We can adjust the set temperature and monitor the operation parameters of thermostat, such as heating/cooling mode, temperature, and humidity.
 
Resources

•	List for the device mode (Refer to the light of the Photon)
Device mode: The device mode of the Photon can be obtained through the LED light on the module. ( https://docs.particle.io/guide/getting-started/modes/photon/)
Table 1. Modes of the Photon
	Color	Mode
Standard mode	Breathe cyan	Connect to the Internet; users can call functions or flash code
	Blink magenta	Update firmware or load an app
	Blink green	Look for internet
	Blink cyan	In the process of connecting to the cloud
	Breathe white	The WIFI is off
	Blink blue	Listening mode: Input to connect the WIFI
	Breathe magenta	Safe mode: connects the Photon to the cloud, but does not run any application firmware. It is useful for troubleshooting
Trouble shooting mode	Breathe blue	WI-FI module is not connected to a network
	Breathe green	Photon is connected to a Wi-Fi network but not to the cloud
	Blink red	Exist errors with Photon

•	Pin description
Pin	Description
VIN	This pin can be used as an input or output. As an input, supply 3.6 to 5.5VDC to power the Photon. When the Photon is powered via the USB port, this pin will output a voltage of approximately 4.8VDC due to a reverse polarity protection series Schottky diode between VUSB and VIN. When used as an output, the max load on VIN is 1A.
RST	Active-low reset input. On-board circuitry contains a 1k ohm pull-up resistor between RST and 3V3, and 0.1uF capacitor between RST and GND.
VBAT	Supply to the internal RTC, backup registers, and SRAM when 3V3 is not present (1.65 to 3.6VDC).
3V3	This pin is the output of the on-board regulator and is internally connected to the VDD of the Wi-Fi module. When powering the Photon via VIN or the USB port, this pin will output a voltage of 3.3VDC. This pin can also be used to power the Photon directly (max input 3.3VDC). When used as an output, the max load on 3V3 is 100mA. NOTE: When powering the Photon via this pin, ensure power is disconnected from VIN and USB.
RX	Primarily used as UART RX, but can also be used as a digital GPIO or PWM.
TX	Primarily used as UART TX, but can also be used as a digital GPIO or PWM.
WKP	Active-high wakeup pin wakes the module from sleep/standby modes. When not used as a WAKEUP, this pin can also be used as a digital GPIO, ADC input, or PWM. It can be referred to as A7 when used as an ADC.
DAC	12-bit Digital-to-Analog (D/A) output (0-4095), referred to as DAC or DAC1 in software. It can also be used as a digital GPIO or ADC. It can be referred to as A6 when used as an ADC. A3 is a second DAC output used as DAC2 in software.
A0~A7	12-bit Analog-to-Digital (A/D) inputs (0-4095), and also digital GPIOs. A6 and A7 are code convenience mappings, which means pins are not labeled as such, but you may use code like analogRead(A7). A6 maps to the DAC pin and A7 maps to the WKP pin. A4, A5, A7 may also be used as a PWM[2] output.
D0~D7	Digital-only GPIO pins. D0~D3 may also be used as a PWM output.

•	Some cloud functions
The full list of the Photon functions can be found: 
https://docs.particle.io/reference/firmware/photon
Particle.variable(): Expose a variable through the Cloud. Returns a success value - true when the variable was registered. The data type can be INT, DOUBLE and STRING
Particle.function(): Expose a function through the Cloud
