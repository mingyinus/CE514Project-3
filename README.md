# CE514 Project-3




<h2> Hardware</h2>

<b>1.1	Photon</b>
<br>Photon is a cloud-based micro-controller platform. It is composed of a powerful ARM Cortex M3 microcontroller and a Broadcom Wi-Fi chip in a tiny thumbnail-sized module called the PØ (P-zero). The micro-controller is the brain for the device and run the application as you desired. It connects to sensors or devices through the IO board.

The link to https://docs.particle.io/datasheets/wi-fi/photon-datasheet/ is the comprehensive description of Photon. Photon can also read data through SPI/I2C telecom protocols.  We can see that D0 and D1 can be used to read I2C protocol, and D2-D5 can be used to read SPI protocol, and A2 – A5 can be used to read SPI protocol.

Besides the microcontroller and WIFI-chip, Particle adds a rock solid 3.3VDC SMPS power supply, RF, and user interface components to the PØ on the Photon. The design is open source, so the users can easily integrate the Photon and sensors into their application.
Connect the photon with Internet
Follow the video: https://vimeo.com/178282058


The follows are all controlled by the Spark Core using a common I2C bus where pin D0 is SDA and pin D1 is SCL. The displays (from left to right) are on I2C addresses 0x70, 0x71, and 0x72.

<br><b>1.2	Breadboard</b>
<br>An electronics breadboard is referring to a solderless breadboard. The backside of the board to show the circuit connections, each metal stripe will have the same current. Central portions, separated, are horizontally connected. Both sides are vertically connected, are supposed to be connected to power supply. When using the Photon, we can get 5V and 3.3 V output from the module. The “-“ is the ground voltage for reference. Please click the link here https://www.youtube.com/watch?v=fq6U5Y14oM4&t=3s to learn more.

<br><b>1.3	LED</b>
<br>Led light connection: longer leg is linked to positive of 3.3 V and the short leg to ground. 
 
<br><b>1.4	The 1st round Test procedures</b>
<br>Step 1: Power the device
Plug the USB cable into your power source: the power source can be power brick, your computer or another power source wired to the port on the module. Here we used the computer for the power supply. Then use the Photon to supply 3.3V (3V3) 5V (VIN) voltage to the breadboard.

<br>Debugging:
•	If your Photon is not blinking blue, hold down the SETUP button.
•	If your Photon is not blinking at all, or if the LED is burning a dull orange color, it may not be getting enough power. Try changing your power source or USB cable.

<br>Step 2: Connect the Photon to the internet using the smartphone
1.	Download the Particle App through the application store.
2.	Sign up an account with Particle.
3.	Press the plus icon and select the device you'd like to add. Then follow the instructions on the screen to connect your device to Wi-Fi.
<br>Debugging: 
•	If this is your Photon's first time connecting, it will blink purple and may take 6-12 minutes for the updates, with the Photon restarting a few times in the process. Please do not restart or unplug your Photon during this time.
•	If the device cannot be connected to WIFI via mobile apps, the Photon can connect with the WIFI over USB. This method requires to install the Particle CLI for your computer.
  
<br>Step 3: Test the LED
A LED is wired to D0. We set it as digitalWrite as an output. When the voltage sensed was on high, the LED is lighted. If using the analog write, scroll left to right for different level of lighting level.

<br><b>1.6	Potentiometer </b>
The potentiometer has three legs: two legs for power connection and one is for output sending to the input board.

<br>Test Potentiometer
Try to link the potentiometer to the LED light directly and adjust the light level by switching the potentiometer without passing through Photon.
•	The potentiometer will be the input board A0, assign analog read, iPhone interface shows the voltage corresponding to the different position of the potentiometer.
•	DO could be used to control light level through photon.

<br><b>1.5	Overview of the other hardware</b>
1)	A Bosch BME 280 is the temperature and humidity sensor
2)	Two Adafruit Mini 8x8 LED Matrix boards are the monitor screens.
3)	Adafruit CCS811 Air Quality Sensor
4)	Panasonic EKMB1101111 is the motion sensor.
5)	Others include Potentiometer, wires, LEDs, and resistors 
6)	A USB cable is used to connect with computer or power brick to provide power 

<b>1.5.1 Temperature and humidity sensor BME280 </b>
Only four legs will be connected: VCC  (3V3), GND (GND), SCL (D1), SDA (D0). The latter two legs are for data exchange. it uses I2C protocol to store data of temperature and humidity.  We can find it needs a power supply (3.3 VDC), which can be provided by photon. And SCL and SDA should connect to the D1 and D0 in the photon.A reference to the temperature sensor: https://github.com/finitespace/BME280

<b>1.5.2. Adafruit Mini 8x8 LED Matrix w/I2C Backpack </b>
Mini 8x8 LED Matrix is used to show the set system parameters. This LED Matrix also supports I2C protocol, which means it should connect to the D0 and D1 of Photon. It also needs a power supply. The 3.3VDC could be provided by Photon. Figure 20 shows the layout of LED.
In addition, the default I2C address of Mini 8x8 LED Matrix is 0x70. Therefore, if we are going to use two LEDs in one bus, we should change the address of it. Figure 21 shows the back of the LED. We can change the address of a backpack very easily. Look on the back to find the two A0, A1 solder jumpers.

Library of Adafruit Mini 8x8 LED Matrix w/I2C Backpack can be found: https://github.com/adafruit/Adafruit_LED_Backpack

<b>1.5.3. Adafruit CCS811 Air Quality Sensor</b>
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

<br><b>1.5.4	Panasonic EKMB1101111 Motion sensor</b>
The motion sensor also has three legs. One for power in and one for ground. The other one is linked to the board as input A1 and led light D7 as an indicator.

The motion sensor has a digital output, which means when it senses someone or something is moving, it will send a digital signal. According to the table1 from motion sensor instruction, the output is a Voltage signal. When it senses moving objects, it will output Vdd voltage, and it has a 0.5VDC output when it detects nothing. The useful link to the motion sensor is 
http://www1.futureelectronics.com/Mailing/etechs/Panasonic/etech_Panasonic_Sensors/Images/PanasonicPIRSensorsCatalog-Update.pdf.

Therefore, the motion sensor should connect to the Analog point of Photon.  And make a program to define the operation signal of motion sensor.

<h2>Software for the Particle</h2>
<br> <b>Online IDE</b>
The detailed instruction can be found on this website. Figure26 shows the interface of IDE.  
https://docs.particle.io/guide/getting-started/build/photon/
There are three functions for the Web IDE. They are Flash, Verify and Save.
•	Flash: Flashes the current code to the device. It initiates an over-the-air firmware update and loads the new software onto your device.
•	Verify: This compiles your code without actually flashing it to the device; if there are any errors in your code, they will be shown in the debug console on the bottom of the screen.
•	Save: Saves any changes you've made to your code.

<br><b> Standalone Particle Dev</b>
Standalone Particle Dev requires to install the software in your computer. It provides more advanced features that make managing large and complicated firmware projects fast and easy. The detailed direction can be found on the website below. Fig. 24 shows the interface of Local Particle Dev.
https://www.particle.io/products/development-tools/particle-desktop-ide
 

<br><b>Mobile App – visualize your data collected.</b>
The detailed direction can be found on the website below. 
https://docs.particle.io/guide/getting-started/start/photon/
 
digitalWrite: Sets the pin to HIGH or LOW, which either connects it to 3.3V (the maximum voltage of the system) or to GND (ground). 

analogWrite: Sets the pin to a value between 0 and 255, where 0 is the same as LOW and 255 is the same as HIGH. This is sort of like sending a voltage between 0 and 3.3V, but since this is a digital system, it uses a mechanism called Pulse Width Modulation, or PWM. 

digitalRead: This will read the digital value of a pin, which can be read as either HIGH or LOW. If you were to connect the pin to 3.3V, it would read HIGH; if you connec t it to GND, it would read LOW. Anywhere in between, it'll probably read whichever one it's closer to, but it gets dicey in the middle.

analogRead: This will read the analog value of a pin, which is a value from 0 to 4095, where 0 is LOW (GND) and 4095 is HIGH (3.3V). All of the analog pins (A0 to A7) can handle this. analogRead is great for reading data from sensors.

<br><b>	Coding  (Some comments have been added on the BME280 code)</b>
The below link is to the code.

<br><b>Run the code </b>
The order of running code is Save, Flash, and Check. Before you check your code, you need to select your particle device in the pulldown tab – particle.

<br><b>Monitor the system</b>
We can use the software platform of Photon to monitor the operation parameters. The console will be used. It can be opened through the online IDE or local Particle DEV.
In console, we can find a lot of operating parameters of system. 

On the left side of the console screen, it shows event logs. Temperature and humidity will update every 10 seconds, and motion sensor will send signal once it detects moving objects.
On the right side of the console screen, we can see the functions and variables defined in the photon program. We can adjust the set temperature and monitor the operation parameters of thermostat, such as heating/cooling mode, temperature, and humidity.
 
<h2>Resources</h2>

<br><b>•	List for the device mode (Refer to the light of the Photon)</b>
Device mode: The device mode of the Photon can be obtained through the LED light on the module. ( https://docs.particle.io/guide/getting-started/modes/photon/)
Table 1. Modes of the Photon
	Color	Mode
	Standard mode	Breathe cyan	Connect to the Internet; users can call functions or flash code
	Blink magenta	Update firmware or load an app
	Blink green	Look for internet
	Blink cyan	In the process of connecting to the cloud
	Breathe white	The WIFI is off
	Blink blue	Listening mode: Input to connect the WIFI
	Breathe magenta	Safe mode: connects the Photon to the cloud, but does not run any application firmware. It is useful for 	troubleshooting
	Trouble shooting mode	Breathe blue	WI-FI module is not connected to a network
	Breathe green	Photon is connected to a Wi-Fi network but not to the cloud
	Blink red	Exist errors with Photon

<br><b>•	Some cloud functions</b>
The full list of the Photon functions can be found: 
https://docs.particle.io/reference/firmware/photon
Particle.variable(): Expose a variable through the Cloud. Returns a success value - true when the variable was registered. The data type can be INT, DOUBLE and STRING
Particle.function(): Expose a function through the Cloud
