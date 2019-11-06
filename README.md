# CE514Project-3




## Hardware

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

