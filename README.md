# CE514Project-3




## Hardware

<b>Temperature and humidity sensor BME280 </b>- Only four legs will be connected: VCC  (3V3), GND (GND), SCL (D1), SDA (D0). The latter two legs are for data exchange. it uses I2C protocol to store data of temperature and humidity.  We can find it needs a power supply (3.3 VDC), which can be provided by photon. And SCL and SDA should connect to the D1 and D0 in the photon.A reference to the temperature sensor: https://github.com/finitespace/BME280

Adafruit Mini 8x8 LED Matrix w/I2C Backpack 
Mini 8x8 LED Matrix is used to show the set system parameters. This LED Matrix also supports I2C protocol, which means it should connect to the D0 and D1 of Photon. It also needs a power supply. The 3.3VDC could be provided by Photon. Figure 20 shows the layout of LED.
In addition, the default I2C address of Mini 8x8 LED Matrix is 0x70. Therefore, if we are going to use two LEDs in one bus, we should change the address of it. Figure 21 shows the back of the LED. We can change the address of a backpack very easily. Look on the back to find the two A0, A1 solder jumpers.

Library of Adafruit Mini 8x8 LED Matrix w/I2C Backpack can be found: https://github.com/adafruit/Adafruit_LED_Backpack

These are all controlled by the Spark Core using a
common I2C bus where pin D0 is SDA and pin D1 is SCL.
The displays (from left to right) are on I2C addresses
0x70, 0x71, and 0x72.

