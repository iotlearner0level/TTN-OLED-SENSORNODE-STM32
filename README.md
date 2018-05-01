# TTN-OLED-SENSORNODE-STM32
A sensor node for the things network using stm32/bluepill

This is my repo for my experments with an STM32 based bluepill board+AiThinker Ra01 lora node.

A few modifications in the IBM LMIC library is required...most important being diabling the disableIRQ and nointerrupts() in hal.cpp



this sketch is based on abp example & tomtor's work (https://www.thethingsnetwork.org/labs/story/a-cheap-stm32-arduino-node ) and it also features an ssd1306 based i2c oled. Messages are display on both the serial monitor as well as the tiny oled. While idle, the oled display pressure/temperature reported by the bmp180 chip. To avoid cutting wires, i used two i2c (pb7/pb6 and pb11/pb10 respectively for sda1/scl1 and sda2/scl2 see the schematic here: http://wiki.stm32duino.com/index.php?title=File:Bluepillpinout.gif )

For connecting the lora module, sx1278 based aithinker ra02, SPI1 is used. From the schematic http://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif PINS ARE: 
PA7: MOSI1
PA6: MISO1
PA5: SCK1
PA4: NSS1

v4-oled sends temp/pressure/altitude readings from a cheap bmp180 sensor to the TTN...many messages are displayed on the oled also

Now voltage levels/temperatre reported by the stm32 chip is also sent. This is useful in the case we power the bluepill using battery. my observations are that this setup (bluepill/stm32+lora module) works any voltage >2.0V I've not checked the max upper voltage, but 2 series 18650 lipo (8.4V) seem to work ok, without the magic smoke.


If you have any comments please record in the issues tab.

thank you.

===========================

I use the following to convert payload to variables. This is useful in many cases, such as using http integration to send all the data to say, google firebase. In that case, we can see variables as json strings.

function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
var temperature = ((bytes[0]<<8)|bytes[1])/100;
var pressure = (bytes[2]<<8)|bytes[3];
var altitude = (bytes[4]<<8)|bytes[5];
var volt = ((bytes[6]<<8)|bytes[7])/1000;

var chipTemp= ((bytes[8]<<8)|bytes[9])/100;   

  // if (port === 1) decoded.led = bytes[0];

  return {"pressure":pressure,
    "temperature":temperature,
    "altitude":altitude,
    "voltage":volt,
    "chipTemp":chipTemp
  };
}

To save bytes on float, i multiply temperature by 100, convert to integer and send two bytes only. in the payload formats function divide by 100 to get the original data.
