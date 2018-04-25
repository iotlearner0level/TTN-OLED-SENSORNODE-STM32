# TTN-OLED-SENSORNODE-STM32
A sensor node for the things network using stm32/bluepill

This is my repo for my experments with an STM32 based bluepill board+AiThinker Ra01 lora node.

A few modifications in the IBM LMIC library is required...most important being diabling the disableIRQ and interrupts in hal.cpp



this sketch is based on otaa example and it also features an ssd1306 based i2c oled. Messages are display on both the serial monitor as well as the tiny oled.

v3-oled sends temp/pressure/altitude readings from a cheap bmp180 sensor to the TTN...many messages are displayed on the oled also

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
