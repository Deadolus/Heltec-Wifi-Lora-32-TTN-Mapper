# Heltec-Wifi-Lora-32-TTN-Mapper

## Create a TTN Mapper node with a Heltec Wifi Lora 32 V2

![TTN Mapper node](nodo_ttn_mapper.jpg?raw=true "TTN Mapper node")

The purpose of this project is to be able to use a Heltec WIFI Lora 32 V2 module together with a smartphone app (https://github.com/deadolus/ttnmapper) to map the coverage of TTN gateways.

It is a fork of (https://github.com/sistemasorp/Heltec-Wifi-Lora-32-TTN-Mapper/).
In this version you do not need a gps module, but an Android phone. 
It works by pushing the location of your android phone to the module, which then sends the coordinates to the TTN network.

### Modules

The GPS's TX pin is connected to Heltec's pin 12.

You can power the entire system with a powerbank. 

### Programming

In your Arduino IDE, install the ESP32 board (https://github.com/espressif/arduino-esp32). Install the libraries U8g2 and the MCCI LoRaWAN LMIC library.

Create an Arduino project and clone or copy the contents from the file TTNMapperNode.ino. Plug the Heltec module to any USB port in your PC and choose in the Arduino IDE the board Heltec Wifi LoRa 32(V2) and the correct serial port. Upload the skecth.

### TTN console

#### Device

The default Activation Method is OTAA. 
For simplicity, you may want to use ABP, do so by changing the "USE_OTAA" define switch and entering your information.

The Frame Counter Checks option must be deactivated in the Device Settings section so that it does not have to be reset every time the module is switched on.

#### Application

In the Integrations section add the TTN Mapper (https://www.thethingsnetwork.org/docs/applications/ttnmapper/#create-the-integration).

In the Decoder section of the Payload Formats the following code has to be applied:

```
function Decoder(bytes, port) {
  var decoded = {};
  decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
  decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
  decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
  decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
  var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
  var sign = bytes[6] & (1 << 7);
  if(sign)
  {
    decoded.alt = 0xFFFF0000 | altValue;
  }
  else
  {
    decoded.alt = altValue;
  }
  decoded.hdop = bytes[8] / 10.0;
  return decoded;
}
```
