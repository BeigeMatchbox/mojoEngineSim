mojoEngineSim README.md

![alt tag](http://www.74hc.co.uk/wp-content/uploads/2014/07/tallBanner_01.jpg)

This is an Arduino project that uses a looping sample to roughly generate engine sound effects. It's
based on the example at http://playground.arduino.cc/Code/PCMAudio you can still see the remnants
in the timer setup functions.

It can be configured to be controlled by a pot, or analog voltage. A pulse width as you would get from
a standard RC receiver, or you can talk to it over SPI. You can also connect a MCP4131 for automatic
volume control.

You can find a demo video here: http://youtu.be/Uw-a45LOt3c


Should work on anything at 16MHz with an ATMega328, and will probably work fine on an ATMega168.

* Pin A1 - Pot wiper
* Pin  2 - Pulsewidth IN
* Pin  3 - Sound OUT
* Pin  4 - MCP4131 CS
* Pin  5 - MCP4131 SCK
* Pin  6 - MCP4131 DATA
* Pin 10 - SPI CS
* Pin 11 - SPI MISO
* Pin 12 - SPI MOSI
* Pin 13 - SPI SCK


In the .ino you can change these values to set which mode you want to use:

```
boolean managedThrottle = true; // Managed mode looks after the digipot if fitted for volume, and adds some mass to the engine
boolean potThrottle = false;    // A pot connected to A1, 0-1023 sets speed
boolean pwmThrottle = false;    // Takes a standard servo signal on pin 2 (UNO)
boolean spiThrottle = true;     // SPI mode, is an SPI slave, expects 1-255 for throttle position, with 0 being engine off
```

managedThrottle can be true or false in any mode, of the last 3 only 1 of them must be true or strange things will happen...

settings.h contains some bits and bobs you can fiddle with, more to add later.