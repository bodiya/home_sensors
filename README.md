Home Sensors
==================
This project allows you to forward data from any number of wireless sensors
to the internet via an Arduino Yun. An Ethernet Shield would also suffice for
the receiver, but the Yun's wireless capabilities have the advantage of being
able to be placed strategically to minimize the maximum distance from any of
the sensors.

Results
--------
Click on an item in the legend to enable/disable the sensor.

http://20-minutes.net/charts/ThingSpeak.html

Yun Receiver
--------
The receiver defines a number of ThingSpeak channels that data will be sent to.
The sensor IDs determine which channel data will be sent to. For example,
sensors 0-7 will be sent to the first channel, 8-15 to the second channel, and
16-23 to the third channel. To minimize the amount of time spend sending data,
and to avoid the rate limit set by ThingSpeak, the latest values will be sent
to ThingSpeak once a minute. Only new values received since the last update to
ThingSpeak will be sent.

Sensor Nodes
--------
A sensor node can really be anything that broadcasts data (along with its
sensor ID) in a format that can be understood by the receiver. All of the
sensor nodes in this project consists of an ATmega328P, one or more sensors,
and a 433MHz ASK/OOK transmitter. They are battery powered, so each sensor
reading is only updated once a minute, and the ATmega329P sleeps for the rest
of the time. Data is only transmitted when there is a change in the sensor
value.

Fritzing Files
--------
TODO
