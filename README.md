# SkywalkerWebsocket
Purpose: use ESP32 to control Skywalker roaster

This program is modified from: https://github.com/jmoore52/SkywalkerRoaster. 
This program is only for using ESP32 to communicate and control Skywalker roaster by using Artisan scope. 
The WiFi function of ESP32 may not stable, please use this program carefully and take risk by yourself.

Must do:
1. Set the Config/Port... in Artisan as follows.
![Ports configuration](images/config.jpg)

2. Set the Config/Events as follows.
![Event Slider configuration](images/Slider.jpg)

The message format between Artisan and ESP32 is JSON. For example:

send( {{ "command": "setControlParams",  "fan": 60 }})

is used to set the fan speed as 60%.

Requirements of package version:<br />
Arduino IDE Borads Manager<br />
esp32 by Espressif: 2.0.17<br />
Arduino ESP32 Boards: 2.0.12<br />
