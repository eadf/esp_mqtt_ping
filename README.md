# esp_mqtt_ping
esp8266 periodically sends HC-SR04 ping results over MQTT


I'm pulling in several subtree projects :
* [tuanpmt's mqtt project: esp_mqtt](https://github.com/tuanpmt/esp_mqtt) 
* [esp8266_ping](https://github.com/eadf/esp8266_ping)
* [esp8266_easygpio](https://github.com/eadf/esp8266_easygpio)
* [esp8266_stdout](https://github.com/eadf/esp8266_stdout)

The makefile is copied from [esp_mqtt.](https://github.com/tuanpmt/esp_mqtt)

# Usage 

The default configuration uses a HC-SR04 connected to GPIO0 (trigger) and GPIO2 (echo).

Look in the console for the mqtt topic it uses (it's unique to each esp).


###Building and installing:

First you need to install the sdk and the easy way of doing that is to use [esp_open_sdk.](https://github.com/pfalcon/esp-open-sdk)

You can put that anywhere you like (/opt/local/esp-open-sdk, /esptools etc etc)

Then you could create a small ```setenv.sh``` file, containing the location of your newly compiled sdk and other platform specific info;
```
export SDK_BASE=/opt/local/esp-open-sdk/sdk
export PATH=${SDK_BASE}/../xtensa-lx106-elf/bin:${PATH}
export ESPPORT=/dev/ttyO0  
```
(or setup your IDE to do the same)

You will have to edit the ```include/user_config.h``` file to suit your WiFi and mqtt settings. Alternatively you can copy ```include/user_config.h``` into ```localinclude/user_config.h``` and git will not bother you about modified files.

To make a clean build, flash and connect to the esp console you just do this in a shell:
```
source setenv.sh # This is only needed once per session
make clean && make test
```

You won't be needing esptool, the makefile only uses esptool.py (provided by esp-open-sdk)

I have tested this with sdk v0.9.5 and v0.9.4 (linux & mac)
