## PREINSTALL:
### Arduino Firmware
``` 
export ARDUINO_ROOT=/opt/arduino-1.7.8-linux64
or 
export ARDUINO_ROOT=$HOME/opt/arduino
or 
export ARDUINO_ROOT=/somewhere/else
``` 
### PC Programms
Install shmFw
``` 
export SHMFW=$HOME/shmfw/
export SHMFW=/somewhere/else
``` 

### Different Vehicles
To distinguish between the different vehicles, you have to set an environment variable
according to your current configuration:
```
Racecar:   export VEHICLE_TYPE=1
RocketCrawler:    export VEHICLE_TYPE=2
```
(Be sure to run ```cmake ..``` after changing the vehicle type)
