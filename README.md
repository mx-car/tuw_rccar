## RaceCar Tutorial
### Setup the RaceCar:
This tutorial is designed for xubuntu (16.04) in usage with ROS Kinetic.
#### .bashrc:

```
export ARDUINO_ROOT=$HOME/opt/arduino     # to arduino 1.7.8
export VEHICLE_TYPE=1                     # 1 for race car 2 for crawler
export RCCAR_DIR=$HOME/projects/rccar     # your project root with ros workspaces
source $RCCAR_DIR/ws01/devel/setup.bash   # to source your workspace
```
#### ROS Installation: 
  A full manual on how to install ROS Kinetic on the device can be found at the [ROS Tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu) (last accessed: 08.05.2017)
  Make sure u set up your Workspace properly.
#### ROS Setup:
- Install the ROS Realsense-Package and the ROS Joy-Package:
```
sudo apt-get install ros-kinetic-realsense-camera
sudo apt-get install ros-kinetic-joy
```
- Additionally you need to install the following Packages from the TUW_Robotics GitHub page (in this package):
    - tuw_geometry 
    - tuw_teleop 
    - tuw_msgs 
    - tuw_arduino_ros 
        - tuw_rccar, 
        - tuw_arduino_ros, 
        - tuw_arduino_bridge
  
```
mkdir -p $RCCAR_DIR/ws01/src
cd $RCCAR_DIR/ws01/src
git clone https://github.com/tuw-robotics/tuw_geometry.git
git clone https://github.com/tuw-robotics/tuw_teleop.git
git clone https://github.com/tuw-robotics/tuw_msgs.git
cd $RCCAR_DIR/ws01
catkin_make
source $RCCAR_DIR/ws01/devel/setup.bash
mkdir -p $RCCAR_DIR/ws02/src
cd $RCCAR_DIR/ws02/src
git clone https://github.com/tuw-robotics/tuw_arduino_ros.git
cd $RCCAR_DIR/ws02
catkin_make
```
  
    
#### Arduino Setup: 
Set root rights to the USB by preparing or creating the file: `/etc/udev/rules.d/98-openocd.rules` with the following content.
    
```
ACTION!="add|change", GOTO="openocd_rules_end"
SUBSYSTEM!="usb|tty|hidraw", GOTO="openocd_rules_end"

#Please keep this list sorted by VID:PID

#CMSIS-DAP compatible adapters
ATTRS{product}=="*CMSIS-DAP*", MODE="664", GROUP="plugdev"

LABEL="openocd_rules_end"
```
    
Get the Arduino IDE Version 1.7.8 (newer ones are actuelly not supported).
In the directory `/some/directory/tuw_arduino_bridge/arduino/build` and run the following commands:
  
```
export ARDUINO_ROOT=/some/where/
```
    
    Depending on the directory where your Andriuno IDE is saved.
    The vehicle type has to be set (TYPE=1: RaceCar;TYPE=2: RocketCrawler).
```
export VEHICLE_TYPE=1
cmake ..
make firmware_motion_demo.upload
```
    Note:
      The tuw_arduino_bridge package holds deprecated ROS nodes which were used a while ago with shmFw. These nodes are no longer required.
      The important part of the tuw_arduino_bridge package is the firmware folder.

### Set the RaceCar into operation:
**Note:** this guide refers to the RaceCars deployed at TU-rWien Treidlstraße 3, 4th Floor.
#### Connect:
  Connect your Computer to the "humans" wifi, you can Login with your TU e-mail address and your TU password (like the "tuwel" wifi).
    Troubleshooting:
      The "humans" network does not work very reliable.
      If you can not connect to "humans" you can try to connect to the "robots" wifi, ask for the Password.
#### Setup the RaceCar: 
  Connect the IntelUp with the Arduino Board (USB 2.0 to Micro USB "programming").
    Troubleshooting:
      Check the connection wire, it needs to be a data (not a power only) wire.
      Check if the "programming" port of the Arduino is connected. You do not want to use the "native" port.

  Connect the motor controller board with the power switch (the power switch is located under the Arduino).
  Connect the battery with the power switch (the power switch is located under the Arduino).
  Connect the IntelUp with the Logitech F510 gamepad (the wired one).
  Provide the IntelUp with power. You can either use the power supply for the IntelUp or power it with the Amazon Basics battery pack, use the 3.4A output (the output with the double bolt).
  Connect the serve motor (which is responsible for the steering) with the motor controller board.
    Optional:
      Connect the IntelUp with the RealSense R200 camera. Use the USB 3.0 Micro-B to USB 3.0 Micro-B cable only! Other cables like combinations of adapters from USB 3.0 Micro-B to USB 3.0 A with an other USB 3.0 A to USB 3.0 Micro-B are very likely to fail.
      Connect the IMU with the motor controller board (ground is not connected).
#### Run the RaceCar:
  SSH connect to "robot" at the RaceCar (IP-address: 192.168.10.53), ask for the password of the IntelUp.
    Troubleshooting:
      Check if your computer is connected to "humans" wifi or "robots" wifi.
      Check if the RaceCar is connected to the "robots" wifi.
      Check if the RaceCar got the IP-address provided above.
  Switch on the power for the motor controller board.
  Run the following command on the RaceCar:
    ```
    roslaunch tuw_rccar arduino_ros_demo.launch
    ```
    
#### Setup IntelUp:
  Install xubuntu (16.04) or any other related distribution (also 16.04).
  Optional:
    This step is not necessary for the Realsense R200 to work but might help in Troubleshooting.
    Follow the steps 5 and 6 (replace step to with the OS of your choice): [IntelUp Tutorial](https://01.org/developerjourney/recipe/intel-realsense-robotic-development-kit) (last accessed: 08.05.2017)
    
#### Run the Realsense:
  ```
  roslaunch realsense_camera r200_nodelet_default.launch
  ```

  Now you should be able to control the RaceCar with the gamepad, usually one of the buttons RB, LB, RT, LT is a deadman's button.
  Running the following command on your device allows you to access the ROS core of the IntelUp (change IP-adress to the IP-adress (in the "robots" wifi) of the IntelUp):
    ```
    export ROS_MASTER_URI=http://192.168.10.53:11311
    ```
