# RobotPower
A Rosserial + **Arduino DUE** based work for Robot's battery and power managment


##Clone and install from source Rosserial 
See reference documentation of [Rosserial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

##Add u8glib library to Arduino's GUI
(u8glib)[https://github.com/olikraus/u8glib] library is needed, you can add (Arduino ZIP version)[https://github.com/olikraus/U8glib_Arduino/releases/latest] directly from **Arduino GUI -> Sketch -> Include Library -> Add .ZIP Library**

##Clone this code and load it in an Arduino DUE board.


##Run RosCore.

```
you$ roscore
... logging to /home/you/.ros/log/d6f526e8-644b-11e6-8eba-00044b26fb48/roslaunch-plutarco-3242.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://plutarco.local:56239/
ros_comm version 1.11.20


SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.20

NODES

auto-starting new master
process[master]: started with pid [3253]
ROS_MASTER_URI=http://plutarco.local:11311/

setting /run_id to d6f526e8-644b-11e6-8eba-00044b26fb48
process[rosout-1]: started with pid [3266]
started core service [/rosout]
```
pay attention to ROS version: *rosdistro: indigo* 

##Run serial_node.py 
This is an output example of output of my *rosrun rosserial_python serial_node.py /dev/ttyACM0* command
```
you$ rosrun rosserial_python serial_node.py /dev/ttyACM0
[INFO] [WallTime: 1471330769.673572] ROS Serial Python Node
[INFO] [WallTime: 1471330769.679982] Connecting to /dev/ttyACM0 at 57600 baud
[ERROR] [WallTime: 1471330786.785870] Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
[INFO] [WallTime: 1471330786.966402] Note: publish buffer size is 512 bytes
[INFO] [WallTime: 1471330786.966842] Setup publisher on BatteryState [sensor_msgs/BatteryState]
[INFO] [WallTime: 1471330786.971024] Setup publisher on adc [rosserial_arduino/Adc]
[INFO] [WallTime: 1471330786.975058] Setup publisher on chatter [std_msgs/String]
```


#Usage Example
There is 3 type of published messages in the first ( test ) release of this code
```
you$ rostopic list 
/BatteryState
/adc
/chatter
```

In a console I can see published message content with *rostopic echo* command

### Chatter message
```
you$ rostopic echo /chatter 
data: hello world!
---
data: hello world!
---
```

### ADC message
```
your$ rostopic echo /adc     
adc0: 0
adc1: 0
adc2: 0
adc3: 0
adc4: 0
adc5: 46509
```

### BatteryState message
```
you$ rostopic echo /BatteryState 
header: 
  seq: 996
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
voltage: 23.2210006714
current: -2.90000009537
charge: 0.240999996662
capacity: 3.59999990463
design_capacity: 5.0
percentage: 0.930000007153
power_supply_status: 2
power_supply_health: 1
power_supply_technology: 2
present: True
cell_voltage: [3.628000020980835, 3.628999948501587, 3.515000104904175, 4.0]
location: robot
serial_number: 123456790
---

