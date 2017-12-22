# Model-Control-RCV

This is a repo for course project of [EL2425 Automatic Control, Project Course](https://www.kth.se/social/course/EL2425/) at KTH. 

The project basically includes two lines of work, namely the RCV simulator and the RCV test run. For the simulator work, we modified an open-source simulator [*car_demo*](https://github.com/osrf/car_demo) to make it compatible with the RCV. You can find it [here](https://github.com/txzhao/car_demo). For the RCV test run, we prepared two sets of configurations.

## Motivation



## How to run

To connect to the RCV and perform control methods on it, following steps should be done:

- git clone this repo and copy the two folders in /src to the src folder of your catkin workspace by
```
$ cd catkin_ws/src/
$ git clone https://github.com/txzhao/Model-Control-RCV.git
$ cp -r Model-Control-RCV/src/rcv_joy_control Model-Control-RCV/src/rcv_common_msgs ./
```
- download another two modules needed for communications into the same src folder:
```
$ cd catkin_ws/src/
$ git clone https://github.com/ros-drivers/nmea_msgs.git
$ git clone git@gits-15.sys.kth.se:rcv/rcv_communication.git
```

## Results



## Future work


