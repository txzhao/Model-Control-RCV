# Model-Control-RCV

This is a repo for course project of [EL2425 Automatic Control, Project Course](https://www.kth.se/social/course/EL2425/) at KTH. 

The project basically includes two lines of work, namely the RCV simulator and the RCV test run. For the simulator work, we modified an open-source simulator [*car_demo*](https://github.com/osrf/car_demo) to make it compatible with the RCV. You can find it [here](https://github.com/txzhao/car_demo). For the RCV test run, we prepared two sets of configurations.

## Motivation



## How to run

### Simulator work

To launch the simulator and run different controllers on it, please see this [part](https://github.com/txzhao/car_demo#how-to-run) of the repo *txzhao/car_demo*.

### RCV test run

To connect to the RCV and perform control methods on it, following steps should be done:

- git clone this repo and copy the two folders in /src to the src folder of your catkin workspace by
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/txzhao/Model-Control-RCV.git
$ cp -r Model-Control-RCV/src/rcv_joy_control Model-Control-RCV/src/rcv_common_msgs ./
```
- download another two modules needed for communications into the same src folder. Note to access the second repo *rcv_communication*, you need to own a [KTH Github](https://www.kth.se/en/student/kth-it-support/work-online/kth-github/kth-github-1.500062) account and [generate a SSH key](https://help.github.com/enterprise/2.11/user/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/) which is associated with this account.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/nmea_msgs.git
$ git clone git@gits-15.sys.kth.se:rcv/rcv_communication.git
```

- build up packages and run rcv_communication:
```
$ cd ~/catkin_ws
$ catkin_make
$ roslaunch rcv_communication_bringup default.launch
```

- After the driver loads the dSpace model, press the "magic" button and the control button on the iPad, simply run the following command in a new terminal:
```
$ roslaunch rcv_joy_control pp_control_no_sim.launch
```

## Results



## Future work


## Related materials

- An introduction to car physics modelling for games [[link]](http://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html)
- ROS answers - plotting real time data [[link]](https://answers.ros.org/question/264767/plotting-real-time-data/)
- GAZEBO answers - How to add a dynamic visual marker in gazebo? [[link]](http://answers.gazebosim.org/question/3383/how-to-add-a-dynamic-visual-marker-in-gazebo/#3394)

