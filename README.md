# Model-Control-RCV

This is a repo for course project of [EL2425 Automatic Control, Project Course](https://www.kth.se/social/course/EL2425/) at KTH. 

The project basically includes two lines of work, namely the simulator of [KTH Research Concept Vehicle (RCV)](https://www.itrl.kth.se/research/itrl-labs/rcv-1.476469) and the RCV field test run. For the simulator work, we modified an open-source simulator [*car_demo*](https://github.com/osrf/car_demo) to make it compatible with the RCV. You can find it [here](https://github.com/txzhao/car_demo). For the RCV test run, we prepared two sets of configurations, and managed to control the RCV to move in different manners automatically.

## Content

- [Motivation](https://github.com/txzhao/Model-Control-RCV#motivation)
	- [User story](https://github.com/txzhao/Model-Control-RCV#user-story)
	- Specifications
- Pipelines
- How to run
	- Run simulator
	- RCV test run
- Results
- Future work
- Reference
- Related materials

## Motivation

### User story

To make the requirements more accessible, we prepare a user story to capture a description of our work's key features from an end-user perspective:

> "As an autonomous-vechicle researcher at [Integrated Transport Research Lab (ITRL)](https://www.itrl.kth.se/), I want to ...
>   1. easily observe and pre-test the performance of an automatic controller so that I can understand whether this controller is ready to be applied and tested on the RCV;
>   2. shift the whole chunck of middle level controllers from Simulink blocks (dSpace) to Python so that more efficient and real-time computing could be achieved."

### Specificications

Research Concept Vechile (RCV) is currently faced with two major challenges during research and development:

- To test algorithms on RCV, either we lift it above the ground a bit with the overhead travelling crane and observe the movements of the four individual wheels, or we ship the RCV to Aranda Test Track for a large field test. Both ways could take too much time and efforts, and turn out to be quite ineffective;
- The previous work is heavily implemented in Simulink blocks, which could be replaced by other coding languages to boost computing efficiency for development purpose.

## Pipelines



## How to run

### Run simulator

To launch the simulator and run different controllers before RCV test run, please read this [part](https://github.com/txzhao/car_demo#how-to-run) of the repo *txzhao/car_demo*.

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

For the simulator work, here are some [gifs](https://github.com/txzhao/car_demo#results) showing its performances.

For RCV test run, please enjoy our project movie [here](https://www.youtube.com/watch?v=nw0xhZjIuw8).

## Future work

- The path visualizer is simply a python live plot, and maybe it will be a better idea to visualize it directly in Gazebo (we could do this in rviz but this may slow down the simulation unexpectedly);
- The MPC controller needs to be fixed in order to deal with the lane-changing situations;
- The lagging effects of feedback from the odometry estimator in RCV terribly influence the field test performance of RCV. It's kind of urgent to find a better way of state estimation before improving the controllers.

## Reference

**[1]** Pereira, Gonçalo Collares, et al. "**Lateral Model Predictive Control for Over-Actuated Autonomous Vehicle.**" Intelligent Vehicles Symposium IEEE, 2017:310-316. [[paper]](http://ieeexplore.ieee.org/document/7995737/)

**[2]** Collares Pereira, G. (2016). **Model Predictive Control for Autonomous Driving of Over-Actuated Research Vehicle** (Dissertation). Retrieved from http://urn.kb.se/resolve?urn=urn:nbn:se:kth:diva-195028 [[paper]](https://kth.diva-portal.org/smash/get/diva2:1043944/FULLTEXT01.pdf)

**[3]** Kokogias, Stefanos, et al. "**Development of Platform-Independent System for Cooperative Automated Driving Evaluated in GCDC 2016.**" IEEE Transactions on Intelligent Transportation Systems PP.99(2017):1-13. [[paper]](http://ieeexplore.ieee.org/document/7891914/)


## Related materials

- An introduction to car physics modelling for games [[link]](http://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html)
- ROS answers - plotting real time data [[link]](https://answers.ros.org/question/264767/plotting-real-time-data/)
- GAZEBO answers - How to add a dynamic visual marker in gazebo? [[link]](http://answers.gazebosim.org/question/3383/how-to-add-a-dynamic-visual-marker-in-gazebo/#3394)

