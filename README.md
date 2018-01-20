# Model-Control-RCV

This is a repo for course project of [EL2425 Automatic Control, Project Course](https://www.kth.se/social/course/EL2425/) at KTH. 

The project basically includes two lines of work, namely the simulator of [KTH Research Concept Vehicle (RCV)](https://www.itrl.kth.se/research/projects/kth-rcv/rcv-1.476469) and the RCV field test run. For the simulator work, we modified an open-source simulator [*car_demo*](https://github.com/osrf/car_demo) to make it compatible with the RCV. You can find it [here](https://github.com/txzhao/car_demo). For the RCV test run, we prepared two sets of configurations, and managed to control the RCV to move in different manners automatically.

We kept track of a [progress log](https://github.com/txzhao/Model-Control-RCV/blob/master/progress_log.md) of our project for synchronization between team members, and for some trivial details we also prepared a simple [memo](https://github.com/txzhao/Model-Control-RCV/blob/master/memo.md) to gather them up.

## Table of contents

- [Motivation](https://github.com/txzhao/Model-Control-RCV#motivation)
	- [User story](https://github.com/txzhao/Model-Control-RCV#user-story)
	- [Specifications](https://github.com/txzhao/Model-Control-RCV#specificications)
- [Pipelines](https://github.com/txzhao/Model-Control-RCV#pipelines)
- [How to run](https://github.com/txzhao/Model-Control-RCV#how-to-run)
	- [Run simulator](https://github.com/txzhao/Model-Control-RCV#run-simulator)
	- [RCV test run](https://github.com/txzhao/Model-Control-RCV#rcv-test-run)
- [Results](https://github.com/txzhao/Model-Control-RCV#results)
- [Future work](https://github.com/txzhao/Model-Control-RCV#future-work)
- [Reference](https://github.com/txzhao/Model-Control-RCV#reference)
- [Related materials](https://github.com/txzhao/Model-Control-RCV#related-materials)

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

As is mentioned previously, we prepared two sets of configurations for RCV test run：

#### Closed-loop simulator + open-loop RCV

In this configuration, we formed a closed loop in the simulator and share its control signal (output from controller) with the RCV, which actually performs a open-loop control during the RCV test run, see the figure below. 

Note the feedback in this case is directly taken from the simulator, and is imaginarily considered and used as it is from the real world. The drawbacks thus are, 1) the error between RCV and the model in simulator may cause suboptimal control; 2) running the simulator and the RCV at the same time may put too much workload on the laptop.

![](https://github.com/txzhao/Model-Control-RCV/blob/master/pic/config_1.png)

#### Closed-loop RCV

Given the problems above, we also tried another set of configuration - we used the estimated measurements from odometry in RCV as feedback, and formed a closed control loop, see the figure below.

Compared with the first configuration, the simulator part is abandoned in order to achieve a better efficiency. The feedback is actually fused and estimated from embedded sensors. However, in practice, the lagging effect of this feedback is quite obvious and badly influences the control performance during the RCV field test. 

![](https://github.com/txzhao/Model-Control-RCV/blob/master/pic/config_2.png)

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

