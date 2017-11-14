# Progress Log

This markdown file serves as a log that records the progress of our project.

<a name="project_plan"></a>
### Project Plan

| Sprint No. | Tasks planned to finish |
|:----------:|:-----------------------|
| 1 |<ul><li>Identify Input/Output variables to the MPC controller in dSpace. </li><li>Identify Input variables to the ROS simulator. </li><li>Determine the variables needed from the above tasks, as well as the units of these variables. </li><li> Implement a simple PI-controller in ROS, and go check if (1) it could control the vehicle to go straight with constant velocity; (2) it could control the vehicle speed up or slow down quickly. | 
| 2 | <ul><li>Compute new dynamics from the collected real data. The ROS simulator used a toyota car, but now uses the RCV. This implies we need to change some dynamic variables, such as the mass, max wheel angle, etc. </li><li> Verify the dynamics, to make sure that they are correct by comparing them with those parameters in dSpace. |
| 3 | <ul><li>Upgrade the PI-controller in ROS to an MPC-controller, and verify it using the same experimental settings of the literature [*Lateral Model Predictive Control for Over-Actuated Autonomous Vehicle*](http://ieeexplore.ieee.org/document/7995737/?reload=true). </li><li> Connect output from dSpace to input in ROS. |
| 4 | <ul><li>Connect the output of the MPC controller in ROS to the low-level controller in dSpace. The new MPC system in ROS should now be fully implemented, as it has input from high-level controller in dSpace, and gives output to low-level controller in dSpace. </li><li>Verify the new system by running a simulation from dSpace. |
| 5 | <ul><li>Create a movie that includes a project description as well as a demonstration of the end results. </li><li>Prepare a project presentation for the class. |

---

| Week 1 | Week 2 | Week 3 | Week 4 | Week 5 | Week 6 | Week 7 |
|:------:|:------:|--------|--------|--------|--------|--------|
|    x   |        |        |        |        |        |        |

### 2017-11-02

- **done:**
  - group meeting in the morning;
  - finished ros installation (kinetic) along with rviz and gazebo 8.1;
  - went through ros tutorial 1.1.1-1.1.3.
- **unsolved:**
  - issue about ros installation on mac;
  - issue about "cannot connect to display" when launching rviz on windows 10 -> go with ubuntu via virtualbox (TODO).
- **next move:** 
  - read literature [*Lateral Model Predictive Control for Over-Actuated Autonomous Vehicle*](http://ieeexplore.ieee.org/document/7995737/?reload=true).

### 2017-11-03

- **done:**
  - group meeting in ITRL;
  - performed test run of simulator [*car_demo*](https://github.com/ecward/car_demo) on the shared laptop. achieved ~0.7 real time factor;
  - tried setting up TortoiseGit for Bitbucket and finish before sub-step 5 of step 5 according to the [instruction](https://gist.github.com/svanas/87330eeb17313ea50d5cf9c265ab693f#step-3-add-your-public-key-to-bitbucket).
- **unsolved:**
  - when starting simulation, we need to untick a lot of properties in rviz to increase real time factor. probably need to check some ways to keep some of the fancy properties while maintaining a high real time factor;
  - ~~some file paths indicated in the [instruction](https://gist.github.com/svanas/87330eeb17313ea50d5cf9c265ab693f#step-3-add-your-public-key-to-bitbucket) are non-existent or maybe deprecated, e.g. ```C:\Users\<your user name>\AppData\Local\GitHub\PORTAB~1\cmd``` in sub-step 5 of step 5~~ (solved in the record of [2017-11-06](#solved_issue_1)).
- **next move:** 
  - read literature [*Lateral Model Predictive Control for Over-Actuated Autonomous Vehicle*](http://ieeexplore.ieee.org/document/7995737/?reload=true) and those RCV part of literature [*Development of Platform-Independent System for Cooperative Automated Driving Evaluated in GCDC 2016*](http://ieeexplore.ieee.org/document/7891914/);
  - play around with ros tutorial beginner level and the [*turtlesim tutorials*](http://wiki.ros.org/turtlesim/Tutorials) if possible;
  - try finding the model part of the simulator in repo [*car_demo*](https://github.com/ecward/car_demo).
  
---

| Week 1 | Week 2 | Week 3 | Week 4 | Week 5 | Week 6 | Week 7 | 
|:------:|:------:|--------|--------|--------|--------|--------|
|    x   |    x   |        |        |        |        |        | 

### 2017-11-06

- **done:**
<a name="solved_issue_1"></a>
  - finished setting up git and bitbucket. The previous [instruction](https://gist.github.com/svanas/87330eeb17313ea50d5cf9c265ab693f#step-3-add-your-public-key-to-bitbucket) turns out to be too redundant to help, and we switch to this [writeup](http://guganeshan.com/blog/setting-up-git-and-tortoisegit-with-bitbucket-step-by-step.html) for quick setup;
  - cloned the simulink files to the local.
- **next move:** 
  - replace the parameters in the simulink diagram with RCV parameters.
  
### 2017-11-07

- **done:**
  - workshop about project management and scrum;
  - git-cloned the new branch [*new_car_model*](https://github.com/ecward/car_demo/tree/new_car_model) (this branch already replaces the Toyota model with RCV) into local repo and tested its running (roslaunch car_demo [rcv_sim.launch](https://github.com/ecward/car_demo/blob/new_car_model/car_demo/launch/rcv_sim.launch)). You can find the command line for git-cloning a specific branch [here](https://stackoverflow.com/questions/4811434/clone-only-one-branch).
- **unsolved:**
  - ~~It seems that rviz is automatically turned off when running the new branch. So perhaps we need to check this and modify it to have the rviz window popped up if those fancy properties, say camera views, are desired~~ (solved in the record of [2017-11-09](#solved_issue_2)).
- **next move:** 
  - double-check the issue above. It could just be a program crashing problem;
  - literature reading.
  
### 2017-11-09

- **done:**
  - We made a (preliminary) plan of our project, including 5 sprints of one week each. The sprint project plan can be found [here](#project_plan).
  - We started trying to find the inputs and outputs. The following graph illustrates the variabes. 
  ![MPC Inputs/Outputs](https://github.com/txzhao/Model-Control-RCV/blob/master/pic/MPCInputOutput.jpg)
  <a name="solved_issue_2"></a>
  - Looked into the rviz-not-popping-up issue. The reason for it comes as: in the launch file [*rcv_sim.launch*](https://github.com/ecward/car_demo/blob/new_car_model/car_demo/launch/rcv_sim.launch), line 34-36 are commented out. That's why the rviz window is not started.
- **next move:** 
  - Understand the input/output variables, such as what are their units.
  - Verify plan.
  
### 2017-11-14

- **done:**
The following are the output variables of the MPC:

| Variable | Explanation |
|----------|-------------|
| MtrTorqueReqAfterLogic | Values between -75-75 [Nm], which equals a force $F=\frac{\tau}{r}$ forward, where r is wheel radius. This torque is applied to all four wheels, as the RCV uses four-wheel driving. | 
| BrkTorqueReqAfterLogic | A torque value between 0-75 [Nm], with the expection of being set to 100 when the velocity is below 0.1[m]. In the same manner as previously, this equals a force F_{Brk} = \frac{\tau}{r}.|
| engageMechBrake | A value that is set to one when BrkTorqueReqAfterLogic is nonzero, otherwise this variable is zero. |
| curvatureReq | jfbirubnvrber |

---


- **next move:** 
  - replace the parameters in the simulink diagram with RCV parameters.
  
