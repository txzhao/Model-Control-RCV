# Progress Log

This markdown file serves as a log that records the progress of our project.

## Project Plan
<a name="project_plan"></a>

| Sprint No. | Tasks planned to finish |
|:----------:|:-----------------------|
| 1 | Identify Input/Output variables to the MPC controller, as well as Input variables to the ROS simulator. (BONUS: Try to implement a simple PI-controller.) | 
| 2 | Compute new dynamics. The ROS simulator used a toyota car, but now uses the RCV. This implies we need to change some dynamic variables, such as the mass, max wheel angle, etc. |
| 3 | Implement a simple PI-controller on ROS simulator if it's not done in sprint 1. Upgrade PI-controller to MPC-controller. |
| 4 | Integrate and verify the new system. The MPC controller should now work in ROS. |
| 5 | Movie and project completion. |

---

| Week 1 | Week 2 | Week 3 | Week 4 | Week 5 | Week 6 | Week 7 |
|:------:|--------|--------|--------|--------|--------|--------|
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
  - It seems that rviz is automatically turned off when running the new branch. So perhaps we need to check this and modify it to have the rviz window popped up if those fancy properties, say camera views, are desired.
- **next move:** 
  - double-check the issue above. It could just be a program crashing problem;
  - literature reading.
  
### 2017-11-09

- **done:**
  - We made a (preliminary) plan of our project, including 5 sprints of one week each. The sprint project plan can be found [here](#project_plan).
  - We started trying to find the inputs and outputs. The following graph illustrates the variabes. 
  ![MPC Inputs/Outputs](https://github.com/txzhao/Model-Control-RCV/blob/master/pic/MPCInputOutput.jpg)
- **next move:** 
  - Understand the input/output variables, such as what are their units.
  - Verify plan.
