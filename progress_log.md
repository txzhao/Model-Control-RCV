# Progress Log

This markdown file serves as a log that records the progress of our project.

### 2017-11-02

- **done:**
  - group meeting in the morning;
  - finished ros installation along with rviz and gazebo 8.1;
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
  - ~~some file paths indicated in the [instruction](https://gist.github.com/svanas/87330eeb17313ea50d5cf9c265ab693f#step-3-add-your-public-key-to-bitbucket) are non-existent or maybe deprecated, e.g. ```C:\Users\<your user name>\AppData\Local\GitHub\PORTAB~1\cmd``` in sub-step 5 of step 5~~ (solved [here](#solved_issue_1)).
- **next move:** 
  - read literature [*Lateral Model Predictive Control for Over-Actuated Autonomous Vehicle*](http://ieeexplore.ieee.org/document/7995737/?reload=true) and those RCV part of literature [*Development of Platform-Independent System for Cooperative Automated Driving Evaluated in GCDC 2016*](http://ieeexplore.ieee.org/document/7891914/);
  - play around with ros tutorial beginner level and the [*turtlesim tutorials*](http://wiki.ros.org/turtlesim/Tutorials) if possible;
  - try finding the model part of the simulator in repo [*car_demo*](https://github.com/ecward/car_demo).
  
### 2017-11-06

- **done:**
<a name="solved_issue_1"></a>
  - finished setting up git and bitbucket. The previous [instruction](https://gist.github.com/svanas/87330eeb17313ea50d5cf9c265ab693f#step-3-add-your-public-key-to-bitbucket) turns out to be too redundant to help, and we switch to this [writeup](http://guganeshan.com/blog/setting-up-git-and-tortoisegit-with-bitbucket-step-by-step.html) for quick setup;
  - cloned the simulink files to the local.
- **next move:** 
  - replace the parameters in the simulink diagram with RCV parameters.
