####camera view
<pose>4.5 -22 3.48 0 0.24 1.63</pose>

####original initial parameters
torque 859.4004393000001
braketorque 687.5203514400001

####launch planning-path visualizer
> cd catkin_ws
> catkin_make
> roslaunch run_sim run_sim.launch

new terminal
> roslaunch move_base_config move_base.launch

new terminal
> rosservice list |grep  file (could skip)
> rosservice call /load_path_from_file "path_filename: '/home/el2425/catkin_ws/src/simulation_nodes/fake_planning/path_from_file_planner/data/path.dat' 
> ignore_heading: false"

in rviz, add -> path
