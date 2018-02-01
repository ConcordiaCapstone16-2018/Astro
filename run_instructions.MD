to launch the package:

sudo -s

roslaunch pack1 sim.launch

rosparam set use_sim_time true

rosrun gmapping slam_gmapping _odom_frame:=yellow_robot/odom _base_frame:=yellow_robot/base_link
