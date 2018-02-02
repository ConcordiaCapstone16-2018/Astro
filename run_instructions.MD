to launch the package:

sudo -s

roslaunch pack1 sim.launch

rosparam set use_sim_time true

rosrun rviz rviz 

rosrun gmapping slam_gmapping _odom_frame:=Astro/odom _base_frame:=Astro/base_link
