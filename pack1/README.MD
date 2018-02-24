ROS package for autonomous operation of Astro Toolcart. 

--GENERAL INSTRUCTIONS--

1. run "mkdir -p ~/workspace/src"  
2. Clone this repository in to your ~/workspace, then change it's name from Astro to src
3. Run catkin_make from ~/workspace
4. run "source ~/workspace/devel/setup.bash" (also add this to ur .bashrc)

--KEY_STROKES--

1. key_strokes node must have root access to run. So, run "sudo -s" to give the shell root access. 

2. key_strokes requires a python package called keyboard  

	run "apt-get install python-pip"
	run "pip install keyboard"

--GAZEBO INSTRUCTIONS--

Gazebo needs to know where to look for models and worlds. 

	for example, the yellow_robot is contained in ~/workspace/src/pack1/models. So, add the following lines to your ~/.bashrc file. Ensure that you change to your home path.  

	export GAZEBO_MODEL_PATH=~/workspace/src/pack1/models
	export GAZEBO_RESOURCE_PATH=~/workspace/src/pack1/worlds
	export GAZEBO_PLUGIN_PATH=~/workspace/src/pack1/plugs

--Other instructions -- 
