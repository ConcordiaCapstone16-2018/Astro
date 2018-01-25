ROS package for autonomous operation of Astro Toolcart. 

--GENERAL INSTRUCTIONS--

1. key_strokes node must have root access to run. So, run "sudo -s" to give the shell root access. 




--GAZEBO INSTRUCTIONS--

1. You can launch gazebo with a world loaded by specifying it in the launch file for astro (located at ~/workspace/src/pack1/worlds) 

The launch file calls the empty_world.launch file, which takes as an argument the world you want to include.  

Gazebo then needs to find the world you want to add, so you have to add the location of our  model to the GAZEBO_MODEL_PATH, and GAZEBO_RESOURCE_PATH  environment variables. 

	for example, the yellow_robot is contained in ~/workspace/src/pack1/models. 
	so, add the following lines to your ~/.bashrc file. 

	export GAZEBO_MODEL_PATH="/usr/share/gazebo-7/media/models:~/workspace/src/pack1/models"
	export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-7/worlds:~/workspace/src/pack1/worlds"


2. You can insert a model in to the world by specifying that in the world.world file (located in ~/workspace/src/pack1/worlds)

