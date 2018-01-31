ROS package for autonomous operation of Astro Toolcart. 

--GENERAL INSTRUCTIONS--

1. run "mkdir -p ~/workspace/src"  
2. Clone this repository in to your ~/workspace/src
3. Run catkin_make from ~/workspace
4. run "source ~/workspace/devel/setup.bash" (also add this to ur .bashrc)

--KEY_STROKES--

1. key_strokes node must have root access to run. So, run "sudo -s" to give the shell root access. 

2. key_strokes requires a python package called keyboard  

	run "apt-get install python-pip"
	run "pip install keyboard"

--GAZEBO INSTRUCTIONS--

1. You can launch gazebo with a world loaded by specifying it in the launch file for astro. 
	You have to go to ~/workspace/src/launch/sim.launch and change the include file to your /home/YOUR_USER/workspace/src/pack1/worlds/empty_world.launch" 

2. Gazebo needs to know where to look for models and worlds. 

	for example, the yellow_robot is contained in ~/workspace/src/pack1/models. So, add the following lines to your ~/.bashrc file. Ensure that you change to your home path.  

	export GAZEBO_MODEL_PATH="/home/YOUR_USER/workspace/src/pack1/models"
	export GAZEBO_RESOURCE_PATH="/home/YOUR_USER/workspace/src/pack1/worlds"
	export GAZEBO_PLUGIN_PATH="/home/YOUR_USER/workspace/src/pack1/plugs"

-- MISCELLANEOUS NOTES -- 
1. Changed cpp to python. If you want to compile a cpp file as a node, you must put the .cpp in to the pack1/src file, and add the build instruction in to the CMakeLists.txt in pack1



