# warthog_auto

First clone this repo into a catkin workspace.

Then install all the dependecies:

	rosdep install --from-paths src --ignore-src -r -y

To visualize the script, run:

	roslaunch warthog_gazebo warthog_world.launch
	roslaunch warthog_gazebo navigation.launch
	roslaunch warthog_gazebo challenge.launch
