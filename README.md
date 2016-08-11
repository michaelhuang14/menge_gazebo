# menge_gazebo

[Gazebo](http://gazebosim.org/) plugin to integrate Menge crowd simulation capabilities into Gazebo simulator. 
 
The plugin inserts agents as models or actors into Gazebo simulation world during startup. On each update cycle, the plugin updates agents positions for corresponding  models/actors in Gazebo. Menge simulation is also synchronized with Gazebo physics simulation by throttling Menge simulation time steps to match Gazebo simulation time.

This plugin depends on a [modified version of to the Menge framework] (https://github.com/michaelhuang14/Menge). 

The source tree is structured as a ROS package. To build, put this source tree under "src" in a [catkin workspace] (http://wiki.ros.org/catkin/workspaces), then invoke catkin_make from the root of catkin workspace.
