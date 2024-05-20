This markdown file lists all the necessary TODOs before porting system to ROS NOETIC

We will begin as soon as either KUKA or URI RBKAIROS base is successfuly converted to a ROS NOETIC system


### TODOs when moving to ROS Noetic, final versions

* [ ] Make sure to copy ```CommonStructs.h``` into the ROS NOETIC orb-slam3 library

* [ ] Create a new package that will only house the ```obj_database.yaml``` file that can be access by both Python node and VSLAM node.

* [ ] Update both ```Systems::Systems()``` overloaded constructors to use ```obj_database.yaml``` from the common package to load static and dynamic object database

* [ ] MUST FIGURE OUT how to get RGB-D version of the SLAM working and match with true scale prediction

* [ ] Need to find out how to compute ```Camera.bf```, ```ThDepth``` factors are coming from and how to get them, ideally for the zed2i camera

* [ ] In ```System::Shutdown()``` programatically figure out how to set the path to save data

* [ ] In ```Frame::Frame()``` overload constructor find out why ```extractSemanticData``` did not work.

* [ ] In ```Frame.cpp``` for the overloaded constructor for RGBD, find out what AssignFeaturesToGrid() does. Merge ```Frame::extractSemanticData()``` function

* [ ] convert ```Object2DBBox``` this to a class that will contain all properties on Object level. This is a ***massive*** system overhaul, do once ROS NOETIC version is stable

* [ ] Learn how to use ROS NOETIC ```Services``` to send one time flag like commands for multi agent system 

* [ ] for the two python nodes turn ```show_rgb_image``` flag as a ros parameter

* [ ] For multi-agent, in a function similar to ```robot0ComputeTcda1()``` the depth of object from camera's current position needs to be considered

* [ ] In constructor only initialize ```agent_name``` and ```config_yaml```, keep all other paths fixed

* [ ] Upgrade ```README.md``` with correct call signatures
