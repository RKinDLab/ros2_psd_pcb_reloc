# py_obj_detrector_yolov5_ros2

ROS 2 package to execute YOLOv5 object detection pipeline and passing data to the ORB SLAM 3 node.
This version is the development version solely used for my projects.

#### Supported datasets

```
* EuRoC MAV Drone Dataset [11 sequences]
"MH01" # Easy, prolonged slow drone manuevers
"MH02" # Easy
"MH03" # Medium
"MH04" # Medium, figure 8 movement
"MH05" # Hard
"V101" # Slow, easy manuver drone movement with one near full 360 turn
"V102" # Medium, no rapid drone movement
"V103" # Full 360 rotation, medium up and downward tilting motion
"V201" # Multi-height movement but a bit slower
"V202" # Multi height fast drone movement
"V0203" # Very hard, rapid drone manuvers

* TUM FR2 Pioneer Ground Robot Datasets [4 sequences]
"FR2PIONEER360" # Slow movement, nearl full 360 rotation, highly jerky motion
"FR2PS1" # TUM FR2 Pioneer SLAM 1
"FR2PS2" # TUM FR2 Pioneer SLAM 2
"FR2PS3" # TUM FR2 Pioneer SLAM 2

* NOTE LSU-iCORE-Mono robot Dataset [3 sequence, 1 debugging sequence]
"ROBOTICSLAB_DEBUG" # OG method test for fixing stuff
"ROBOTICSLAB0" # Hard, two loop
"ROBOTICSLAB1" # Medium, U loop in a densly packed room, no dynamic object, medium
"ROBOTICSLAB2" # Hard, multiple loop closures, congested room dataset
```

### git branch definitions
```main```: Primary branch, only updated after a feature is developed, tested and is stable. Only updated by the ```dev``` branch.
```dev```: Primary development branch, may take updates from other branches.

### Requirements
1. Requires the ```matimg_custom_msg_interface``` package that creates a custom interface of the form

```
sensor_msgs/Image rgb
sensor_msgs/Image depth
std_msgs/Float32MultiArray mat
float64 rgb_timestamp
float64 depth_timestamp
```

#### IMPORTANT TODOS before final ROS NOETIC version

- [ ] Integrate rgbd data and test bidirectional handshake with its VSLAM counterpart ***
- [ ] Complete ```self.get_obj_weights()``` for ```vicon room```, ```lsu-icore-mono``` and ```TUM FR2```
- [ ] Carefull bring in all configurations into the ```obj_database.yaml``` file

#### NORMAL

- [ ] Convert ```submodules.py_parameters.py``` as a YAML file to be read by ```AgentObjectDetector``` class
- [ ] Check why ```FLANN``` was require in ```submodules.py_utils.py.retrieve_camera_config()``` function
- [ ] Figure out a programmatic way to do ```self.this_node_dir = str(Path.home()) + "/ros2_ws/src/py_obj_detector_yolov5_ros2/"``` in ```AgentObjectDetector``` class
- [ ] Add instruction on how to create the weights folder and downloading the weights
- [ ] Add note regarding where to put the weights within the package and state the reason i.e. HARDCODED
- [ ] Make a timeout for the bidirecitonal handshake and also consider making it into a function***
         

#### COMPLETED
- [x] Check the following methods in ```submodules.py_utils.py``` file and delete when builds and runs correctly
    * ```tempo_save()```
    * ```find_exp()```
    * ```save_CameraTrajectory()```
    * ```cycle_data_kp_des()```
    * ```enquiry()```


##### Misc. notes

* On setting ROS 2 parameters using YAML file https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/#Run_your_node_with_params
