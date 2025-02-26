# py_obj_detector_yolov5_ros2

A ROS 2 package that wrappes YOLOv5 object detection pipeline. Detected object classes and their 2D bounding box data can be passed to other ROS 2 nodes using custom messages in ```matimg_custom_msg_interface``` package. Part of the `ros2_pcd_psb_reloc` workspace. See top-level `README.md` on how to setup and use this package.

## Requirements

* Python: 3.6+
* CUDA 12+ with compatible version of PyTorch
* ```matimg_img_msg_interface``` ROS 2 package

## Tested datasets

The following datasets have been tested to work with this package

* EuRoC MAV Drone Dataset [11 sequences]
    - "MH01" # Easy, prolonged slow drone manuevers
    - "MH02" # Easy
    - "MH03" # Medium
    - "MH04" # Medium, figure 8 movement
    - "MH05" # Hard
    - "V101" # Slow, easy manuver drone movement with one near full 360 turn
    - "V102" # Medium, no rapid drone movement
    - "V103" # Full 360 rotation, medium up and downward tilting motion
    - "V201" # Multi-height movement but a bit slower
    - "V202" # Multi height fast drone movement
    - "V0203" # Very hard, rapid drone manuvers

* TUM FR2 Pioneer Ground Robot Datasets [4 sequences]
    - "FR2PIONEER360" # Slow movement, nearl full 360 rotation, highly jerky motion
    - "FR2PS1" # TUM FR2 Pioneer SLAM 1
    - "FR2PS2" # TUM FR2 Pioneer SLAM 2
    - "FR2PS3" # TUM FR2 Pioneer SLAM 2

### LSU-iCORE-Mono robot Dataset [3 sequence, 1 debugging sequence]
"ROBOTICSLAB_DEBUG" # Simple U loop, mainly made for testing
"ROBOTICSLAB0" # Hard, two loop
"ROBOTICSLAB1" # Medium, U loop in a densly packed room, no dynamic object, medium
"ROBOTICSLAB2" # Hard, multiple loop closures, congested room dataset


## Misc 

### git branch definitions
```main```: Masterbranch, only updated after a feature is developed, tested and is stable. 
```dev```: Primary development branch, may take updates from other branches.


