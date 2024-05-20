This text file will keep notes of what we did for NSF24DEMO for later use

### Notes on how I implemented the multi-agent mapping schema for NSF 24 DEMO

* Both OpenCV and Pangolin frame needs to be continually updated. OpenCV window is updated by ```Tracker::mpFrameDrawer->Update(this)```

* Pangolin viewer is running in a forever while loop. The observed Keyframes are drawn by ```MapDrawer::robot0DrawKeyframesFromRobot1()``` 

* ```System``` has pointers to the thread drawing both OpenCV and Pangolin assestts as ```mpFrameDrawer``` and ```mpMapDrawer``` respectively

* ```roboto0``` is primary agent, ```robot1``` is secondary agent


### Call signatures for NSF24 Demo single agent

* NSF ***robot0*** VSLAM call signature:  ```ros2 run orb_slam3_ros2 nsf24_single --ros-args -p agent_name:=robot0 -p config_yaml:=LSU_iCORE_RGBD```

* NSF ***robot1*** VSLAM call signature:  ```ros2 run orb_slam3_ros2 nsf24_single --ros-args -p agent_name:=robot1 -p config_yaml:=LSU_iCORE_RGBD```

* Python calls

* NSF ***robot0*** agent: python node call signature: ```ros2 run py_obj_detector_yolov5_ros2 nsf24_robot0 --ros-args -p dataset_name:="nsf_combo" -p config_query:="LSU_iCORE_RGBD"```

* NSF ***robot1*** agent: python node call signature: ```ros2 run py_obj_detector_yolov5_ros2 nsf24_robot1 --ros-args -p dataset_name:="nsf_combo" -p config_query:="LSU_iCORE_RGBD"```


* Run with debugger ```ros2 run --prefix 'gdbserver localhost:3000' orb_slam3_ros2 nsf24_single --ros-args -p agent_name:=robot0 -p config_yaml:=LSU_iCORE_RGBD"```

* NSF single agent: python node call signature: ```ros2 run py_obj_detector_yolov5_ros2 nsf24_single_agent --ros-args -p dataset_name:="nsf_combo" -p config_query:="LSU_iCORE_RGBD"```

Let $c$ represnt camera coordinate of ```robot0``` and $d$ represent the camera coordinate frame of ```robot1```

All poses with respect to camera coordinate frame

Let $T_{c_1}$ be robot0's current position when rendevouz happened. $R_{f}$ is an estimation of ```robot1``` orientation's in coordinate frame $c$. 
\
Then $T_{c{d_1}} = \begin{bmatrix} R_f & t_c + \mathcal{z}^* \\ 0 & 1 \end{bmatrix}$ is the first keyframe pose where $\mathcal{z}^* = [0, 0, z] \in R^3$ and $z$ is the depth of middle of robot1 as viewed by robot0's depthmap.

* 04/18/24 As of this date, the system is integrated but our understanding of the object coordinate system is wrong. 
