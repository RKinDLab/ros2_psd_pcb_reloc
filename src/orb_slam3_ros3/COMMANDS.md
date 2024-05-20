### Call signature for day-to-day development

* source and launch ***robot0*** node: ```./source_run_robot0.sh```

* Run ***robot0*** slam node:  ```ros2 run orb_slam3_ros2 nsf24_single --ros-args -p agent_name:=robot0 -p config_yaml:=LSU_iCORE_RGBD```

* Run python node ***robot0*** : ```ros2 run py_obj_detector_yolov5_ros2 nsf24_robot0 --ros-args -p dataset_name:="nsf_combo" -p config_query:="LSU_iCORE_RGBD"```


* Launching gdb debugger
* make sure to have launched the ORB SLAM 3 node separately
```gdb --eval-command="target remote localhost:3000" --eval-command="continue"```

* Build GNU debugger ```colcon build --packages-select orb_slam3_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo```

* Build with reduced memory overhead ```colcon build --executor sequential --packages-select orb_slam3_ros2```

* With ZED data ```ros2 run orb_slam3_ros2 nsf24_single --ros-args -p agent_name:=robot0 -p config_yaml:=LSU_iCORE_ZED```

* With ZED data: python node call signature: ```ros2 run py_obj_detector_yolov5_ros2 nsf24_single_agent --ros-args -p dataset_name:=zed_kuka -p config_yaml:=LSU_iCORE_ZED```
