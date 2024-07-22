## ros2_psd_pcb_reloc

## Authors

1. Azmyin Md. Kamal
2. Neyni K. N. Dadson
3. Donovan Gegg
4. Dr. Corina Barbalata

All authors are with the Department of Mechanical Engineering at Louisiana State University, Baton Rouge, Louisiana, United States of America.

---

## 0a. Introduction

This repository contains a complete ***ROS 2 workspace*** that demonstrates the use of a novel ***multimodal keyframe descriptor*** and a new *** Keyframe-based Place Recognition (KPR)*** method for solving the ***short-term relocalization*** problem in Monocular Keyframe Visual SLAM (MKVSLAM) systems.   

This work was presented in [2024 IEEE/ASME International Conference on Advanced Intelligent Mechatronics](https://www.aim2024.org/) held in Boston, MA from July 15th to July 19th, 2024 A copy of the published paper, presentation slides and a short video showing the system in action are given in this repository. 

## 0b. Quick Summary

The entire system is a combination of **three** packages 

* A modified version of [ROS2 ORB-SLAM3 V1.0](https://github.com/Mechazo11/ros2_orb_slam3) VSLAM framework.

* A ROS 2 package of [YOLOv5 V5.0](https://github.com/ultralytics/yolov5). Although it's an older version, it met our requirements for this paper as we did not need faster performance or image segmentation. This package may be used in a **stand-alone** configuration with minor modifications.

* ```matimg_custom_msg_interface```, a package containing all custom messages, primarily used to transport data between the C++ and Python nodes.

All experiments were done in a laptop with the following configurations
* Ubuntu 22.04
* Intel i5-9300H
* Nvidia RTX 2060
* 16 GB ram. **Note!**, ORB-SLAM3 is a [memory-intensive application](https://github.com/Mechazo11/ros2_orb_slam3/issues/7) and requires at least 16Gb RAM with 8Gb swap during compilation.

Key results are given below

**TODO** add a picture of the results slide

Please note, the original experiment was conducted in ROS 1, but over time, the ```RKinD``` group in the ```iCORE Lab``` has transitioned its research to the ROS 2 ecosystem. There is no difference in the algorithm between the two versions, hence only the ROS 2 version is released to the public.

---

## 1. Paper, Contributions and License 

The contributions of this work are listed below

* A ***new keyframe descriptor*** called the Pose Semantic Descriptor (PSD) is proposed. It utilizes semantic data and camera pose to uniquely characterize keyframe objects in the pose graph. 

* A novel ***Keyframe Place Recognition (KPR)*** algorithm called the Pose-Class-Box (PCB) is formulated that significantly improves pose recovery performance from sudden tracking loss events

* The integration of the proposed descriptor and KPR method for iwithin the open-source ORB-SLAM3 VSLAM framework.

Following ORB-SLAM3 and YOLOv5, this system will also use [GPLv3](https://gist.github.com/kn9ts/cbe95340d29fc1aaeaa5dd5c059d2e60) License

Thank you for taking the time in checking this project out. If you find this work helpful, please consider citing this paper as shown below

***TODO*** add citation

---

### 2. Prerequisites

* [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)

* Colcon: ```sudo apt install python3-colcon-common-extensions```

* Eigen3: ```sudo apt install libeigen3-dev```

* Numpy: ```pip3 install numpy```

* Install CUDA with Pytorch: Follow this [tutorial](https://docs.vultr.com/how-to-install-pytorch-on-ubuntu-22-04)

---

## 2. Installation

The following steps must be performed in sequence as shown below

### Step 0: Configure ```.bashrc``` to use ```ros2_config.sh``` and dynamic library paths

* Download ```ros2_config.sh``` from ```\shell_script``` folder into ```\home```

* In ```.bashrc``` file, add the following lines at the very end of the file

* ```ros2_config.sh``` will automatically load all the configurations and source both ros global and local workspaces everytime a new terminal is opened

???
```
source ~/ros2_config.sh

if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi
```



### Step 1b: Setup the workspace and important directories

Name has to be exactly same,

---

## Step 2: How to run?

TODO

---


## Acknowledgement

The authors would like to acknowledge financial support from NSF #2024795, and the Louisiana Board Of Regents Support Fund, under the Louisiana Materials Design Alliance (LAMDA), provided by the Board as cost share to the NSF under grant number OIA-#1946231.

---

## Bibliography

TODO




## TODOs

- [ ] Update README.md file with clear step by step instruciton on setting up the system


- [ ] Test execution commands and make sure only two terminals is sufficient

- [ ] Start Zenodo and only publish LSU_iCORE_MONO dataset with instructions on how to download and setup the EuRoC and TUM FR2 dataset [1st week July 2024]

- [ ] Delete all package 2 related codes, ros2_tictoc_profiler, line_lbd, fast_mf

* [ ] Make sure to mention that dataset names must be in all caps

* [ ] Make sure to mention that the camera intrinsic matrice YAML may be different from ORB SLAM3 V1

* [ ] make sure to state only pinhole camera models was tested in this framework

* [ ] For making a custom dataset work with the YOLOv5 in py_obj_detector, custom yaml files are needed. State how to add them into the /global_yaml files . This information needs to be placed ion the ```ros2_psd_pcb``` repository

* [ ] Clear all the TODO paragraphs

## DONE

- [x] Build and test the packages [05/20/24]
- [x] Better introduction line shown in the top-left [07/02/24]

## Misc.

* [DUAL-SLAM](https://github.com/HuajianUP/Dual_SLAM) A purely spatial data based relocalization method, published in IROS 2020. They also solved the same problem as this paper

* [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

* [ROS2 workspace](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.youtube.com/watch%3Fv%3D3GbrKQ7G2P0&ved=2ahUKEwi4gLGAyZyGAxUt38kDHfQBDkAQwqsBegQIExAG&usg=AOvVaw3DbkiwvqPzk4Im6OomO3jM). 

* [Tutorial on multiple ROS 2 workspaces](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.youtube.com/watch%3Fv%3DEk2nnWM5zp8&ved=2ahUKEwi4gLGAyZyGAxUt38kDHfQBDkAQwqsBegQIFBAG&usg=AOvVaw1KlhKy-YPUIyzQWg2C4Buc)

* [GDB with ros2](https://juraph.com/miscellaneous/ros2_and_gdb/)

* [shell scripting](https://www.shellscript.sh/variables1.html)