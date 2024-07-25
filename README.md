## ros2_psd_pcb_reloc

## Authors

1. Azmyin Md. Kamal
2. Neyni K. N. Dadson
3. Donovan Gegg
4. Dr. Corina Barbalata

All authors are with the Department of Mechanical Engineering at Louisiana State University, Baton Rouge, Louisiana, United States of America.

---

## 0. Introduction

This repository contains a complete ***ROS 2 workspace*** that contains a modified ORB-SLAM3 SLAM framework paired with a YoloV5 object detector for solving the **short-term relocalization** problem using a novel multimodal keyframe descriptor dubbed **Pose-Semantic-Descriptor (PSD)**  and a new Keyframe-based Place Recognition (KPR) method called the **Pose-Class-Box (PCB)** method. This work was presented in [2024 IEEE/ASME International Conference on Advanced Intelligent Mechatronics](https://www.aim2024.org/). A short video demonstrating the performance of the proposed and DBoW2 based short-term relocalization methods is shown below

* [arXiv](TODO) 

* [Slides](https://docs.google.com/presentation/d/1p_Ukic0ZfXyZjq8wjxNMZpKOc4GEQOv2/edit?usp=sharing&ouid=110812301970152705380&rtpof=true&sd=true) 

* A video of the proposed system in action is shown below

TODO RESUME FROM HERE

## 1. High-level Description / Contributions / Bibliography

The contributions of this work are listed below

- A ***new keyframe descriptor*** called the Pose Semantic Descriptor (PSD) is proposed. It utilizes semantic data and camera pose to uniquely characterize keyframe objects in the pose graph. 

- A novel ***Keyframe Place Recognition (KPR)*** algorithm called the Pose-Class-Box (PCB) is formulated that significantly improves pose recovery performance from sudden tracking loss events

- The integration of the proposed descriptor and KPR method for iwithin the open-source ORB-SLAM3 VSLAM framework.

Following ORB-SLAM3 and YOLOv5, this framework will also use [GPLv3](https://gist.github.com/kn9ts/cbe95340d29fc1aaeaa5dd5c059d2e60) License

Thank you for checking this project out. If you find this work helpful, please consider citing this paper as shown below

```
TODO bibtex
```

---

The novel framework is a combination of **three** packages / modules 

* ```orb_slam3_ros2```: A MKVSLAM package based on [ROS2 ORB-SLAM3 V1.0](https://github.com/Mechazo11/ros2_orb_slam3) VSLAM system.

* ```py_ob_detector_yolov5``` A custom ROS 2 package that implements [YOLOv5 V5.0](https://github.com/ultralytics/yolov5), a PyTorch-based Object Detector. This package may be used in a **stand-alone** configuration with minor modifications. Please see the ```README.md``` file in  package for more details

* ```matimg_custom_msg_interface```, a package containing all custom messages required to transfer the semantic matrix between the MKVSLAM and Object Detector nodes.

* All experiments were done in a laptop with the following configurations
    * Ubuntu 22.04 Jammy Jellyfish
    * Intel i5-9300H
    * Nvidia RTX 2060
    * 16 GB ram. 
    * **Note!**, ORB-SLAM3 is a [memory-intensive application](https://github.com/Mechazo11/ros2_orb_slam3/issues/7) and requires at least 16Gb RAM with 8Gb swap during compilation.



### Key results are given below

**TODO** add a picture of the results slide
 
---

## 2. Paper, Contributions and License 



### 3. Prerequisites

The following software needs to be installed and tested before building the workspace in this directory.

* ROS 2 [Humble Hawksbill](https://docs.ros.org/en/humble/index.html)

* Colcon: ```sudo apt install python3-colcon-common-extensions```

* Eigen3: ```sudo apt install libeigen3-dev```

* Numpy: ```pip3 install numpy```

* Install CUDA with Pytorch: Follow this [tutorial](https://docs.vultr.com/how-to-install-pytorch-on-ubuntu-22-04)

---

## 4. Installation

The following steps must be performed in sequence as shown below

### Configure ```.bashrc``` to use ```ros2_config.sh``` and dynamic library paths

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



### Build workspace

Name has to be exactly same,

---

### Setup datasets

TODO

---

### Setup Yolov5

TODO

---

### Run the framework

TODO

---

### Evaluate results

TODO

--

### Acknowledgement

The authors would like to acknowledge financial support from NSF #2024795, and the Louisiana Board Of Regents Support Fund, under the Louisiana Materials Design Alliance (LAMDA), provided by the Board as cost share to the NSF under grant number OIA-#1946231.

---



## TODOs

- [ ] Update README.md file with clear step by step instruciton on setting up the system

- [ ] Delete all package 2 related codes, ros2_tictoc_profiler, line_lbd, fast_mf

- [ ] Test execution commands and make sure only two terminals is sufficient

- [ ] Start Zenodo and only publish LSU_iCORE_MONO dataset with instructions on how to download and setup the EuRoC and TUM FR2 dataset [1st week July 2024]

- [ ] Upload all weights to Zendo and then download and test to make sure they work

* [ ] For making a custom dataset work with the YOLOv5 in py_obj_detector, custom yaml files are needed. State how to add them into the /global_yaml files . This information needs to be placed ion the ```ros2_psd_pcb``` repository

* [ ] Make sure to mention that dataset names must be in all caps

* [ ] Make sure to mention that the camera intrinsic matrice YAML may be different from ORB SLAM3 V1

* [ ] make sure to state only pinhole camera models was tested in this framework



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