## ros2_psd_pcb_reloc

* One liner intro

## Installation

We assume that you already have ROS 2 Humble installed and are familiar with the basics of a workspace. If not please install and test [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and then familiarize yourself with how to work with a ros2 [workspace](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.youtube.com/watch%3Fv%3D3GbrKQ7G2P0&ved=2ahUKEwi4gLGAyZyGAxUt38kDHfQBDkAQwqsBegQIExAG&usg=AOvVaw3DbkiwvqPzk4Im6OomO3jM). This tutorial on [multiple workspaces](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.youtube.com/watch%3Fv%3DEk2nnWM5zp8&ved=2ahUKEwi4gLGAyZyGAxUt38kDHfQBDkAQwqsBegQIFBAG&usg=AOvVaw1KlhKy-YPUIyzQWg2C4Buc) will also come in handy

Please do the following


### Step 0: Configure ```.bashrc``` to use ```ros2_config.sh``` and dynamic library paths

* ```ros2_config.sh``` will automatically load all the configurations and source both ros global and local workspaces.

* Download ```ros2_config.sh``` from ```\shell_script``` folder into ```\home```

* In ```.bashrc``` file, add the following lines at the very end of the file

```
source ~/ros2_config.sh

if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi
```

## Step 1: Prerequisits

#### Eigen3, Numpy

* Colcon: ```sudo apt install python3-colcon-common-extensions```

* Eigen3: ```sudo apt install libeigen3-dev```

* Numpy: ```pip3 install numpy```

* Install CUDA with Pytorch: [tutorial](https://docs.vultr.com/how-to-install-pytorch-on-ubuntu-22-04)


## Step 2: How to run?

pass

## TODOs

- [ ] Better introduction line shown in the top-left

- [ ] Update README.md file with instructions on downloading and building files [July 2024]

- [ ] On Zenodo only publish LSU_iCORE_MONO dataset with instructions on how to download and setup the EuRoC and TUM FR2 dataset [ June 2024]

- [ ] Delete all package 2 related codes, ros2_tictoc_profiler, line_lbd, fast_mf [05/20/24]

- [ ] Release ros2_psd_pcb_reloc public [July 18th 2024]

- [x] Build and test the packages [05/20/24]

* [ ] Make sure to mention that dataset names must be in all caps

* [ ] Make sure to mention that the camera intrinsic matrice YAML may be different from ORB SLAM3 V1

* [ ] make sure to state only pinhole camera models was tested in this framework

* [ ] For making a custom dataset work with the YOLOv5 in py_obj_detector, custom yaml files are needed. State how to add them into the /global_yaml files . This information needs to be placed ion the ```ros2_psd_pcb``` repository
