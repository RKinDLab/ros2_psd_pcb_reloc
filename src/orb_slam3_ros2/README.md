# orb_slam3_ros2

An open-source VSLAM framework. Original paper 


### Useful Reference / Reading materials

* Good notes on CubeSLAM: https://patrick-llgc.github.io/Learning-Deep-Learning/paper_notes/cube_slam.html

* [cube_slam_with_comments](https://github.com/wuxiaolang/Cube_SLAM_wu) 

* Notes on [Sophus](https://strasdat.github.io/Sophus/latest/docs/intro)

* [Spatial Transformation matrices](https://www.brainvoyager.com/bv/doc/UsersGuide/CoordsAndTransforms/SpatialTransformationMatrices.html)

* 



### Requirements

* Requires the ```matimg_custom_msg_interface``` package
* Ensure the ```std::string packagePath``` variable in the ```vslam.hpp```, ```robot_slam.hpp``` files contains the correct name for your ROS2 workspace
* All config files to be located in ```orb_slam3_ros2/orb_slam3/config``` directory, no need to make Monocular, Stereo etc. etc. folders.
* Use convention ```<config_file_name>_MONO.yaml```, ```<config_file_name>_STEREO.yaml``` to differentiate between Monocular and Stereo configuration

### Useful commands
```mCurrentFrame.frameUnitQuat = pKFcur->GetPoseInverse().unit_quaternion(); // Sophus::unit_quaternion() converts 4x4 transformation matrix to a unit quaternion```

```
if (mnAgentName == "robot1" && start_sending_keyframes)
{
    pass;
    // IDEA 1 query here to send out the latest Keyframe added to /robot0
    // Pass this up to ROS level to be published
}
```

### ORB SLAM3 current state

* ```VSLAM::initializeORBSLAM3``` setups up the ORB-SLAM3 library with agent name, sensor configuration, TODO {will add later}
* ```VSLAM::matImg_callback``` takes one MatImg message and executes the ORB-SLAM3 pipeline through it
* ```System::System(const string& nsf, const string &strVocFile, ...)``` is the constructor suitable for 


### TODOs

* [] Make sure to use only one README.md file in ```ros2_psd_pcb_reloc``` repo. Just keep a bare bones readme for individual packages

* [] Make sure to update this readme so that no future work related stuff is given out.

* [] Make a YAML file to read catkin workspace location for both C++ and python node to formalize this

* [ ] In ```System``` class, AFTER NSF, configure ```cameraK``` to be read from the ```Camera``` class file. Currently its ```HARDCODDED```

* [] Make sure to convert blocks of logical blocks```Tracking::Track()``` ***

* [] Once these two problems are fixed, then make the master copy to start paper 1 code

* [ ] Figure out how to use the ```MapDrawer::DrawObservedKeyframes()``` function from last time

### Future Upgrades 

* [ ] Update make Python node wait till receiving clear to receive next processed data by the VSLAM node essentially a bidirectional communication ***

* Use the YAML file to read in the FPS and set ```cvWinWaitTimeVal``` in ```System``` class.


#### Copyright notice

Applicable to all files in the system

```
/**
* This file is part of ORB-SLAM3
* Copyright (C) 2022-2025 Azmyin Md. Kamal, Corina Barbalata, Louisiana State University, Louisiana, USA.
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M,Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/
```

### git branch

```main```: Primary branch, only updated after a feature is developed, tested and is stable.
```dev```: Primary development branch, may take updates from other branches and pushes to main
```nsf24```: branch that was used for NSF24DEMO. No longer used since ***05/01/2024***