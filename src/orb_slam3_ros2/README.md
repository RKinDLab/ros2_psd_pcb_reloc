[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
# orb_slam3_ros2

This ROS 2 package is a heavily modified version of my original ROS 2 wrapper for ORB-SLAM3, [ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3). Part of the `ros2_pcd_psb_reloc` workspace. See top-level `README.md` on how to setup and use this package.

Except for addressing some build problems, no further update is made to this copy of ORB-SLAM3. 


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

### Requirements

* Requires the ```matimg_custom_msg_interface``` package
* Ensure the ```std::string packagePath``` variable in the ```vslam.hpp```, ```robot_slam.hpp``` files contains the correct name for your ROS2 workspace
* All config files to be located in ```orb_slam3_ros2/orb_slam3/config``` directory, no need to make Monocular, Stereo etc. etc. folders.
* Use convention ```<config_file_name>_MONO.yaml```, ```<config_file_name>_STEREO.yaml``` to differentiate between Monocular and Stereo configuration

### Future Upgrades 

* [ ] Update make Python node wait till receiving clear to receive next processed data by the VSLAM node essentially a bidirectional communication ***
* [ ] Use the YAML file to read in the FPS and set ```cvWinWaitTimeVal``` in ```System``` class.

### Useful Reference / Reading materials

* [cube_slam_with_comments](https://github.com/wuxiaolang/Cube_SLAM_wu) 

* Notes on [Sophus](https://strasdat.github.io/Sophus/latest/docs/intro)

* [Spatial Transformation matrices](https://www.brainvoyager.com/bv/doc/UsersGuide/CoordsAndTransforms/SpatialTransformationMatrices.html)
