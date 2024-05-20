# mat_img_custom_msg_interface

ROS 2 custom message for my Ph.D. project. 

### TODO

- [ ] description of the two custom messages
- [ ] EigenMsg may be not used

Sends a copy of the RGB image, the registered 16-bit depth map, a $M \times 5$ semantic matrix (TODO add paper here) and the timestamps for both rgb and depth image respectively

The structure of the interface is as follows

```
sensor_msgs/Image rgb
sensor_msgs/Image depth
std_msgs/Float32MultiArray mat
float64 rgb_timestamp
float64 depth_timestamp
```
