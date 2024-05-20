#!/usr/bin/env python3

"""

Send semantic matrix and depth data to ```robot0``` and ```robot1``` vslam nodes.

Used in IEEE AIM 2024
Author: Azmyin Md. Kamal
Date: 05/18/2024
Version: 1.0

"""

# Imports
from py_obj_detector_yolov5_ros2.agentobjectdetector import AgentObjectDetector   # Brings class definition and all imports
import cv2
import time
from .submodules.py_utils import debug_lock # Import helper functions
import rclpy
import torch.backends.cudnn as cudnn
cudnn.benchmark = True # Not sure but needed

# Main entry point of the code
def main(args = None):
    """Main entry point to the script."""  # noqa: D401
    rclpy.init(args=args) # Initialize node
    n0 = AgentObjectDetector("robot0")
    n1 = AgentObjectDetector("robot1")
    # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    rate0 = n0.create_rate(20) 
    rate1 = n1.create_rate(20) 
    # Define common parameters
    start_frame = 0
    stop_frame = -1
    sleep_time = 0.00
    manual_reloc = -1
    # run_ieeeaim2024_exp = True # This by default is True for IEEE AIM 2024 experiments only
    # Set parameters to the Python nodes
    n0.start_frame = start_frame
    n0.frame_stop = stop_frame # Set a specific frame to stop
    n0.sleep_time = sleep_time  # In seconds, should be zero
    n0.manual_reloc = manual_reloc
    # n0.do_reloc_experiment = run_ieeeaim2024_exp
    n0.show_rgb_depth_output = False  # Set true to see images being read and sent
    n0.build_experiment_string()

    n1.start_frame = start_frame
    n1.frame_stop = stop_frame # Set a specific frame to stop
    n1.sleep_time = sleep_time  # In seconds, should be zero
    n1.manual_reloc = manual_reloc
    # n1.do_reloc_experiment = run_ieeeaim2024_exp
    n1.show_rgb_depth_output = False  # Set true to see images being read and sent
    n1.build_experiment_string()

    print("Attempting handshake with robot0 and robot1 C++ node .................")
    while(n0.send_config is True):
        n0.handshake_with_vslam_node()
        n1.handshake_with_vslam_node()
        rclpy.spin_once(n0)
        rclpy.spin_once(n1)
    print("Handshake complete with robot0 and robot1 nodes \n")

    ## In IEEE AIM 2024 experiment, both systems requires to recieve exactly the same RGB image
    ## Hence controlling this loop with respect to robot0 is sufficient
    cnt = n0.start_frame
    for _ in n0.rgb_sequence[n0.start_frame:n0.frame_stop]:
        try:
            n0.run_py_node(cnt) # sends image + semantic matrix
            n1.run_py_node(cnt) # sends image + semantic matrix
            cnt = cnt + 1
            rclpy.spin_once(n0) # Blocking we need a non blocking take care of callbacks
            rclpy.spin_once(n1) # Blocking we need a non blocking take care of callbacks
            rate0.sleep()
            rate1.sleep()
            # DEBUG halt after a certain frame is reached
            if (n0.frame_id>n0.frame_stop and n0.frame_stop != -1):
                print("BREAK!")
                # debug_lock()
                break
        except KeyboardInterrupt:
            break

    # Cleanup
    cv2.destroyAllWindows() # Close all image windows
    n0.destroy_node() # Release all resource related to this node
    n1.destroy_node() # Release all resource related to this node
    rclpy.shutdown()

if __name__ == '__main__':
    main()