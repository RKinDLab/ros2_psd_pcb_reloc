#!/usr/bin/env python3

"""

Send semantic matrix and depth data to ```robot0``` vslam node.

Author: Azmyin Md. Kamal
Date: 04/09/2024
Version: 1.0

"""

# Imports
from py_obj_detector_yolov5_ros2.agentobjectdetector import AgentObjectDetector   # Brings class definition and all imports
import cv2
from .submodules.py_utils import debug_lock # Import helper functions
import rclpy
import torch.backends.cudnn as cudnn
cudnn.benchmark = True # Not sure but needed

# Main entry point of the code
def main(args = None):
    
    rclpy.init(args=args) # Initialize node
    n = AgentObjectDetector("robot0") #* Initialize the node
    rate = n.create_rate(20) # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
        
    # Define parameters
    n.start_frame = 0
    n.frame_stop = 1000 # Set a specific frame to stop
    n.sleep_time = 0.00  # In seconds, should be zero
    n.manual_reloc = -1
    n.do_reloc_experiment = True
    n.show_rgb_depth_output = False  # Set true to see images being read and sent
    n.build_experiment_string()

    # TODO ------------- remove this in ros2_psd_pcb repo
    # Experiment variables
    # Manually simulate rendevouz event, 
    # Dataset sepecific
    # n.robot0_rendevouz_id = 720 
    # TODO ------------- remove this in ros2_psd_pcb repo

    print("Attempting handshake with C++ node .................")
    #n.get_logger().info("Attempting handshake with C++ node .................")
    while(n.send_config is True):
        n.handshake_with_vslam_node()
        rclpy.spin_once(n) # TODO some details about spin_once and a flag for self testing
    print("Handshake complete with C++ node\n")
    #n.get_logger().info("Handshake complete with C++ node\n")

    cnt = n.start_frame
    for _ in n.rgb_sequence[n.start_frame:n.frame_stop]:
        try:
            n.run_py_node(cnt) # sends image + semantic matrix
            cnt = cnt + 1
            rclpy.spin_once(n) # Blocking we need a non blocking take care of callbacks
            rate.sleep()
            # DEBUG halt after a certain frame is reached
            if (n.frame_id>n.frame_stop and n.frame_stop != -1):
                print("BREAK!")
                # debug_lock()
                break
        except KeyboardInterrupt:
            break

    # Cleanup
    cv2.destroyAllWindows() # Close all image windows
    n.destroy_node() # Release all resource related to this node
    rclpy.shutdown()

if __name__ == '__main__':
    main()