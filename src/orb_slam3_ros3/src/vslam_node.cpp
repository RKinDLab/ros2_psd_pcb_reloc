/**
 * @file vslam_node.cpp
 * @brief Main node running the ORB-SLAM3 MKVSLAM system
 * @author Azmyin Md. Kamal
 * @date 04/09/2024
*/

// Imports
#include "orb_slam3_ros2/vslam.hpp"

//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    /**
     * try-except block to catch if ROS2_WS env variable is not set
    */
    try {
        auto node = std::make_shared<VSLAM>();  // Declare the node object

        rclcpp::Rate rate(20); // Set the desired update rate (e.g., 10 Hz)
        std::cout << "Performing handshake with python node .................."<<std::endl;
        while (node->bSettingsFromPython!=true) {
            node->handshakeWithPythonNode();
            rclcpp::spin_some(node); // Handle the incoming ROS callbacks
            rate.sleep(); // Sleep to maintain the 20 Hz rate
        }
        node->initializeORBSLAM3(); // Initialize ORB-SLAM3 library
        rclcpp::spin(node); // Blocking node, main entry point to ORB SLAM3 via the VSLAM::matImg_callback() method
        rclcpp::shutdown();
        return 0;

    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}