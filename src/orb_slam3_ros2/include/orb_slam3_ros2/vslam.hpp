#ifndef VSLAM_HPP
#define VSLAM_HPP

/**
 * @file vslam.hpp
 * @brief Header file for VSLAM System.
 *
 * Contains declarations for common classes and utilities used my VSLAM module
 */

// C++ includes
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <algorithm> // The header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <fstream> // Input/output stream class to operate on files.
#include <thread> // class to represent individual threads of execution.
#include <chrono> // c++ timekeeper library
#include <vector> // vectors are sequence containers representing arrays that can change in size.
#include <queue>
#include <mutex> // A mutex is a lockable object that is designed to signal when critical sections of code need exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <cstdlib> // to find home directory
#include <filesystem>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"    // Header files for rclcpp class
#include "matimg_custom_msg_interface/msg/mat_img.hpp" 
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <stdexcept> // For std::runtime_error
using std::placeholders::_1; //* TODO why this is suggested in official tutorial

// Eigen include
#include <Eigen/Dense> // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header file

// cv-bridge include
#include <cv_bridge/cv_bridge.h>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // Image processing tools
#include <opencv2/highgui/highgui.hpp> // GUI tools
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>

// ORB SLAM 3 library includes. Brings in ORBSLAM3 naemspace
#include "System.h" 
//* Gobal defs
#define pass (void)0 // Python's equivalent of No operation

/**
 * @brief Class definition for VSLAM
 * 
 * Uses RGBD data, estimates orientation
 */
class VSLAM: public rclcpp::Node
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Variables
    std::string agentName = ""; // Name of this agent
    
    // Path related variables
    std::string homeDir = "";
    std::string packagePath = "";
    std::string vocFilePath = ""; // Path to ORB vocabulary provided by DBoW2 package
    
    std::string experimentConfig = ""; // String to receive settings sent by python node
    std::string globalYamlFilePath = ""; // path to /global_yamls directory
    std::string pathToDatasetDBYaml = ""; // full path to the dataset_db.yaml file
    std::string experimentConfigCopy = ""; // Local copy of the configuration string from python node
    std::string experimentType = "others"; // "ieeeaim2024" or "others"
    std::string imageSequenceName = ""; // "MH01", "V101" or if "", raise error 

    // dataset_db.yaml related
    std::string mainKey = "";
    std::string configYamlFile = ""; // Name of config_yaml i.e. EuRoC.yaml, TUM2.yaml etc.
    std::string settingsFilePath = ""; // Full path to the yaml file containing ORB and camera intrinsics
    
    // ORB_SLAM3 library related variables
    ORB_SLAM3::System* pAgent; // pointer to a ORB SLAM3 object
    ORB_SLAM3::System::eSensor sensorType;  // Sensor type
    std::string OPENCV_WINDOW = ""; // TODO one-liner, why required
    bool enablePangolinWindow = true; // Shows Pangolin window output
    bool enableOpenCVWindow = true; // Shows OpenCV window output
    
    // ROS 2 related
    bool bSettingsFromPython = false; // Set true only once when handshake is complete
    
    // Methods
    VSLAM(); // Constructor 
    ~VSLAM(); // Destructor
    void initializeORBSLAM3(); // Initializes the ORB-SLAM3 library with sensor and other configurations
    void handshakeWithPythonNode(); // Performs handshake with the Python node
    // Returns primary key, camera_config yaml file to read
    void readDatasetDBYaml(std::string& pathToDatasetDBYaml, 
                            std::string& imageSequenceName,
                            std::string& mainKey, 
                            std::string& settingsFilePath);
    // 
    void buildFullPathToConfigYaml(std::string& globalPath, 
                                    std::string& configYaml,
                                    std::string& fullPath);
    // Reads the ```ORBSLAM3.sensorType``` field from the config_yaml file 
    // TODO delete, functionality merged with readDatasetDBYaml
    // void readSensorTypeFromYAML(std::string& configYamlPath, 
    //                             ORB_SLAM3::System::eSensor& sensorType);

    private:
    
    // ROS2 related variables
    std::string subexperimentconfigName = ""; // Subscription topic name
    std::string pubconfigackName = ""; // Publisher topic name
    std::string subMatimgName = ""; // Topic to subscribe to receive RGB image and semantic matrix
    std::string subRunFastMFName = "";
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
    rclcpp::Subscription<matimg_custom_msg_interface::msg::MatImg>::SharedPtr subMatimg_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subRunFastMF_subscription_;

    private:
    // Methods
    void setupROSTopics();  // Set up all ROS topics
    ORB_SLAM3::eigenMatXf convertToEigenMat(const std_msgs::msg::Float32MultiArray& msg);

    // ROS 2 callbacks
    void experimentSetting_callback(const std_msgs::msg::String& msg); // Callback to process settings sent over by Python node
    void MatImg_callback(const matimg_custom_msg_interface::msg::MatImg& msg); // Callback to process RGB image and semantic matrix sent by Python node
    void runFastMF_callback(const std_msgs::msg::String& msg);

};









#endif  // VSLAM_HPP