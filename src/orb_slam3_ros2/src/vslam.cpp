/**
 * @file vslam.cpp
 * @brief Implementation file for a number of classes of the VSLAM module
 * @author Azmyin Md. Kamal
 * @date 04/09/2024 [version 1] 02/26/25 [updates]
*/

#include "orb_slam3_ros2/vslam.hpp"    // Imports all necessary imports

/**
 * @brief Class constructor
*/
VSLAM::VSLAM(): Node("vslam_agent"){
    // Constructor
    homeDir = getenv("HOME");   // Path to \home directory
    std::cout<<"Home: "<<homeDir<<std::endl;
    
    // Extract path to ros2_workspace
    const char* cstr = std::getenv("ROS2_WS_PATH");
    if (cstr != nullptr) {
        std::string ros2_ws_path(cstr);
        packagePath = ros2_ws_path + "/src/orb_slam3_ros2/"; // Set path to this package's directory
        globalYamlFilePath = ros2_ws_path + "/global_yamls/"; // Set path to /global_yamls directory
        pathToDatasetDBYaml = globalYamlFilePath + "dataset_db.yaml"; 
        std::cout << "pathToDatasetDBYaml " << pathToDatasetDBYaml << std::endl;

        // TODO check if dataset_db yaml file exsists or not
        if (std::filesystem::exists(pathToDatasetDBYaml)) {
            pass;
        } else {
            throw std::runtime_error("dataset_db.yaml not found in /global_yamls directory!");
        }
        
        std::cout<<"VSLAM NODE STARTED\n";
        //RCLCPP_INFO(this->get_logger(), "\nVSLAMAgent NODE STARTED");
        // Declare ROS2 parameter files
        this->declare_parameter("agent_name", "not_given"); // Name of this agent 
        agentName = "not_set";  // Initialize agent name

        // Initialize path variables
        // vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        // settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/";
        vocFilePath = packagePath  + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        //settingsFilePath = packagePath  + "orb_slam3/config/";
        // settingsFilePath = globalYamlFilePath; // /home/user/ros2_ws/global_yaml/

        // Read RO2 parameters
        rclcpp::Parameter param1 = this->get_parameter("agent_name");
        agentName = param1.as_string();
        // rclcpp::Parameter param2 = this->get_parameter("config_yaml");
        // configYaml = param2.as_string();    // Example "LSU_iCORE_MONO", "LSU_iCORE_RGBD"

        setupROSTopics();   // Spin up all ROS topics, different based on agents

        // DEBUG print
        std::cout << "\n" <<std::endl;
        std::cout <<"VSLAM node for agentName: " << agentName << std::endl;
        // std::cout <<"vocFilePaht: " << vocFilePath << std::endl;
        // std::cout <<"settingsFilePath: " << settingsFilePath << std::endl;
        std::cout << "\n" <<std::endl;

    } else {
        throw std::runtime_error("Environment variable ROS2_WS_PATH is not set");
    }
}

/**
 * @brief Class destructor
*/
VSLAM::~VSLAM(){
    
    /**
     * Prints results and saves trajectory for IEEE AIM 2024 experiment
    */
    pAgent->ShutdownModified(); 
}

/**
 * @brief Performs handshake with the Python node
 * @note Called from node driver, returns None
 * @note Internally access configuration string sent by python node
*/
void VSLAM::handshakeWithPythonNode(){
    if(bSettingsFromPython){
        pass;
    }
}


/**
 * @brief Returns primary key, camera_config yaml, ORB_SLAM3 sensor type file to read
 * Pass by reference
*/
void VSLAM::readDatasetDBYaml(std::string& pathToDatasetDBYaml, std::string& imageSequenceName,
                    std::string& mainKey, std::string& settingsFilePath){
    
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(pathToDatasetDBYaml);
    std::string sensorTypeStr;
    // Iterate through the top-level keys in the YAML file
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        std::string key = it->first.as<std::string>();

        // Check if the dataset_names node exists for this key
        if (config[key]["dataset_names"]) {
            std::vector<std::string> dataset_names = config[key]["dataset_names"].as<std::vector<std::string>>();

            // Check if imageSequenceName is in the dataset_names list
            if (std::find(dataset_names.begin(), dataset_names.end(), imageSequenceName) != dataset_names.end()) {
                mainKey = key;
                configYamlFile = config[key]["settings_config"].as<std::string>();
                sensorTypeStr = config[key]["ORBSLAM3.sensorType"].as<std::string>();
                break;
            }
        }
    }
    // Choose the correct ORB_SLAM3::System::eSensor
    // sensorType is a public variable in VSLAM class
    if (sensorTypeStr == "MONOCULAR") {
        sensorType = ORB_SLAM3::System::eSensor::MONOCULAR;
    } else if (sensorTypeStr == "STEREO") {
        sensorType = ORB_SLAM3::System::eSensor::STEREO;
    } else if (sensorTypeStr == "RGBD") {
        sensorType = ORB_SLAM3::System::eSensor::RGBD;
    } else {
        std::cerr << "Unknown sensor / IMU with sensor not supported: " << sensorTypeStr << std::endl;
        exit(-1);
    }
    // DEBUG print
    // std::cout << "\n" <<std::endl;
    // std::cout <<"mainKey: " << mainKey << std::endl;
    // std::cout <<"configYamlFile: " << configYamlFile << std::endl;
    // std::cout << "\n" <<std::endl;
}

/**
 * @brief Initializes the ORB-SLAM3 library with sensor and other configurations
 * @attention Called by the driver cpp file.
 * @note Considers name of agent to setup parameters differently
*/
void VSLAM::initializeORBSLAM3(){
    // Programmatically chose the appropriate yaml file to use
    // Also returns the type of sensor to be used
    readDatasetDBYaml(pathToDatasetDBYaml, imageSequenceName ,mainKey, settingsFilePath);
    
    // Build full path to camera configuration yaml file
    buildFullPathToConfigYaml(globalYamlFilePath, configYamlFile , settingsFilePath);
    
    // TODO make this control able from the launch file or a global yaml 
    enableOpenCVWindow = true;
    enablePangolinWindow = true;

    // DEBUG
    // std::cout << "settingsFilePath: "<< settingsFilePath << std::endl;
    // std::cout << "sensortType: " << sensorType << std::endl;

    // Overloaded constructor for multi-agent systems
    //* IEEE AIM 2024: Left robot uses psd-pcb and right agent uses dbow2 model
    pAgent = new ORB_SLAM3::System("multi-agent",packagePath, pathToDatasetDBYaml,
                                    vocFilePath, settingsFilePath, sensorType, 
                                    experimentConfig, agentName, 
                                    enableOpenCVWindow, enablePangolinWindow);
    
    std::cout << "\nORB-SLAM3 library initialized" << std::endl;
}

/**
 * @brief Converts semantic matrix eigenMatXf, a Eigen 4x4 float matrix
 * @param const std_msgs::msg::Float32MultiArray A ROS2 Float32MultiArray message
 * @note Returns ORB_SLAM3::eigenMatXf 
*/
ORB_SLAM3::eigenMatXf VSLAM::convertToEigenMat(const std_msgs::msg::Float32MultiArray& msg){
    const int dstride0 = msg.layout.dim[0].stride; // msg.layout.dim[0]
    const int dstride1 = msg.layout.dim[1].stride;
    const float h = msg.layout.dim[0].size;
    const float w = msg.layout.dim[1].size;
    // Convert to a Eigen matrix
    std::vector<float> data = msg.data;
    Eigen::Map<ORB_SLAM3::eigenMatXf> mat(data.data(), h, w);
    return mat;
}

/**
 * @brief Callback which accepts experiment parameters from the Python node
 * @note access parameters internally
 * @note returns None
*/
void VSLAM::experimentSetting_callback(const std_msgs::msg::String& msg){
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    experimentConfigCopy = experimentConfig; // Keep a local copy
    
    std::vector<std::string> exp_set = SystemUtils::splitString(experimentConfigCopy, '/');
    experimentType = exp_set[0]; // Name of experiment
    imageSequenceName = exp_set[1]; // Name of image sequence
    // RCLCPP_INFO(this->get_logger(), "experimentConfig received: %s", experimentConfig.c_str());
    // Publish
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    configAck_publisher_->publish(message);
    
    // DEBUG
    std::cout <<"\n"<<std::endl;
    std::cout << "Handshake with Python node complete!"<<std::endl;
    // std::cout << "experimentType: " << experimentType << std::endl;
    std::cout<<"Received configuration: "<<experimentConfig<<std::endl;
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    std::cout <<"\n"<<std::endl;
}

/**
 * @brief Callback to process MatImg message from Python node
 * @attention Primary entry point to ORB SLAM3 library
 * @note returns None
*/
void VSLAM::MatImg_callback(const matimg_custom_msg_interface::msg::MatImg& msg)
{
    // Initialize work variables
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_bridge::CvImagePtr cv_ptr_depth; 
    double rgb_timestep, depth_timestep;
    
    // DEBUGGING, test if Cpp nodes received RGB-D images from python (or other external nodes)
    // Update GUI Window
    // cv::imshow("rgb_image", cv_ptr_rgb->image);
    // cv::waitKey(3);
    // cv::imshow("depth_image", cv_ptr_depth->image);
    // cv::waitKey(3);
    // std::cout <<"\n"<<std::endl;
    // std::cout << "rgbTimestep: "<<rgbTimestep<<std::endl;
    // std::cout << "depthTimestep: "<<depthTimestep<<std::endl;
    // std::cout <<"\n"<<std::endl;

    // Depending on the experiment name switch between the two modes
    
    // Get the RGB image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr_rgb = cv_bridge::toCvCopy(msg.rgb); // Local scope
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading rgb image");
        return;
    }

    // Get the depth image
    if (sensorType == ORB_SLAM3::System::eSensor::RGBD || sensorType == ORB_SLAM3::System::eSensor::STEREO ){
        try
        {
            //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
            cv_ptr_depth = cv_bridge::toCvCopy(msg.depth); // Local scope 
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"Error reading depth image");
            return;
        }
    }
    
    // Get semantic matrix
    ORB_SLAM3::eigenMatXf mat2 = convertToEigenMat(msg.mat);
    
    // Get timesteps
    // Convert timestamp into C++ double
    rgb_timestep = msg.rgb_timestamp;
    depth_timestep = msg.depth_timestamp;
    // ros::Time msg_time = this->get_clock()->now(); // NOTE reversed for future use  
    
    // Run VSLAM pipeline for RGBD data
    if (experimentType == "ieeeaim2024"){
        // IEEE AIM 2024
        Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr_rgb->image, rgb_timestep, mat2);

    } else {
        // OTHERS
        std::cerr << "OTHERS not implemented for IEEE AIM 2024 paper" << std::endl;
        exit(-1);
        // Sophus::SE3f Tcw = pAgent->TrackRGBD_LSU(cv_ptr_rgb->image, cv_ptr_depth->image, 
        //                                     rgb_timestep, mat2);
    }
    
    // NOTE reserved for future use, Pose with respect to global image coordinate
    // Sophus::SE3f Twc = Tcw.inverse(); 
}

/**
 * @brief Set up all ROS topics names and 
 * @note access parameters internally
 * @note returns None
*/
void VSLAM::setupROSTopics(){
    subexperimentconfigName = "/" + agentName + "/experiment_settings"; // subscription topic name
    pubconfigackName = "/" + agentName + "/exp_settings_ack"; // publish topic name
    subMatimgName = "/" + agentName + "/matimg_msg"; // subscription topic name

    // subscribe to receive flag to run fast_mf
    subRunFastMFName = "/" + agentName + "/run_fast_mf_flag_msg";

    // robot0 additional topics: subscribe rendevouz_event, publish get_keyframes, subscribe robot1_keyframe
    if (agentName == "robot0"){
        std::cout << "configured ROS pub/sub for "<<agentName<<std::endl;
        // subscribe to python node to receive settings
        expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&VSLAM::experimentSetting_callback, this, _1));
        // publisher to send out acknowledgement
        configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
        // subscrbite to MatImg topics sent out by Python node
        subMatimg_subscription_= this->create_subscription<matimg_custom_msg_interface::msg::MatImg>(subMatimgName, 1, std::bind(&VSLAM::MatImg_callback, this, _1));
        // subscrbite to MatImg topics sent out by Python node
        // TODO delete
        // subRunFastMF_subscription_= this->create_subscription<std_msgs::msg::String>(subRunFastMFName, 1, std::bind(&VSLAM::runFastMF_callback, this, _1));

    }
    else
    {
        std::cout << "configured ROS pub/sub for "<<agentName<<std::endl;
        // subscribe to python node to receive settings
        expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&VSLAM::experimentSetting_callback, this, _1));
        // publisher to send out acknowledgement
        configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
        // subscrbite to MatImg topics sent out by Python node
        subMatimg_subscription_= this->create_subscription<matimg_custom_msg_interface::msg::MatImg>(subMatimgName, 1, std::bind(&VSLAM::MatImg_callback, this, _1));
    }
}

/**
 * @brief Build the full path to the configuration YAML file
 * @note maybe redundant but kept regardless
 * @note Returns None, in place update
 */
void VSLAM::buildFullPathToConfigYaml(std::string& globalPath, std::string& configYaml, std::string& fullPath){
    // Build full path to chosen .yaml file
    // fullPath = globalPath; // /home/usr/ros2_ws/global_yamls/
    //fullPath = fullPath.append(configYaml); // /home/usr/ros2_ws/global_yamls/LSU_iCORE_Mono.yaml
    fullPath = globalPath + configYaml;
}

// ---------------------------------------------- VSLAMAgent -----------------------------------------------
