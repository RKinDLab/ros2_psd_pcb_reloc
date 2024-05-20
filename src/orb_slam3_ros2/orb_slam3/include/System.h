/**
 * @file System.h
 * @brief Contains definitions for System.cc
 * @author Original authors are attributed in README.md
 * @attention Modified to work with Multiagent system by Azmyin Md. Kamal
 * @date 01/12/22 - 01/12/2025
*/

#ifndef SYSTEM_H
#define SYSTEM_H

//* C/C++ includes
#include <unistd.h>
#include<stdio.h>
#include<string>
#include<thread>
#include<memory>    // To use smart pointers
#include<algorithm> // min(), max() etc./*  */

//* OpenCV
#include<opencv2/core/core.hpp>

//* To find current time
#include <iomanip>
#include <ctime>

//* yaml-cpp
#include <fstream>
#include "yaml-cpp/yaml.h"

//* ORB-SLAM3 includes
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"
#include "SystemUtils.h" // Usful debugging tools written by Azmyin
#include <stdlib.h> 
#include "CommonStructs.h" // Common structs useable across multiple classes

// TODO ---------------------- remove for ros2_pcb_psd
// TFTN library
#include "ros2_tftn/tftn_class.hpp"    // tftn_class.hpp is the file where TFTN library is packaged as an object

// FastMF library
#include "ros2_fast_mf/fast_mf_lib.hpp"
// TODO ---------------------- remove for ros2_pcb_psd

namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            std::cout << str << std::endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System
{

// Aliases
// using FloatStringMap = std::unordered_map<float, std::string>;

public:
    
    // Input sensor
    enum eSensor{
        NOT_SET=-1,
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // VSLAM parameters sent to this node by the Python node
    std::string mnpackagePath; // Copy of path to the VSLAM package
    std::string mnpathToDatasetDBYamlFile; // Copy of the path to the /global_yaml/dataset_db.yaml file
    vslamParameters vslamParam; // Object to hold various parameters sent over by the python node 
    
    // Variables
    // Depricit this variable, we will be using a master function specific to ieeeaim2024
    //bool bsaveTrajIEEEAIM2024 = false; // Default false, set true for ieeeaim2024 experiment
    int nFramesToGen = 5; // How many frames we induce short-term relocalization
    std::vector<int> mvInFrameList; // List that holds the randomly chosen frame ids for inducing relocalization
    std::string mnAgentName; // Variable to store the agent's name
    bool mbEnableCVWin; // Activates/Deactivates OpenCV view window
    std::string time_when_exp_began;
    double cvWinWaitTimeVal = 1e3 / 20; // cv::Waitkey value matching camera FPS

    /**
     * @brief Overloaded constructor built for multiagent systems
     * @param const string& nsf is a place holder to differentiate this constructor from others
     * @note Original version with modifications to accept parameters related to multiagent system
    */
    System(const string& multi, const string& packagePath ,const string& pathToDatasetDBYamlFile,
    const string &strVocFile, const string &strSettingsFile, 
    const eSensor sensor, const string &strExpParameters, std::string strAgentName, 
    const bool bEnableCVWin, const bool bUseViewer = true, const int initFr = 0, 
    const string &strSequence = std::string());

    /**
     * @brief Overloaded version, uses semantic matrix and exectues PCB relocalization pipeline
     * @param Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
     * @note Returns the camera pose (empty if tracking fails).
    */
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const eigenMatXf &sem_mat,const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    /**
     * @brief Method to load the static and dynamic object databases based on passed parameter
     * @note obj_database.yaml file must exsist in the /global_yamls file
    */
    void loadStaticDynamicObjDatabase(vslamParameters& params, std::string& filepath);
    
    /**
     * @brief Randomly sample 5 Frame to induce short-term tracking loss
     * @note All parameters accessed are public variables
    */
    void sampleFramesForExperiment();

    /**
     * @brief Prints out the key and value of class labels from the unordered set
    */
    void printObjectDatabase(std::unordered_map<float, std::string>& objData, const std::string& name); // Prints the Key Value pair from an std::unordered_map object

    void ShutdownModified(); // Modified version, compatible with ROS2
    
    // TODO ------------------- remove in ros2_psd_pcb ------------------------
    
    // TFTN Surface Normal Estimator
    ThreeFiltersToNormal tftn;
    cv::Matx33d cameraK; // camera intrinsic matrix, openCV
    std::vector<Object2DBBox> vObjNameWith2DBBox;
    
    // FastMF
    FastMF fast_mf;     // Object to use Fast MF library

    bool run_fast_mf = false;   // Set True from ROS level
    std::mutex mMutexMultiAgent;
    
    /**
     * @brief Overloaded interface method, passes semantic matrix to Tracking class
     * @attention: Used to interface with Tracking::GrabImageRGBD_LSU() driver method
     * @todo needs to be remodified to make it work with ROS NOETIC VERSION
    */
    Sophus::SE3f TrackRGBD_LSU(const cv::Mat &im, const cv::Mat &depthmap,
    const double &timestep, const eigenMatXf &sem_mat, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), 
    string filename="");

    void robot0ComputeTcd1(Eigen::Matrix3d& R_f, Sophus::SE3f& Tc1, ROBOT1KF& Tcd1); // Computes Tcd1

    int glob_cnt = 0;   // Counter to keep track of keyframes received from robot1
    std::vector<ROBOT1KF> robot1KeyframeMap; 
    bool start_sending_keyframes = false; // Set true from ROS level

    cv::Mat prepDepthImageForTFTN(cv::Mat imDepth, double& mDepthMapFactor); // Implements processing of a depth.png image for the TFTN library.
    cv::Mat generateCvImgFromSurfaceNormals(cv::Mat& resMat); // Returns an openCV image from a surface normal map
    void visualizeSurfNormals(cv::Mat& resMat); // Draw 2D bounding box on surface normal image
    void visualizeObjectSurfNormalsPatch(cv::Mat& surfIn, std::string& objName);

    void extractDataFromSemMat(const eigenMatXf &sem_mat,std::vector<Object2DBBox>& objsVecs, cv::Mat& imIn); // From semantic matrix, populate the std::vector Object2DBBox objects 
    void nsfRbkairosObj(std::vector<Object2DBBox>& objsVecs, cv::Mat &snMat, 
                        Object2DBBox& objOut);
    
    // TODO ------------------- remove in ros2_psd_pcb ------------------------
    
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();
    bool isShutDown();

    //* 09/02/23 evo evaluation compatible versions
    // Custom version, EVO compatible timestep. Poses are in TUM format
    // https://github.com/MichaelGrupp/evo
    void SaveFrameByFrameTrajectoryEuRoC_evo(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC_evo(string &filename); 

    // Save camera trajectory in the TUM RGB-D dataset format.
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset 
    void SaveFrameByFrameTrajectoryTUM(const string &filename); // Save frame by frame trajectory estimate

    // Save keyframe poses in the TUM RGB-D dataset format.
    
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename); // Saves frame by frame 

    //* TODO depriciate these three
    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    //* Not used in our system
    void SaveTrajectoryEuRoC(const string &filename, Map* pMap);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);

    // Save data used for initialization debug
    void SaveDebugData(const int &iniIdx);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    bool SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
    std::vector<MapPoint*> GetAllMapPoints();
    
    // Function used in common.cc
    std::vector<Sophus::SE3f> GetAllKeyframePoses(); // Depricit??

    //* MULTI-AGENT
    std::vector<KeyFrame*> GetAllKeyframesFromSystemDecending();

    cv::Mat GetCurrentFrame();

    Sophus::SE3f GetCamTwc();
    Sophus::SE3f GetImuTwb();
    Eigen::Vector3f GetImuVwb();
    bool isImuPreintegrated();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    float GetImageScale();

    
    
#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

private:

    bool SaveAtlas(int type);
    bool LoadAtlas(int type);

    string CalculateCheckSum(string filename, int type);

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Shutdown flag
    bool mbShutDown;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    // TODO ?
    std::string mStrLoadAtlasFromFile;
    std::string mStrSaveAtlasToFile;

    std::string mStrVocabularyFilePath;

    Settings* settings_;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
