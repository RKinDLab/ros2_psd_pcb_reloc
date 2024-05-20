/**
 * @file Tracking.h
 * @brief Definition for Tracking.cc
 * @author Original authors are mentioned above
 * @attention Modified to work with Multiagent system by Azmyin Md. Kamal
 * @date 01/12/22 - 01/12/2025
 */

#ifndef TRACKING_H
#define TRACKING_H

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/aruco.hpp> // To read aruco markers

// ORB SLAM3 includes
#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h" // Struct definition expParameters from System.h will be avialable here
#include "ImuTypes.h"
#include "Settings.h"
#include "SystemUtils.h" // Brings in useful debugging tools, user defined aliases
#include "GeometricCamera.h"

// C/C++ includes
#include <mutex>
#include <unordered_set>

// High precision chrono library https://www.geeksforgeeks.org/chrono-in-c/

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings; 

class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Original version
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq=std::string());

    /*
    pSys already has the struct object built into it to perform IROS 23 experiments
    Order of pMapDrawer and pFrameDrawer variables swapped w.r.t origial in order to prevent ambiguity between the original and overloaded version
    */
    
    /**
     * @brief Overloaded version: PSD_PCB Relocalization, Mutliagent
    */
    Tracking(const std::string& multi, System* pSys, ORBVocabulary* pVoc, MapDrawer* pMapDrawer, 
        FrameDrawer* pFrameDrawer, Atlas* pAtlas, KeyFrameDatabase* pKFDB, 
        const string &strSettingPath, const int sensor, Settings* settings);

    /**
     * @brief Destructor
    */
    ~Tracking();

    //* Both mImGray and mSemMat points to the "original" image and semantic matrix 
    cv::Mat mImGray;
    eigenMatXf mSemMat; 
    cv::Mat mImgGrayCopy; // Untouched copy of the grayscale iamge, needed for aruco marker reading
    
    //* Local copy of VSLAM and experiment parameters from System class
    std::string mnAgentName; // Variable to store the agent's name
    long unsigned int induceReloc = -1;
    bool doSemReloc = false; // Default use bag of words
    bool doRelocExp = false; // Default do not do experiment
    std::string strExpSequence = ""; // "ROBOTICSLAB0" like names
    std::string strModel = ""; // "bag" or "psd_pcb"

    //* Semantic matrix and object detection related variables
    std::unordered_map<float, std::string> STAT_DB;
    std::unordered_map<float, std::string> DYNA_DB;
    std::vector<float> static_obj_database;
    std::vector<float> dynamic_obj_database;
    // std::vector<int> mvTrackRelocInducFrameList;
    
    //* IEEE AIM 2024 Experiment variables
    std::vector<int> mvRelocTestFrames; // List that holds Frame ids where short-term loss is induced
    std::vector<int> vPCBCandCount; // List that holds number of candidates reported by PCB method
    std::vector<int> vDBWo2CandCount; // List that holds number of candidates reported by DBWo2 method
    int nFramesNeededToInitialize = 1; // Starts from 1 as we need a pair of frames to try initialization 
    std::vector<double> vListAvgPoseClose; // List containing AvgPoseQuality of candidates chosen for every execution of Tracking::Relocalization()
    int nMapCount = 1; // How many Maps the system created before merging them together, initialized with value 1
    // float nScaleOfWorld = 1.0; // Depricited
    bool bChx = false;
    std::vector<float> mvLastStaticObjsSeen; // Array that is continously updated to hold list of last objects seen
    vector<KeyFrame*> vpCandidateKFsSem; // Keyframes chosen using the PCB method
    bool bEpisode = false; // Set to true when a loss state is initialized and becomes false when tracking is successful
    bool bNaturalLoss = false; // Set true when natural loss occurs for the first time
    std::vector<int> vNumFrameNeededToReloc; // List that holds how many frames into future were requried from commencement of the reloc episode to recover a pose    
    long unsigned int nRunningLossCount = 0;

    //* Time Statistics {these variables are all global vectors accessible within Tracking class}
    double nTime = 0.0; // A time keeper used across the entire Tracking class
    std::chrono::steady_clock::time_point time_Start; // Time keeper
    std::chrono::high_resolution_clock::time_point tStart, tStop;
    std::vector<double> vFrobCalc_ms; // List of time needed to find average computation time for Frobenius norm
    std::vector<double> vAvgPCBPlaceRecog_ms; // List of time needed to find average computaiton time for PCB method
    std::vector<double> vAvgDBWo2PlaceRecog_ms; // List of time needed to find average computaiton time for PCB method
    std::vector<double> vAvgPoseRecovery_ms; // List of time needed to find average computaiton time needed to recover pose using candidates
    // std::vector<double> vdTrackTotal_ms; // Total time taken by the 

    //* Ints and Floats variables 
    // Accessed by the System::Shutdown() method
    int numTrackLossFrames = 0; // Total number of Frames that had lost episode
    double mAvgTimePlaceRecog; // Average time (ms) to perform Place recognition
    int mAvgRelocCandidatesReported; // Average number (int) of Keyframe candidates found to do Place Recognition
    double mAvgPoseSimilarity; // On average how "close" the poses between query and candidate KeyFrames
    double mAvgTimeToDoRelocalization; // Average time required to perform the full Relocalization
    int mNumFrameBeforeReloc; // Average number of Frames spent in the LOST state
    int nLocalMapsInAtlas; // How many local maps were created in the Atlas
    
    /**
     * @breif Overloaded version, uses the semantic matrix to perform the PCB short-term relocalization
     * @param Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
     * @note Returns the camera pose (empty if tracking fails).
    */
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, 
                                    const eigenMatXf &sem_mat ,string filename);

    /**
     * @brief Print results for IEE AIM 2024 experiment on console
    */
    void IEEAIM2024Results(); 

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);
    // void newParameterLoader(Settings* settings); // DEPRICATED

    // The following methods preprocess the input and call Track(). 
    // Extract features and performs stereo matching.
    // Interface methods called by System class
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);
    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    Sophus::SE3f GetCamTwc();
    Sophus::SE3f GetImuTwb();
    Eigen::Vector3f GetImuVwb();
    bool isImuPreintegrated();

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    //DEBUG
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    float GetImageScale();

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

//* public variable declarations
public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5,
        INITIALIZED=6
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    //* Multi agent
    double mpubDepthMapFactor;  // Public copy of the DepthMapFactor

    // Input sensor
    int mSensor;
    
    // Frame objects
    Frame mCurrentFrame;
    Frame mLastFrame;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;
    
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // Frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;

    vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;
    
protected:

    void Track();   // OG Master tracking function. It is independent of the input sensor.

    // Initialization related
    void StereoInitialization();    // Map initialization for stereo and RGB-D
    void MonocularInitialization(); // Map initialization for monocular
    void CreateInitialMapMonocular();   // Logic to create the initial map for MONOCULAR system
    
    // Keyframe generation
    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Book keeping
    void CheckReplacedInLastFrame(); // TODO write one-liner description
    bool TrackReferenceKeyFrame();  // TODO write one-liner description
    void UpdateLastFrame(); // TODO write one-liner description
    bool TrackWithMotionModel(); // TODO write one-liner description
    
    // Short and Long term Relocalization
    bool Relocalization(); // Original Relocalization method
    
    // Pose-Class-Box method
    bool PerformRelocalization(std::vector<KeyFrame*> &vpCandidateKFs); // Performs 3D-2D PnP and P4P RANSAC matching
    double calcPoseClosenessOfCands(Frame *F,vector<KeyFrame*> &vpCandidList, 
                                    std::string &modelName); // Computes how two pose are similar in 3D Eucledian space

    // Local Mapping
    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();
    bool TrackLocalMap();
    void SearchLocalPoints();
    
    // IMU stuff (not used in my work)
    bool PredictStateIMU(); // TODO write one-liner description
    void PreintegrateIMU(); // Perform preintegration from last frame
    void ResetFrameIMU();   // Reset IMU biases and compute frame velocity

    bool mbMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix, global varialbes
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    //Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    int initID, lastID;

    Sophus::SE3f mTlr;

    

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

public:
    cv::Mat mImRight;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
