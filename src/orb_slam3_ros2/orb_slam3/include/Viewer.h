/**
 * @file Viewer.h
 * @brief Draws 3D MapPoints, 2D keypoints, keyframes, the current camera pose and the last processed frame
 * @author Original authors are mentioned above
 * @attention Modified to work with Multiagent system by Azmyin Md. Kamal
 * @date 01/12/22 - 01/12/2025
 */

#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"

#include <mutex>

namespace ORB_SLAM3
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Settings;

class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings);

    //* MULTI-AGENT, overloaded version, keeps record of whether to activate or deactivate OpenCV window and keeps a copy of agent's anme
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings, bool bEnableCVWin, std::string strAgentName);

    void newParameterLoader(Settings* settings);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed frame
    // Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    bool isStepByStep();

    void Release();

    //void SetTrackingPause();

    bool both;
    bool mbEnableCVWin; //* MULTI-AGENT, variable to activate or decative openCV window
    std::string mnAgentName; //* MULTI-AGENT, vkeep record of this agent's name
    bool mbMoveViewersToDesiredPositions; // Set false after going thru a few frames
    int mnKeepFrameCount; // Internal frame counter
    
    // PAPER 2
    // Color to draw observed agent's camera poses
    std::vector<float> colorObservedAgent = {1.0f, 0.0f, 1.0f}; // Red color

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mImageViewerScale;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    bool mbStopTrack;
    

};

}


#endif // VIEWER_H
	

