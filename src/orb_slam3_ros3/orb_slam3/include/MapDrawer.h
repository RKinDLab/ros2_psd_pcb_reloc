// TODO docstring


#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Atlas.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include "Settings.h"
#include<pangolin/pangolin.h>

#include<mutex>

#include "CommonStructs.h" // Common structs useable across multiple classes

namespace ORB_SLAM3
{

class Settings;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings);

    void newParameterLoader(Settings* settings);

    Atlas* mpAtlas;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc, std::vector<float> &color); //* Overloaded version, draw's camera based on user chosen color
    void SetCurrentCameraPose(const Sophus::SE3f &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);
    void GetCurrentOpenGLObservedMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);

    //* MULTI-AGENT
    void SetCurrentObservedPose(const Sophus::SE3f &Twc2); // Observer's pose already in this agent's world coordinate frame
    // void GetCurrentOpenGLObservedCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw); // Depricited, delete
    
    
    // NSF 24 DEMO
    bool robot0StartRenderingObservedKFs = false; // Set true once to draw the Keyframes from other robot
    void robot0DrawKeyframesFromRobot1(std::vector<ROBOT1KF>& robot1KeyframeMap);

    // NSF 23 version
    // void DrawObservedKeyframes(std::vector<Sophus::SE3f> &observedCams); // Observer's pose already in this agent's world coordinate frame

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Sophus::SE3f mCameraPose;
    Sophus::SE3f mObservedPose; //* MULTI-AGENT, SE3f pose of the observed robot

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};

};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
