// TODO docstring

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM3
{
// Class constructor
FrameDrawer::FrameDrawer(Atlas* pAtlas):both(false),mpAtlas(pAtlas)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mImRight = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

// Overloaded version, passes the name of the Agent [maybe depricit]
// ************************************************************************************************************************************************************
FrameDrawer::FrameDrawer(Atlas* pAtlas, std::string agentName):both(false),mpAtlas(pAtlas)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mImRight = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mnAgentName = agentName;
}
// ************************************************************************************************************************************************************


/**
 * @briefMaster function which populates a OpenCV image with drawing assests
*/
cv::Mat FrameDrawer::DrawFrame(float imageScale)
{
    cv::Mat im;
    std::vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    std::vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    std::vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    std::vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    std::vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state
    vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0,255,0); // Green
    cv::Scalar odometryColor(255,0,0); // Red
    cv::Scalar boundingBoxColor(0,0,255); // Red

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    // If image size needs to be rescaled
    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);
    
    // How many rows and columns semantic matrix has?
    // auto const rows = pSemMat.rows();
    // auto const cols = pSemMat.cols();

    int rowss = pSemMat.rows();
    int colss = pSemMat.cols();
    
    // std::cout<<"rowss: "<< rowss << " colss: " <<colss<<std::endl;

    //* Bounding box parameters
    int RectThk = 3; // Controls thickness of the bound box lines. Must be non negative integer

    //* Draw bounding boxes on CV image only if detections were present
    // showObjectDetections a boolean that controls whether object detections are drawn on the CV frame
    
    // Check the state of the eigen matrix
    // std::cout<<"pSemMat state: "<<pSemMat<<"\n\n";
    
    if (rowss !=0 && showObjectDetections){
            
        // Iterate through each row. Zero index
        for (int i = 0; i<rowss; i++){
            // Initialize some work variables
            float cls_id;
            std::stringstream ss; // Variable to print the class id
            std::vector<float> vRow;
            vRow.reserve(5); // Each bounding box value has 5 values, [class_id, x1,y1,x2,y2]
            
            //NOTE 09/08/23 without the .coeff(i,j) accessing matrix with .(i,j) leasds to an assertion error
            // Extract 1D row vector
            for (int j = 0; j<colss; j++){
                // vRow.push_back(pSemMat(i,j)); // Originally causes error  
                vRow.push_back(pSemMat.coeff(i,j));
            }
            
            //SystemUtils::printStdVecFloat(vRow); // DEBUG
            //* Draw bounding box if vRow has valid data
            if (!vRow.empty()){
                //std::cout<<"cls_id: "<<cls_id<<" x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2<<"\n";
                //* https://www.geeksforgeeks.org/draw-an-rectangle-using-opencv-in-cpp/ {How to draw rectangle}
                // cv::Point p1(vRow.at(1),vRow.at(2)); // Top Left Corner [x1,y1] // Original function
                // cv::Point p2(vRow.at(3),vRow.at(4)); // Bottom Right Corner [x2,y2]
                
                // Potential fix
                int x1 = static_cast<int>(vRow.at(1));
                int y1 = static_cast<int>(vRow.at(2));
                int x2 = static_cast<int>(vRow.at(3));
                int y2 = static_cast<int>(vRow.at(4));

                //* Guards against overflow 09/15
                //* TODO if this works, programmatic way of passing size of image
                if (x1 < 0 || x1 >= 640){
                    x1 = 640;
                }

                if (x2 < 0 || x2 >= 640){
                    x2 = 640;
                }

                if (y1 < 0 || y1 >= 480){
                    y1 = 480;
                }

                if (y2 < 0 || y2 >= 480){
                    y2 = 480;
                }

                cv::Point p1(x1,y1); // Top Left Corner [x1,y1]
                cv::Point p2(x2,y2); // Bottom Right Corner [x2,y2]
                
                // cv::rectangle(im,p1,p2,boundingBoxColor, RectThk); //* 09/13/23 We are being hit with an error here for some of the sequences
                
                // try{
                //     cv::rectangle(im,p1,p2,boundingBoxColor, RectThk);
                // }
                // catch (const std::exception& e){
                //     std::cout<<"Error in FrameDrawer::DrawFrame(), ignored drawing bounding box! \n";
                // }

                try{
                    cv::rectangle(im,p1,p2,boundingBoxColor, RectThk);
                }
                catch (const cv::Exception& e){
                    std::cout<<"Error in FrameDrawer::DrawFrame(), ignored drawing bounding box! \n";
                }

                // Add the text string
                cls_id = vRow.at(0);
                //s = SystemUtils::findClassNameFromBD(cls_id, pSTAT_DB, pDYNA_DB);
                
                ss << "["<<SystemUtils::findClassNameFromBD(cls_id, pSTAT_DB, pDYNA_DB)<<"]";
                
                //std::cout<<s<<"\n";

                float box_w = vRow.at(3) - vRow.at(1); // Box width x2 - x1
                float box_h = vRow.at(4) - vRow.at(2); // Box height y2 - y1

                // TODO find out why? NEED TO DOUBLE CHECK WITH EUROC DATASET
                int text_offset_x = std::floor(- 2 * (box_w / 2));
                int text_offset_y = - 7;
                int cx = int(std::floor(vRow.at(1)) + 10);
                
                cv::Point coord_text(cx, std::floor(vRow.at(2)) + text_offset_y); // Print at upper middle
                cv::putText(im,ss.str(),coord_text,cv::FONT_HERSHEY_PLAIN,2,boundingBoxColor,2,8);
            }
            
        } // end of for()      
    }

    //* Draw rectangles/circles on CV windows
    // Logic for uninitialized system
    if(state==Tracking::NOT_INITIALIZED)
    {   
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im,pt1,pt2,standardColor);
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
        {
            cv::Point2f pt1,pt2;
            if(imageScale != 1.f)
            {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            }
            else
            {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im,pt1,pt2, standardColor,5);
        }

    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = vCurrentKeys[i].pt / imageScale;
                    float px = vCurrentKeys[i].pt.x / imageScale;
                    float py = vCurrentKeys[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = vCurrentKeys[i].pt;
                    pt1.x=vCurrentKeys[i].pt.x-r;
                    pt1.y=vCurrentKeys[i].pt.y-r;
                    pt2.x=vCurrentKeys[i].pt.x+r;
                    pt2.y=vCurrentKeys[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,standardColor);
                    cv::circle(im,point,2,standardColor,-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,odometryColor);
                    cv::circle(im,point,2,odometryColor,-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    // To check OpenCV image size
    // std::cout<<"Size before\n";
    // std::cout << "Width : " << im.size().width << std::endl;
    // std::cout << "Height: " << im.size().height << std::endl;
    // std::cout<<"\n";

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo); // NOTE 20px width is added to the bottom of the image to make room for the status information
    // TODO add information about relocalization and loss of track
    
    return imWithInfo;
}

cv::Mat FrameDrawer::DrawRightFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mImRight.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeysRight;
        }
    } // destroy scoped mutex -> release mutex

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }

                cv::line(im,pt1,pt2,cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for(int i=0;i<n;i++)
        {
            if(vbVO[i + Nleft] || vbMap[i + Nleft])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = mvCurrentKeysRight[i].pt / imageScale;
                    float px = mvCurrentKeysRight[i].pt.x / imageScale;
                    float py = mvCurrentKeysRight[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = mvCurrentKeysRight[i].pt;
                    pt1.x=mvCurrentKeysRight[i].pt.x-r;
                    pt1.y=mvCurrentKeysRight[i].pt.y-r;
                    pt2.x=mvCurrentKeysRight[i].pt.x+r;
                    pt2.y=mvCurrentKeysRight[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i + Nleft])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,point,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,point,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

/**
 * @brief Draw the black rectangle with white information status bar
*/
void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

/**
 * @brief Interface function called by top-level classes
 * @note example mpFrameDrawer->Update(this)
*/
void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mThDepth = pTracker->mCurrentFrame.mThDepth;
    mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;
    
    //* Keep a copy of the semantic matrix within this class
    pSemMat = pTracker->mSemMat; // Make mutex safe copy
    
    
    if(pSTAT_DB.empty()){
        pSTAT_DB = pTracker->STAT_DB;
        pDYNA_DB = pTracker->DYNA_DB;
    }
    

    // Logic for stereo image pairs
    if(both){
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    }
    else{
        N = mvCurrentKeys.size();
    }

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    //Variables for the new visualization
    mCurrentFrame = pTracker->mCurrentFrame;
    mmProjectPoints = mCurrentFrame.mmProjectPoints;
    mmMatchedInImage.clear();

    mvpLocalMap = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;

                    mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
                }
                else
                {
                    mvpOutlierMPs.push_back(pMP);
                    mvOutlierKeys.push_back(mvCurrentKeys[i]);
                }
            }
        }

    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
