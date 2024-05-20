/**
 * @file CommonStructs.h
 * @brief Common struct object used across multiple classes
 * @author Azmyin Md. Kamal
 * @date 01/12/24
*/

#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

//* C/C++
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <unordered_map> 
#include <iomanip>

//* Eigen / Sophus C++
#include <sophus/se3.hpp>

//* Custom Aliases uses through out the system
#include "Alias.h" 

// Struct defintions

/**
 * @brief VSLAM parameters sent out by Python node
 * @attention Used in IEEE AIM 2024 paper
*/
struct vslamParameters
{
    std::string experimentName; // Name of experiment
    std::string sequenceName; // Name of the image sequence
    std::string queryKey; // Query key for the obj_database.yaml in /global_yamls
    std::string modelName; // Model i.e "bag" or "sss"
    std::unordered_map<float, std::string> STAT_DB; // Class names of static objects
    std::unordered_map<float, std::string> DYNA_DB; // Class names of dynamic objects
    std::vector<int> mvInFrameList;
    int nInduceReloc = -1; // Frame Id to manually induce relocalization. -1 ==> no manual induction
    bool doRelocExp = false; // To perform IROS 23 relocalization experiment
};

/**
 * @brief Propsed keyframe data structure dubbed the Pose Semantic Descriptor
 * @attention See Section III part A for more details
*/
struct psdStruct
{
    long unsigned int kfID; // Keyframe ID associated with this SSA
    std::vector<float> StatObjs; // List of static objects for this keyframe
    float L2NormStatObjs; // Absolute value of L2 norm of Static class ids
    ORB_SLAM3::stdVecInVecsFloat BBoxCordsStatObjs; // Bounding box coordinates of static objects
    ORB_SLAM3::eigenMatXf mTwc; // 4x4 homogenous version of pose in world coordinate frame
    Eigen::Quaternionf UnitQuat; // 1x7 version of pose in world coordinate frame 
};

// TODO depriciate
// struct expParamNSF24
// {
//     std::string sequenceName; // Image sequence name
//     std::string modelName; // Model i.e "bag" or "sss"
//     std::unordered_map<float, std::string> STAT_DB; // Class names of static objects
//     std::unordered_map<float, std::string> DYNA_DB; // Class names of dynamic objects
// };

// -------------- TODO delete these ----------------------------------

struct ROBOT1KF {
    // Objects to store keyframe / frames recevied from robot1
    // ```d``` is coordinate frame for robot1, ```n``` is an arbitary counter
    int n; // An integer id,0 points to the last keyframe in robot1`s Map
    Sophus::SE3f relTdn; // Relative pose w.r.t last frame/keyframe, when multiplied from left 
    Sophus::SE3f Tcw; // Pose of this n^th robot1 keyframe in robot0's camera coordinate
};

// For TFTN library, modified to work with SLAM pipeline
// TODO convert this to a class that will contain all properties on Object level 
// after we are successful with the multi agent framework

struct Object2DBBox {
    std::string objectName;
    int obj_id;        // Integer representation of the object in neural networks database 
    int x1, y1, x2, y2;  // To-left corner (x1,y1) and bottom-right corner (x2, y2)
    int x, y, w, h;   // Top left (x,y) width, height
    int cx, cy;       // 2D bounding box center
    cv::Mat surfNormals;    // Surface normals of all pixels in this object, openCV
    cv::Mat surfCvImage;    // cv::Image of surface normals
    Eigen::Matrix3d R; // 3x3 rotaiton estimation of this object
};

// -------------- TODO delete these ----------------------------------

#endif // COMMON_STRUCTS_H