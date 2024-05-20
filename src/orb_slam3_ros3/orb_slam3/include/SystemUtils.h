/**
 * @file SystemUtils.h
 * @brief Definition of various utility methods, mostly for debugging
 * @author Azmyin Md. Kamal
 * @date 01/12/22 - 01/12/2025
 */

// https://stackoverflow.com/questions/16851227/same-header-file-in-multiple-source-files-in-a-c-program
#ifndef SystemUtils_H
#define SystemUtils_H

// Define includes
#include <iostream>
# include <iomanip> // To debug print full precision of a double

// Include Eigen
// Quick reference: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense> // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header file
#include "sophus/se3.hpp" // For Sophus::SE3f 
#include <opencv2/opencv.hpp> // Needed to define cv::Mat object
#include<opencv2/core/core.hpp>
#include<vector>
#include <unordered_map>

#include "Alias.h" // Rename to something else

#include "KeyFrame.h" // Also imports Frame.h

#define pass (void)0 // Python's equivalent of No operation

// Default eigenMatrixf is column major, we use row-major throughout the code

//* Bring in specific aliases from ORB_SLAM3 namespace
using ORB_SLAM3::eigenMatXf;
using ORB_SLAM3::eigenMat4f; // Used in KeyFrameDatabase
using ORB_SLAM3::sophusSE3f; // Sophus::SE3<float>
using ORB_SLAM3::stdVecInVecsFloat;
using ORB_SLAM3::eigenJacobiMatXf;
using ORB_SLAM3::eigenMapToEigenVecFloat;

namespace SystemUtils
{
    // Aliases
    
    // Debug tools
    
    void printTwoBlankLines(); // Prints two blank lines in console
    void printBoolean(std::string msg, bool val); // Prints a message and a boolean value
    void printStrMsgOneBlnk (std::string msg); // Prints a string message and a blank line
    void printEigenRowColNum (const int &row, const int &col); // Prints number of rows and columns in an Eigen matrix
    void printFrameId(const int &val); // Prints the frame ID number
    std::vector<float> eigenFloatColtoStdVec(const eigenMatXf &mat, const int &col); // Converts a Eigen matrix column to a std:vector in floats
    void printStdVecFloat(std::vector<float> &vec1); // Prints out elements of a std::vector<float>
    void printStdVecFloat(const std::vector<float> &vec1); // Overloaded version
    void printStdVecDouble(const std::vector<double> &vec1);
    void printStdVecInt(const std::vector<int> &vec1); // Integer version
    void printStdVecInt(std::vector<int> &vec1); // Integer version
    void printStdVecFloatSize(std::vector<float> &vec1);
    void printEigenQuaternionFloat(Eigen::Quaternionf &q); // Prints to console, a 3D position in unit quaternion, qx,qy,qz,qw format
    // void printSE3(eigenMatXf mat); // Need to call
    void printSE3fMatrix(sophusSE3f &mat); // Prints out the contents of a 4x4 Sophus::SE3f matrix

    // Prints out the status of the 5 condition used to assess keyframe creation
    void printKeyFrameConditionStatus(const bool &c1a, const bool &c1b, const bool &c1c, const bool &c2, const bool &c3, const bool &c4, const bool &c5); 
    
    // Print out elements of a vector in vectors {2D matrix in vector in vectors form}
    void printStdVecInVecsFloatMemEff(const stdVecInVecsFloat &vec); // Mason's memory efficient version
    void printStdVecInVecsFloat(const stdVecInVecsFloat &vec1); //Azmyin's version
    void printKFSSAInStdVec(std::vector<ORB_SLAM3::KeyFrame*> &kfList); // Prints out the SSA (or SSS) in the
    //void printFrameID(Frame *ptrFrame); // Takes in pointer to a Frame and prints its ID
    void printFrameID(long unsigned int *fID); // Aliased version
    void printKFIDsInStdVec(std::vector<ORB_SLAM3::KeyFrame*> &vpKFs); // Prints KeyFrame ID in the std::vector<KeyFrame*> container
    void printFloatComparatorValueBtnTwoKFs(ORB_SLAM3::KeyFrame* pKFF, ORB_SLAM3::KeyFrame* ptrKF, float &value);
    void printDoubleTimeValueWithMsg(std::string msg, double &val);
    void printDatabase(std::unordered_map<float, std::string> &DB, std::string db_name);

    //* Helper functions
    std::vector<float> filterFromStdVecFloat(const std::vector<float> &vec1, const std::vector<float> &vec2);
    eigenMat4f sophusToEigenMat4f(sophusSE3f mat); // Convert a Sophus matrix to Eigen::Matrixf object with row-major order
    stdVecInVecsFloat extractBBoxCordsForKnownList(const eigenMatXf &semMat, const std::vector<float> &vec2);
    float intersectionOverUnion(std::vector<float> &box1, std::vector<float> &box2);
    std::vector<sophusSE3f> reverseStdVecSophusMats(std::vector<sophusSE3f> &vec_in);

    // Function that returns how many bboxes in query frame is >90% area match with bboxes in the candidate frame
    int boxMatchedBtwnQueryCandidate(stdVecInVecsFloat &query_boxes, stdVecInVecsFloat &candidate_boxes);
    int boxMatchedBtwnQueryCandidate(stdVecInVecsFloat &query_boxes, stdVecInVecsFloat &candidate_boxes, float matchThres);

    std::vector<int> ChooseFramesToInduceRelocalization(int nToChoose, int nLowerBound, int nUpperBound); // Function to create a list of random Frame Id to simuilate short-term relocalization
    std::string findClassNameFromBD(float &cls_id, std::unordered_map<float, std::string> &pSTAT_DB, std::unordered_map<float, std::string> &pDYNA_DB);
    bool inStdIntVec(int item,std::vector<int> &vec); // Function that checks if an integer is present in a std::vector<int> container
    void extractClassIdsInStdVecFloat(std::unordered_map<float, std::string> &STAT_DB,std::unordered_map<float, std::string> &DYNA_DB,std::vector<float> &static_obj_db, std::vector<float> &dynamic_obj_db);
    std::vector<std::string> splitString(const std::string str, char splitter);

    //* Linear algebra tools
    float calcFrobeniusNorm(eigenMatXf &mat);
    float calcFrobeniusNorm(eigenMatXf &mat1, eigenMatXf &mat2, double &nTime); // SVD method
    float calcFrobeniusNormDirect(eigenMatXf &mat); // Direct method
    float calcFrobeniusNormDirect(eigenMatXf &mat1, eigenMatXf &mat2, double &nTime); // Version with time keeper codes
    float calcFrobeniusNormDirect(eigenMatXf &mat1, eigenMatXf &mat2); // Version with no time keepers
    float calcL2NormFromStdVecFloat(std::vector<float> &vec);

    //* Statistics tools
    double calcAverage(std::vector<double> v_times);
    double calcDeviation(std::vector<double> v_times, double average);
    double calcAverage(std::vector<int> v_values);
    double calcDeviation(std::vector<int> v_values, double average);  
}


#endif