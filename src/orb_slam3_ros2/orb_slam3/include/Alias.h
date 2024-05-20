/**
 * @file Alias.h
 * @brief Common struct object used across multiple classes
 * @author Azmyin Md. Kamal
 * @date 01/12/22
*/

#ifndef Alias_H
#define Alias_H

namespace ORB_SLAM3
{
    // Alias inside this class
    //using eigenMatFloatDynamic = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using eigenMatXf =  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>; 
    using eigenVecFloat = Eigen::VectorXf;
    using stdCVKey = std::vector<cv::KeyPoint>;
    using eigenMat3f = Eigen::Matrix3f;
    using eigenMat4f = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;
    using sophusSE3f = Sophus::SE3<float>;
    using JacobiSVDMat3f = Eigen::JacobiSVD<Eigen::MatrixXf>; // TODO depriciate this
    using stdVecInVecsFloat = std::vector<std::vector<float>>;
    using eigenJacobiMatXd = Eigen::JacobiSVD<Eigen::MatrixXd>;
    using eigenJacobiMatXf = Eigen::JacobiSVD<Eigen::MatrixXf>;
    using eigenMapToEigenVecFloat = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>;
    
}

#endif