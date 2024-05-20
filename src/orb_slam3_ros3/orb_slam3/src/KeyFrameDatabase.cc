/**
 * @file KeyframeDatabase.cc
 * @brief Contains implementation KeyFrameDatabase class
 * @author Original authors are attributed in README.md
 * @attention PSD Descriptor and PCB KPR both implemented in this class.
 * @attention Please refer to the "Solving Short-Term Relocalization Problems in Monocular Keyframe Visual SLAM Using Spatial and Semantic Data"
 * @attention Modified to work with Multiagent system by Azmyin Md. Kamal
 * @date 01/12/22 - 01/12/2025
*/

//* C/C++
#include<mutex>
#include<complex> // std::complex, std::norm

//* ORB-SLAM3 
#include "KeyFrameDatabase.h"
#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "ORBmatcher.h"

using namespace std; //*Bad practice, change affected areas and remove

namespace ORB_SLAM3
{

//* Class constructor
KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());
}

void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

void KeyFrameDatabase::clearMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(std::vector<list<KeyFrame*> >::iterator vit=mvInvertedFile.begin(), vend=mvInvertedFile.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =  *vit;

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend;)
        {
            KeyFrame* pKFi = *lit;
            if(pMap == pKFi->GetMap())
            {
                lit = lKFs.erase(lit);
                // Dont delete the KF because the class Map clean all the KF when it is destroyed
            }
            else
            {
                ++lit;
            }
        }
    }
}

/**
 * @brief Apply all three constraints as described in Algorithm 1 
 * @note Requires pointer to query keyframe and list of pointers to a number of keyFrame lists
*/
std::vector<KeyFrame*> KeyFrameDatabase::ApplyPCBConstraint(KeyFrame *pKFF, std::vector<KeyFrame*> kfAllList, const float &nPoseSimThres, double &nAvgFrobTime){
    // Initialize
    std::vector<KeyFrame*> vpPCBCandidates; // Keyframes filtered via full PCB constraint
    std::vector<KeyFrame*> vpPoseFilteredCandidates; // KeyFrames filtered via Pose Constraint  
    std::vector<KeyFrame*> vpClassFilteredCandidates; // Keyframes filtered via Pose and Class constraints
    std::vector<float> vpClsSimiScore; // List that holds class similarity score
    float nFrobNormVal = 0; // Frobenius norm for poses of a set of keyframes

    // Time keeping variables
    std::vector<double> vTForb;
    vTForb.reserve(kfAllList.size());

    // Apply Pose Constraint and calculate all class similarity score
    for (KeyFrame* ptrKF : kfAllList)
    {
        double nT = 0;
        
        //NOTE a keyframe may not have all static objects but it is guranteed to have a pose
        
        //nFrobNormVal = SystemUtils::calcFrobeniusNorm(pKFF->psdKfObj.mTwc, ptrKF->psdKfObj.mTwc, nT); // SVD version
        // nFrobNormVal = SystemUtils::calcFrobeniusNormDirect(pKFF->psdKfObj.mTwc, ptrKF->psdKfObj.mTwc, nT); // ||T1 - T2||_F version
        float f1 = SystemUtils::calcFrobeniusNormDirect(pKFF->psdKfObj.mTwc);
        float f2 = SystemUtils::calcFrobeniusNormDirect(ptrKF->psdKfObj.mTwc);
        nFrobNormVal = std::fabs(f1 - f2);
        vTForb.push_back(nT);
        
        if (nFrobNormVal <= nPoseSimThres){
            // Push this KeyFrame into vpPoseFilteredCandidate list
            vpPoseFilteredCandidates.push_back(ptrKF);
            // Compute L2 abs norm difference between this candidate and query keyframe
            float diffAbsNorm = std::abs(pKFF->psdKfObj.L2NormStatObjs - ptrKF->psdKfObj.L2NormStatObjs);
            vpClsSimiScore.push_back(diffAbsNorm); // Push Abs value of difference of L2 norms of class labels 
        }
    }
    
    //std::cout<<"vpPoseFilteredCandidates size: "<<vpPoseFilteredCandidates.size()<<"\n";
    
    //* Return time statistics [example of pass by reference]
    nAvgFrobTime = SystemUtils::calcAverage(vTForb);
    
    //* Did we get any KFs with Pose Constraint?
    if (vpPoseFilteredCandidates.size() != 0){
        // Condition: Reduce candidates in vpPoseFilteredCandidates further using Class Box constraints

        //* WatchDog 1: If just 1-2 candidates, return vpPoseFilteredCandidates
        if (vpPoseFilteredCandidates.size()>0 and vpPoseFilteredCandidates.size()<= 2){
            //SystemUtils::printStrMsgOneBlnk("ApplyPCB: returned vpPoseFilteredCandidates");
            return vpPoseFilteredCandidates;
        }

        // Calculate nMinClassThres, nMaxClassThres before going into Class constraint
        auto nMinClassThres = *std::min_element(vpClsSimiScore.begin(),vpClsSimiScore.end());
        auto nMaxClassThres = nMinClassThres + (nMinClassThres * 0.1); // 10% of nMinClassThres result
        
        //* Class Constraint
        for (KeyFrame* ptrKF : vpPoseFilteredCandidates){
            // Compute difference score between filtered and query frame
            float diffAbsNorm2 = std::abs(pKFF->psdKfObj.L2NormStatObjs - ptrKF->psdKfObj.L2NormStatObjs);
            
            // Is this candidate within nMinClassThres -- nMinClassThres + 0.1 threshold?
            if (diffAbsNorm2>=nMinClassThres && diffAbsNorm2<=nMaxClassThres){
                //* Yes, push to vpClassFilteredCandidates
                vpClassFilteredCandidates.push_back(ptrKF);
                
                //* Box Constraint
                int nMatchedBoxes = SystemUtils::boxMatchedBtwnQueryCandidate(pKFF->psdKfObj.BBoxCordsStatObjs,ptrKF->psdKfObj.BBoxCordsStatObjs);
                //* If number of matches is same or more (??) then it passes box constraint
                if (nMatchedBoxes >= pKFF->psdKfObj.BBoxCordsStatObjs.size()){
                    // Push this candidate to vpPCBCandidates
                    vpPCBCandidates.push_back(ptrKF);
                }   
            }  
        }

        /*
            * Check if vpPCBCandidates is empty
            * If yes check if vpClassFilteredCandidates is empty
            * If yes, return vpPoseFilteredCandidates
            * If no, return vpClassFilteredCandidates
        */
        if (vpPCBCandidates.size()==0){
            // Any keyframes filtered through Class constraint?
            if (vpClassFilteredCandidates.size() != 0){
                //SystemUtils::printStrMsgOneBlnk("ApplyPCB: returned vpClassFilteredCandidates, vpPCBCandiates == 0");
                return vpClassFilteredCandidates;
            }
            else{
                // Return list from pose constraint is empty
                //SystemUtils::printStrMsgOneBlnk("ApplyPCB: returned vpPoseFilteredCandidates, vpPCBCandiates == 0");
                return vpPoseFilteredCandidates;
            }  
        }
        else{
            // Return list of KFs passing through all three constraints
            //SystemUtils::printStrMsgOneBlnk("ApplyPCB: returned vpPCBCandidates");
            return vpPCBCandidates;
        }

    } // End If (vpPoseFilteredCandidates.size() != 0) block
    else{
         //* Case when pose constrainted itself failed 
         // NOTE: In Loop Closing, it is possible to have no matches with respect to pose, this helps to avoid system crash
         return vpPCBCandidates; // Returns a blank array  
    }
}

/**
 * @brief Applies the CB version of the PCB KPR algorithm
*/
std::vector<KeyFrame*> KeyFrameDatabase::ApplyCBConstraint(KeyFrame *pKFF, std::vector<KeyFrame*> kfAllList, const float &nPoseSimThres){
    //std::cout<<"In ApplyCBConstraint "<<std::endl;
    // Initialize
    std::vector<KeyFrame*> vpClassFilteredCandidates; // If candiate just passed class constraint
    std::vector<KeyFrame*> vpCBCandidates; // IF candidate passed both constraints
    std::vector<float> vpClsSimiScore; // List that holds class similarity score
    
    /*
        Pseudocode
        * Calculate vpClsSimiScore
        * Then for each candidate, first apply Class constraint and then Box constraint
    */

    for (KeyFrame* ptrKF : kfAllList){
        float diffAbsNorm = std::abs(pKFF->psdKfObj.L2NormStatObjs - ptrKF->psdKfObj.L2NormStatObjs);
        vpClsSimiScore.push_back(diffAbsNorm); // Push Abs value of difference of L2 norms of class labels 
    }
    
    // Calculate nMinClassThres, nMaxClassThres before going into Class constraint
    auto nMinClassThres = *std::min_element(vpClsSimiScore.begin(),vpClsSimiScore.end());
    //auto nMaxClassThres = nMinClassThres * 1.1; //* 10% of nMinClassThres value, there is a problem, this causes max threshold to become 0 if all '0' objects were seen
    auto nMaxClassThres = nMinClassThres + (nMinClassThres * 0.1); //* 10% of nMinClassThres value

    for (KeyFrame* ptrKF : kfAllList){
        // Compute difference score between filtered and query frame
        float diffAbsNorm2 = std::abs(pKFF->psdKfObj.L2NormStatObjs - ptrKF->psdKfObj.L2NormStatObjs); //! vpClsSimiScore[i] should also work
        // Is this candidate within nMinClassThres -- nMinClassThres + 0.1 threshold?
        if (diffAbsNorm2>=nMinClassThres && diffAbsNorm2<=nMaxClassThres){
            //* Yes, push to vpClassFilteredCandidates
            vpClassFilteredCandidates.push_back(ptrKF);
            
            //* Box Constraint?
            int nMatchedBoxes = SystemUtils::boxMatchedBtwnQueryCandidate(pKFF->psdKfObj.BBoxCordsStatObjs,ptrKF->psdKfObj.BBoxCordsStatObjs);
            //* If number of matches is same or more (??) then it passes box constraint
            if (nMatchedBoxes >= pKFF->psdKfObj.BBoxCordsStatObjs.size()){
                // Push this candidate to vpCBFilteredCandidates
                vpCBCandidates.push_back(ptrKF);
            }   
        }
    }

    // Output
    if (vpCBCandidates.size() != 0){
        return vpCBCandidates;
    }
    else{
        return vpClassFilteredCandidates;
    }
}

/**
 * @brief Master function that performs PCB KPR algorithm
 * @attention: Name refactored to DetectCandidatesWithPCBKPR
*/
std::vector<KeyFrame*> KeyFrameDatabase::DetectCandidatesWithPCB_KPR(Frame* F, Map* pMap){
    
    // Initialize
    std::vector<KeyFrame*> vpRelocKfs; // Output variable
    std::vector<KeyFrame*> kfAllList;
    const float nPoseSimThres = 0.5; // NOTE 0.1 seems to work well in all cases 
    double nAvgFrobTime; // Average time taken to compute frobenius norm

    // Build 'query' Keyframe
    KeyFrame *pKFF = new KeyFrame((*F),pMap);
    
    // Build its PSD descriptor
    pKFF->ComputePSD(F); // F is a pointer to the passed frame object
    
    // Pull all keyframes in the current map
    kfAllList = pMap->GetAllKeyFrames();

    vpRelocKfs.reserve(kfAllList.size()); // Not sure but allows to return vpRelocKFs correctly
    //std::cout<<"kfAlllist count: "<<kfAllList.size()<<"\n";

    //* This value be very close to zero or exactly zero if F->GetPose() was only initialized
    //! TODO check what happens if we use pKFF->GetPoseInverse() i.e pose with respect to world coordinate frame not camera coordinate frame
    float nTest = (SystemUtils::sophusToEigenMat4f(F->GetPose()) - eigenMat4f::Identity(4,4)).norm(); // Take L2 norm of a matrix
    
    // std::cout<<std::endl;
    // std::cout<<"nTest: "<<nTest<<"\n\n";

    //* Check if Frame F's pose is identity or some other value
    if (nTest > 0){
        // Condition 1: nTest not identity, using mnLastFrame, using full PCB constraint
        
        //std::cout<<"nTest not zero, calling ApplyPCBConstraint "<<std::endl;
        // vpRelocKfs = KeyFrameDatabase::ApplyPCBConstraint(pKFF, kfAllList, nPoseSimThres);
        vpRelocKfs = KeyFrameDatabase::ApplyPCBConstraint(pKFF, kfAllList, nPoseSimThres, nAvgFrobTime);
        // SystemUtils::printDoubleTimeValueWithMsg("nAvgFrobTime:", nAvgFrobTime);

        // For debugging
        // std::cout<<"KFs found using PCB KPR: "<<vpRelocKfs.size()<<std::endl;
        
        //SystemUtils::printTwoBlankLines();
        return vpRelocKfs;
    }
    
    else{
        //std::cout<<"nTest zero, calling ApplyCBConstraint "<<std::endl;
        vpRelocKfs = KeyFrameDatabase::ApplyCBConstraint(pKFF, kfAllList, nPoseSimThres);
        
        std::cout<<"KFs found using CB method: "<<vpRelocKfs.size()<<std::endl;
        // for (KeyFrame* ptrTest : vpRelocKfs){
        //     std::cout<<"Returned KF ID: "<<ptrTest->psdKfObj.kfID<<std::endl;
        // }

        //SystemUtils::printTwoBlankLines();
        return vpRelocKfs;
    }
}

// ! ----------------------------------- Abandoned ------------------------------
/**
 * @brief Detects Loop closure candidates using PSD descriptor and PCB KPR algorithm
 * All these methods are part of a chain
 * Experimental, this needs more testing
*/
void KeyFrameDatabase::DetectLoopCandidatesWithSemantics(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, Map* pMap){
    //! DEPRICITED DOES NOT WORK
    // Method that determines whether the current keyframe is viewing a region previously visited
   
    // Initialize
    std::vector<KeyFrame*> vpCovisibleList;
    set<KeyFrame*> spConnectedKF;
    
    //* Parameters
    int minCovisibleFramesToReturn = 3; // Keeping it same as ORB_SLAM3 VI A point 5 "Verification of three covisible frame" 
    //const float nLoopPoseSimThres = 0.1; // 0.1 gives about 1 - 4 candidates, original
    const float nLoopPoseSimThres = 0.5; // 0.1 gives about 1 - 4 candidates, 2.5 gives how many
    
    // Resize output std::vector
    vpLoopCand.reserve(minCovisibleFramesToReturn); // Ensures at least the vector has memory allocated to return 3 covisible keyframes

    // Mutex safe execution
    {
        unique_lock<mutex> lock(mMutex);
        
        //vpCovisibleList = pKF->GetBestCovisibilityKeyFrames(10); // This is slowing down stuff
        vpCovisibleList = pMap->GetAllKeyFrames(); // DO NOT DELETE

        // Pull all the keyframes that is connected to this keyframe
        //spConnectedKF = pKF->GetConnectedKeyFrames();
        //std::vector<KeyFrame*> vpCovisibleList(spConnectedKF.begin(), spConnectedKF.end()); // Initialize vector from a list  
    }

    //* Only Candidates that goes through all 3 constraints
    // vpLoopCand = KeyFrameDatabase::ApplyLoopPCBConstraint(pKF, vpCovisibleList, nLoopPoseSimThres);
    
    //* Candidates that goes through class and box constraints
    vpLoopCand = KeyFrameDatabase::ApplyCBConstraint(pKF, vpCovisibleList, nLoopPoseSimThres);

    //* Relocalization's PCB constraint
    //vpLoopCand = KeyFrameDatabase::ApplyPCBConstraint(pKF, vpCovisibleList, nLoopPoseSimThres);
}

std::vector<KeyFrame*> KeyFrameDatabase::ApplyLoopPCBConstraint(KeyFrame *pKFF, std::vector<KeyFrame*> kfAllList, const float &nPoseSimThres){
    //* Modified version of ApplyPCBConstraint method
    //? Big idea only return keyframes that goes through FULL PCB does it work??
    
    // Initialize
    std::vector<KeyFrame*> vpPCBCandidates; // Keyframes filtered via full PCB constraint
    std::vector<KeyFrame*> vpPoseFilteredCandidates; // KeyFrames filtered via Pose Constraint  
    std::vector<KeyFrame*> vpClassFilteredCandidates; // Keyframes filtered via Pose and Class constraints
    std::vector<float> vpClsSimiScore; // List that holds class similarity score
    float nFrobNormVal = 0; // Frobenius norm for poses of a set of keyframes

    // Apply Pose Constraint and calculate all class similarity score
    for (KeyFrame* ptrKF : kfAllList)
    {
        double nT = 0;
        //NOTE a keyframe may not have all static objects but it is guranteed to have a pose
        nFrobNormVal = SystemUtils::calcFrobeniusNorm(pKFF->psdKfObj.mTwc, ptrKF->psdKfObj.mTwc, nT);
        
        if (nFrobNormVal <= nPoseSimThres){
            // Push this KeyFrame into vpPoseFilteredCandidate list
            vpPoseFilteredCandidates.push_back(ptrKF);
            // Compute L2 abs norm difference between this candidate and query keyframe
            float diffAbsNorm = std::abs(pKFF->psdKfObj.L2NormStatObjs - ptrKF->psdKfObj.L2NormStatObjs);
            vpClsSimiScore.push_back(diffAbsNorm); // Push Abs value of difference of L2 norms of class labels 
        }
    }
    
    //std::cout<<"vpPoseFilteredCandidates size: "<<vpPoseFilteredCandidates.size()<<"\n";

    //* Did we get any KFs with Pose Constraint?
    if (vpPoseFilteredCandidates.size() != 0){
        // Condition: Reduce candidates in vpPoseFilteredCandidates further using Class Box constraints

        // Calculate nMinClassThres, nMaxClassThres before going into Class constraint
        auto nMinClassThres = *std::min_element(vpClsSimiScore.begin(),vpClsSimiScore.end());
        auto nMaxClassThres = nMinClassThres * 1.1; //* 10% of nMinClassThres value
        
        //* Class Constraint
        for (KeyFrame* ptrKF : vpPoseFilteredCandidates){
            // Compute difference score between filtered and query frame
            float diffAbsNorm2 = std::abs(pKFF->psdKfObj.L2NormStatObjs - ptrKF->psdKfObj.L2NormStatObjs);
            
            // Is this candidate within nMinClassThres -- nMinClassThres + 0.1 threshold?
            if (diffAbsNorm2>=nMinClassThres && diffAbsNorm2<=nMaxClassThres){
                // Yes, push to vpClassFilteredCandidates
                // vpClassFilteredCandidates.push_back(ptrKF);
                
                //* Box Constraint
                int nMatchedBoxes = SystemUtils::boxMatchedBtwnQueryCandidate(pKFF->psdKfObj.BBoxCordsStatObjs,ptrKF->psdKfObj.BBoxCordsStatObjs);
                
                //? Geometric Map Point matching or 3D aligning transformation
                
                //* If number of matches is same or more (??) then it passes box constraint
                if (nMatchedBoxes >= pKFF->psdKfObj.BBoxCordsStatObjs.size()){
                    // Push this candidate to vpPCBCandidates
                    vpPCBCandidates.push_back(ptrKF);
                }   
            }  
        }

        // Only return candidates that goes through vpPCBCandidates
        return vpPCBCandidates;

    } // End If (vpPoseFilteredCandidates.size() != 0) block
    else{
         //* return blank array
         return vpPCBCandidates; // Returns a blank array  
    }
}

//bool KeyFrameDatabase::FireLoopClosure(KeyFrame *pKFOrigin, std::vector<KeyFrame*> &vpLoopCand){
bool KeyFrameDatabase::FireLoopClosure(Map* pMap, std::vector<KeyFrame*> &vpLoopCand){
    // Watchdog 1: Return false if there are no members in vpLoopCand
    if (vpLoopCand.empty()){
        return false;
    }
    
    // Initialize work variables
    float nFrobNormVal = 0;
    float nGoodThres = 0.5; // 0.5 works in SX-2, Seems to be a threshold that needs to be learned
    int mnBoWMatchThres = 50; // Is this a good number for all datasets
    KeyFrame* pKFOrigin;
    std::vector<KeyFrame*> vpGoodKFs;
    std::vector<float> pKFOriginClassList;
    stdVecInVecsFloat pKFOriginBBoxCordsStatObjs;
    std::vector<float> vpClsSimiScore; // List that holds class similarity score
    int nCandWithGoodBowMatch = 0;
    ORBmatcher matcherBoW(0.90, true); // What happens if we try to count 3D map point matches

    //* Get origin keyframe
    {
        unique_lock<mutex> lock(mMutex);
        pKFOrigin = pMap->GetOriginKF(); // mpLastMap->GetFirstKeyFrame(); to get first keyframe after initalization
        
        //pKFOrigin = pMap->GetFirstKeyFrame();
        pKFOriginClassList = pKFOrigin->psdKfObj.StatObjs;
        pKFOriginBBoxCordsStatObjs = pKFOrigin->psdKfObj.BBoxCordsStatObjs;
    }
    

    for (KeyFrame* ptrKF : vpLoopCand){
        float diffAbsNorm = std::abs(pKFOrigin->psdKfObj.L2NormStatObjs - ptrKF->psdKfObj.L2NormStatObjs);
        vpClsSimiScore.push_back(diffAbsNorm); // Push Abs value of difference of L2 norms of class labels 
    }
    
    // Calculate nMinClassThres, nMaxClassThres before going into Class constraint
    auto nMinClassThres = *std::min_element(vpClsSimiScore.begin(),vpClsSimiScore.end());
    auto nMaxClassThres = 1 + (nMinClassThres * 0.25); //* 25% of nMinClassThres value
    // heuristic solution

    // Watchdog, sometimes nMinClassThres is just 0

    //std::cout<<"Running FireLoopClosure\n";
    // Cycle through each Keyframe candidate and set true if pose is very close to pKFOrigin
    for (KeyFrame* ptrKF: vpLoopCand){
        
        // Watchdog: Discard if candidate's KeyFrame ID is as same as Origin's
        if(pKFOrigin->mnId == ptrKF->mnId){
            continue;
        }

        double nT = 0;

        nFrobNormVal = SystemUtils::calcFrobeniusNorm(pKFOrigin->psdKfObj.mTwc, ptrKF->psdKfObj.mTwc, nT);
        // SystemUtils::printFloatComparatorValueBtnTwoKFs(pKFOrigin, ptrKF, nFrobNormVal);
        
        //* Original, works but is calling Loop Closure a lot [DO NOT DELETE]
        // if (nFrobNormVal >= 0 and nFrobNormVal<=nGoodThres){
        //     return true; // If any of the candidates shows true we fire at once
        // }

        //* Method 2, does not fire loop
        if (nFrobNormVal >= 0 and nFrobNormVal<=nGoodThres){ 
            // Prepare work variables
            std::vector<MapPoint*> vpMatchedMPs;
            int num = matcherBoW.SearchByBoW(pKFOrigin, ptrKF, vpMatchedMPs);
            float num2 = float (num);

            //* Add a heuristic counter??

            //std::cout<<"matcherBow: ";
            //SystemUtils::printFloatComparatorValueBtnTwoKFs(pKFOrigin, ptrKF, num2);
            // std::cout<<"\n";
            
            if (num2 >= mnBoWMatchThres){
                //? If prtKF does not have a number of static objects this would be evaluated false
                float diffAbsNorm2 = std::abs(pKFOrigin->psdKfObj.L2NormStatObjs - ptrKF->psdKfObj.L2NormStatObjs);

                // std::cout<<"nMinClassThres: "<< nMinClassThres<< std::endl;
                // std::cout<<"nMaxClassThres: "<< nMaxClassThres<< std::endl;
                // std::cout<<"diffAbsNorm2: "<< diffAbsNorm2 << std::endl;
                // std::cout<<"\n";

                nCandWithGoodBowMatch = nCandWithGoodBowMatch + 1;

                if (diffAbsNorm2>=nMinClassThres && diffAbsNorm2<nMaxClassThres){  
                    int nMatchedBoxes = SystemUtils::boxMatchedBtwnQueryCandidate(pKFOriginBBoxCordsStatObjs,ptrKF->psdKfObj.BBoxCordsStatObjs, 0.8);
                    float nMB = float (nMatchedBoxes);

                    // std::cout<<"nMatchedBoxes: ";
                    // SystemUtils::printFloatComparatorValueBtnTwoKFs(pKFOrigin, ptrKF, nMB);
                    // std::cout<<"pKFOrigin->psdKfObj.BBoxCordsStatObjs: "<<pKFOrigin->psdKfObj.BBoxCordsStatObjs.size()<<std::endl;
                    // std::cout<<"\n";

                    //* Immediately return true
                    if (nMatchedBoxes >= pKFOrigin->psdKfObj.BBoxCordsStatObjs.size()){
                        return true;
                    }
                }
                else{
                    continue;
                }
            } // end of mnBoWMatch   
        } // end of frob threshold check
    } // end of main for loop
    
    // WatchDog: At least 2 of the candidates had good BoW based MapPoint matches
    if (nCandWithGoodBowMatch>= 2){
        return true;
    }

    // All checks failed, do no loop closure
    return false; // Ensure we do not raise error -11        

}

// ! ----------------------------------- Abandoned ------------------------------

// Old Place detection method that got depricited in ORB_SLAM 3
void KeyFrameDatabase::DetectCandidates(KeyFrame* pKF, float minScore,vector<KeyFrame*>& vpLoopCand, vector<KeyFrame*>& vpMergeCand)
{
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWordsLoop,lKFsSharingWordsMerge;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->GetMap()==pKF->GetMap()) // For consider a loop candidate it a candidate it must be in the same map
                {
                    if(pKFi->mnLoopQuery!=pKF->mnId)
                    {
                        pKFi->mnLoopWords=0;
                        if(!spConnectedKeyFrames.count(pKFi))
                        {
                            pKFi->mnLoopQuery=pKF->mnId;
                            lKFsSharingWordsLoop.push_back(pKFi);
                        }
                    }
                    pKFi->mnLoopWords++;
                }
                else if(!pKFi->GetMap()->IsBad())
                {
                    if(pKFi->mnMergeQuery!=pKF->mnId)
                    {
                        pKFi->mnMergeWords=0;
                        if(!spConnectedKeyFrames.count(pKFi))
                        {
                            pKFi->mnMergeQuery=pKF->mnId;
                            lKFsSharingWordsMerge.push_back(pKFi);
                        }
                    }
                    pKFi->mnMergeWords++;
                }
            }
        }
    }

    if(lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty())
        return;

    if(!lKFsSharingWordsLoop.empty())
    {
        list<pair<float,KeyFrame*> > lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        int maxCommonWords=0;
        for(list<KeyFrame*>::iterator lit=lKFsSharingWordsLoop.begin(), lend= lKFsSharingWordsLoop.end(); lit!=lend; lit++)
        {
            if((*lit)->mnLoopWords>maxCommonWords)
                maxCommonWords=(*lit)->mnLoopWords;
        }

        int minCommonWords = maxCommonWords*0.8f;

        int nscores=0;

        // Compute similarity score. Retain the matches whose score is higher than minScore
        for(list<KeyFrame*>::iterator lit=lKFsSharingWordsLoop.begin(), lend= lKFsSharingWordsLoop.end(); lit!=lend; lit++)
        {
            KeyFrame* pKFi = *lit;

            if(pKFi->mnLoopWords>minCommonWords)
            {
                nscores++;

                float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

                pKFi->mLoopScore = si;
                if(si>=minScore)
                    lScoreAndMatch.push_back(make_pair(si,pKFi));
            }
        }

        if(!lScoreAndMatch.empty())
        {
            list<pair<float,KeyFrame*> > lAccScoreAndMatch;
            float bestAccScore = minScore;

            // Lets now accumulate score by covisibility
            for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
            {
                KeyFrame* pKFi = it->second;
                vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                float bestScore = it->first;
                float accScore = it->first;
                KeyFrame* pBestKF = pKFi;
                for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
                {
                    KeyFrame* pKF2 = *vit;
                    if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
                    {
                        accScore+=pKF2->mLoopScore;
                        if(pKF2->mLoopScore>bestScore)
                        {
                            pBestKF=pKF2;
                            bestScore = pKF2->mLoopScore;
                        }
                    }
                }

                lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
                if(accScore>bestAccScore)
                    bestAccScore=accScore;
            }

            // Return all those keyframes with a score higher than 0.75*bestScore
            float minScoreToRetain = 0.75f*bestAccScore;

            set<KeyFrame*> spAlreadyAddedKF;
            vpLoopCand.reserve(lAccScoreAndMatch.size());

            for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
            {
                if(it->first>minScoreToRetain)
                {
                    KeyFrame* pKFi = it->second;
                    if(!spAlreadyAddedKF.count(pKFi))
                    {
                        vpLoopCand.push_back(pKFi);
                        spAlreadyAddedKF.insert(pKFi);
                    }
                }
            }
        }

    }

    if(!lKFsSharingWordsMerge.empty())
    {
        list<pair<float,KeyFrame*> > lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        int maxCommonWords=0;
        for(list<KeyFrame*>::iterator lit=lKFsSharingWordsMerge.begin(), lend=lKFsSharingWordsMerge.end(); lit!=lend; lit++)
        {
            if((*lit)->mnMergeWords>maxCommonWords)
                maxCommonWords=(*lit)->mnMergeWords;
        }

        int minCommonWords = maxCommonWords*0.8f;

        int nscores=0;

        // Compute similarity score. Retain the matches whose score is higher than minScore
        for(list<KeyFrame*>::iterator lit=lKFsSharingWordsMerge.begin(), lend=lKFsSharingWordsMerge.end(); lit!=lend; lit++)
        {
            KeyFrame* pKFi = *lit;

            if(pKFi->mnMergeWords>minCommonWords)
            {
                nscores++;

                float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

                pKFi->mMergeScore = si;
                if(si>=minScore)
                    lScoreAndMatch.push_back(make_pair(si,pKFi));
            }
        }

        if(!lScoreAndMatch.empty())
        {
            list<pair<float,KeyFrame*> > lAccScoreAndMatch;
            float bestAccScore = minScore;

            // Lets now accumulate score by covisibility
            for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
            {
                KeyFrame* pKFi = it->second;
                vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                float bestScore = it->first;
                float accScore = it->first;
                KeyFrame* pBestKF = pKFi;
                for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
                {
                    KeyFrame* pKF2 = *vit;
                    if(pKF2->mnMergeQuery==pKF->mnId && pKF2->mnMergeWords>minCommonWords)
                    {
                        accScore+=pKF2->mMergeScore;
                        if(pKF2->mMergeScore>bestScore)
                        {
                            pBestKF=pKF2;
                            bestScore = pKF2->mMergeScore;
                        }
                    }
                }

                lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
                if(accScore>bestAccScore)
                    bestAccScore=accScore;
            }

            // Return all those keyframes with a score higher than 0.75*bestScore
            float minScoreToRetain = 0.75f*bestAccScore;

            set<KeyFrame*> spAlreadyAddedKF;
            vpMergeCand.reserve(lAccScoreAndMatch.size());

            for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
            {
                if(it->first>minScoreToRetain)
                {
                    KeyFrame* pKFi = it->second;
                    if(!spAlreadyAddedKF.count(pKFi))
                    {
                        vpMergeCand.push_back(pKFi);
                        spAlreadyAddedKF.insert(pKFi);
                    }
                }
            }
        }

    }

    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
    {
        list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            KeyFrame* pKFi=*lit;
            pKFi->mnLoopQuery=-1;
            pKFi->mnMergeQuery=-1;
        }
    }

}

void KeyFrameDatabase::DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nMinWords)
{
    list<KeyFrame*> lKFsSharingWords;
    set<KeyFrame*> spConnectedKF;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        spConnectedKF = pKF->GetConnectedKeyFrames();

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(spConnectedKF.find(pKFi) != spConnectedKF.end())
                {
                    continue;
                }
                if(pKFi->mnPlaceRecognitionQuery!=pKF->mnId)
                {
                    pKFi->mnPlaceRecognitionWords=0;
                    pKFi->mnPlaceRecognitionQuery=pKF->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
               pKFi->mnPlaceRecognitionWords++;

            }
        }
    }
    if(lKFsSharingWords.empty())
        return;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnPlaceRecognitionWords>maxCommonWords)
            maxCommonWords=(*lit)->mnPlaceRecognitionWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    if(minCommonWords < nMinWords)
    {
        minCommonWords = nMinWords;
    }

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnPlaceRecognitionWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);
            pKFi->mPlaceRecognitionScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return;

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnPlaceRecognitionQuery!=pKF->mnId)
                continue;

            accScore+=pKF2->mPlaceRecognitionScore;
            if(pKF2->mPlaceRecognitionScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mPlaceRecognitionScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vpLoopCand.reserve(lAccScoreAndMatch.size());
    vpMergeCand.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                if(pKF->GetMap() == pKFi->GetMap())
                {
                    vpLoopCand.push_back(pKFi);
                }
                else
                {
                    vpMergeCand.push_back(pKFi);
                }
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
}

bool compFirst(const pair<float, KeyFrame*> & a, const pair<float, KeyFrame*> & b)
{
    return a.first > b.first;
}

//* Newer Place Recognition technique in ORB_SLAM3
void KeyFrameDatabase::DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates)
{
    /*
        * KeyFrame *pKF -- Query keyframe
        * vector<KeyFrame*> &vpLoopCand -- vector of pointers to keyframes that is considered for loop closure
        * vector<KeyFrame*> &vpMergeCand -- vector of pointers to keyframes that is considered for map merging
        * int nNumCandidates -- How many covisible keyframes needs to be returned
    */
    
    list<KeyFrame*> lKFsSharingWords;
    set<KeyFrame*> spConnectedKF;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        spConnectedKF = pKF->GetConnectedKeyFrames();

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;

                if(pKFi->mnPlaceRecognitionQuery!=pKF->mnId)
                {
                    pKFi->mnPlaceRecognitionWords=0;
                    if(!spConnectedKF.count(pKFi))
                    {

                        pKFi->mnPlaceRecognitionQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnPlaceRecognitionWords++;
            }
        }
    }
    
    if(lKFsSharingWords.empty())
        return;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnPlaceRecognitionWords>maxCommonWords)
            maxCommonWords=(*lit)->mnPlaceRecognitionWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnPlaceRecognitionWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);
            pKFi->mPlaceRecognitionScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return;

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnPlaceRecognitionQuery!=pKF->mnId)
                continue;

            accScore+=pKF2->mPlaceRecognitionScore;
            if(pKF2->mPlaceRecognitionScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mPlaceRecognitionScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    lAccScoreAndMatch.sort(compFirst);

    vpLoopCand.reserve(nNumCandidates); //* Makes space for at least three candidates
    vpMergeCand.reserve(nNumCandidates);
    set<KeyFrame*> spAlreadyAddedKF;
    int i = 0;
    list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin();
    while(i < lAccScoreAndMatch.size() && (vpLoopCand.size() < nNumCandidates || vpMergeCand.size() < nNumCandidates))
    {
        KeyFrame* pKFi = it->second;
        if(pKFi->isBad())
            continue;

        if(!spAlreadyAddedKF.count(pKFi))
        {
            if(pKF->GetMap() == pKFi->GetMap() && vpLoopCand.size() < nNumCandidates)
            {
                vpLoopCand.push_back(pKFi); //* Condition to add the i^th keyframe into vpLoopCand which is an alias to vpLoopBowCand
            }
            else if(pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < nNumCandidates && !pKFi->GetMap()->IsBad())
            {
                vpMergeCand.push_back(pKFi);
            }
            spAlreadyAddedKF.insert(pKFi);
        }
        i++;
        it++;
    }
}

// Place Recognition for recovering from Tracking loss using Bag-of-Words in ORB_SLAM3
// Name refactored to DetectCandidatesWithDBoW2KPR
vector<KeyFrame*> KeyFrameDatabase::DetectCandidatesWithDBoW2_KPR(Frame *F, Map* pMap)
{
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch; // Python tuple like

    int nscores=0;

    // Compute similarity score.
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size()); // It indicates that the vector is created such that it can store at least the number of the specified elements without having to reallocate memory
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if (pKFi->GetMap() != pMap)
                continue;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

// Setter method
void KeyFrameDatabase::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    ORBVocabulary** ptr;
    ptr = (ORBVocabulary**)( &mpVoc );
    *ptr = pORBVoc;

    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

} //namespace ORB_SLAM