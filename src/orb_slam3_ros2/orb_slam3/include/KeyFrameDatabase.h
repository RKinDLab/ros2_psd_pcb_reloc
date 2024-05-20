/**
 * @file KeyframeDatabase.h
 * @brief Contains definitions for KeyframeDatabase.cc
 * @author Original authors are attributed in README.md
 * @attention PSD Descriptor and PCB KPR both implemented in this class.
 * @attention Modified to work with Multiagent system by Azmyin Md. Kamal
 * @date 01/12/22 - 01/12/2025
*/

/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
** Modified by Azmyin Md. Kamal
*/


#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>
#include <cmath> // std::abs()

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>

#include<mutex>
#include "SystemUtils.h" // Brings in useful debugging tools, user defined aliases


namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class Map;


class KeyFrameDatabase
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mvBackupInvertedFileId;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrameDatabase(){}
    KeyFrameDatabase(const ORBVocabulary &voc);

    void add(KeyFrame* pKF);
    void erase(KeyFrame* pKF);
    void clear();
    void clearMap(Map* pMap);

    //* PSD Keyframes and PCB KPR algorithm for short-term relocalization
    std::vector<KeyFrame*> ApplyPCBConstraint(KeyFrame *pKFF, std::vector<KeyFrame*> kfAllList, const float &nPoseSimThres, double &nAvgFrobTime);
    std::vector<KeyFrame*> ApplyCBConstraint(KeyFrame *pKFF, std::vector<KeyFrame*> kfAllList, const float &nPoseSimThres);
    std::vector<KeyFrame*> DetectCandidatesWithPCB_KPR(Frame* F, Map* pMap);
    
    // ! ----------------------------------- Abandoned ------------------------------
    /**
     * Future users, I abondened this part since my core research is not directly related
     * to Loop closure. Should you be able to make use of the ideas encoded in these codes, 
     * please let me know. I will be happy to integrate them into the this experiment`s repository
    */
    void DetectLoopCandidatesWithSemantics(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, Map* pMap);
    std::vector<KeyFrame*> ApplyLoopPCBConstraint(KeyFrame *pKFF, std::vector<KeyFrame*> kfAllList, const float &nPoseSimThres);
    //bool FireLoopClosure(KeyFrame *pKFOrigin, std::vector<KeyFrame*> &vpLoopCand); 
    bool FireLoopClosure(Map* pMap, std::vector<KeyFrame*> &vpLoopCand);
    // ! ----------------------------------- Abandoned ------------------------------

    //* Loop and Merge Detection using DBoW2 model [new in ORB SLAM 3]
    void DetectCandidates(KeyFrame* pKF, float minScore,vector<KeyFrame*>& vpLoopCand, vector<KeyFrame*>& vpMergeCand);
    void DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nMinWords);
    void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates);
    std::vector<KeyFrame*> DetectCandidatesWithDBoW2_KPR(Frame* F, Map* pMap);

    void PreSave();
    void PostLoad(map<long unsigned int, KeyFrame*> mpKFid);
    void SetORBVocabulary(ORBVocabulary* pORBVoc);

protected:

   // Associated vocabulary
   const ORBVocabulary* mpVoc;

   // Inverted file
   std::vector<list<KeyFrame*> > mvInvertedFile;

   // For save relation without pointer, this is necessary for save/load function
   std::vector<list<long unsigned int> > mvBackupInvertedFileId;

   // Mutex
   std::mutex mMutex;

};

} //namespace ORB_SLAM

#endif
