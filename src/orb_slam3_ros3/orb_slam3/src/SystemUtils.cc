/**
 * @file SystemUtils.h
 * @brief Various utility methods written to debug various aspects of ORB-SLAM3
 * @author Azmyin Md. Kamal
 * @date 01/12/22 - 01/12/2025
 */
# include "SystemUtils.h"

namespace SystemUtils
{
    //* Deubg tools

    void printTwoBlankLines(){
        std::cout << "\n\n" << std::endl;
    }
    
    void printBoolean(std::string msg, bool val){
        std::cout << std::boolalpha;
        std::cout << msg << val <<std::endl;
    }

    void printStrMsgOneBlnk (std::string msg){
        std::cout << msg << std::endl;
    }

    void printEigenRowColNum (const int &row, const int &col){
        std::cout << "Row count: " << row << " Col count: " << col << "\n" <<std::endl;
    }

    void printFrameId(const int &val){
        std::cout << "Frame (or Keyframe) ID: " << val << "\n" <<std::endl;
    }
    
    std::vector<float> eigenFloatColtoStdVec(const eigenMatXf &mat, const int &col){
        std::vector<float> column1(mat.rows()); // Initialize, array with 0 elements
        
        //Watchdog: If mat points to a null matrix, then this column1 vector will have a size 0 
        if (column1.size() != 0)
        {
            for (int r =0; r< mat.rows(); r++)
            {
                column1.at(r) = mat(r, col);
            }
        }
        
        return column1; // Either a zero vector or a vector detected object ids
    
    }

    void printStdVecFloat(std::vector<float> &vec1){
        // Source: https://stackoverflow.com/questions/10750057/how-do-i-print-out-the-contents-of-a-vector?page=1&tab=modifieddesc#tab-top
        for (const float& i: vec1) std::cout << i << ' ';
        std::cout<<std::endl;
    }

    // Overloaded version
    void printStdVecFloat(const std::vector<float> &vec1){
        // Source: https://stackoverflow.com/questions/10750057/how-do-i-print-out-the-contents-of-a-vector?page=1&tab=modifieddesc#tab-top
        for (const float& i: vec1) std::cout << i << ' ';
        std::cout<<std::endl;
    }

    void printStdVecDouble(const std::vector<double> &vec1){
        // Source: https://stackoverflow.com/questions/10750057/how-do-i-print-out-the-contents-of-a-vector?page=1&tab=modifieddesc#tab-top
        for (const double& i: vec1) std::cout << i << ' ';
        std::cout<<std::endl;
    }

    void printStdVecInt(const std::vector<int> &vec1){
        // Source: https://stackoverflow.com/questions/10750057/how-do-i-print-out-the-contents-of-a-vector?page=1&tab=modifieddesc#tab-top
        for (const int& i: vec1) std::cout << i << ' ';
        std::cout<<std::endl;
    }

    void printStdVecInt(std::vector<int> &vec1){
        // Source: https://stackoverflow.com/questions/10750057/how-do-i-print-out-the-contents-of-a-vector?page=1&tab=modifieddesc#tab-top
        // Non-constant version
        for (const int& i: vec1) std::cout << i << ' ';
        std::cout<<std::endl;
    }

    void printEigenQuaternionFloat(Eigen::Quaternionf &q){
        //* Eigen::Quaterion<type, option> expects arguments in form (w,x,y,z). So print in the same order
        std::cout<< "q.w -- " << q.w() << " q.x -- " << q.x() << " q.y -- " << q.y() << " q.z -- " << q.z() << std::endl;
    }

    void printStdVecFloatSize(std::vector<float> &vec1){
        std::cout << "Number of elements in this std::vector<float>: " << vec1.size() << std::endl;
    }

    void printKeyFrameConditionStatus(const bool &c1a, const bool &c1b, const bool &c1c, const bool &c2, const bool &c3, const bool &c4, const bool &c5){
        std::cout <<"\n";
        std::cout << "Need new keyframe conditions\n";
        std::cout << "Condition 1a: " << c1a << "\n";
        std::cout << "Condition 1b: " << c1b << "\n";
        std::cout << "Condition 1c: " << c1c << "\n";
        std::cout << "Condition 2: " << c2 << "\n";
        std::cout << "Condition 3: " << c3 << "\n";
        std::cout << "Condition 4: " << c4 << "\n";
        std::cout << "Condition 5 (our contrib): " << c5 << "\n" <<std::endl;
    }

    // Helper function that uses C++ iterator to print out elements of a vector in vectors object
    void printStdVecInVecsFloatMemEff(const stdVecInVecsFloat &vec){
        // Author: Mason Passon
        // jdx is a pointer to a pointer
        std::cout <<vec.size()<<" rows"<<"\n";
        for (auto idx = vec.begin(); idx < vec.end(); idx++) {
            std::cout << "Row: ";
            for (auto jdx = idx->begin(); jdx < idx->end(); jdx++) {
                std::cout << (*jdx) << " ";
            }
            std::cout << "\n";
        }
    }

    void printStdVecInVecsFloat(const stdVecInVecsFloat &vec1){
        // Source: https://stackoverflow.com/questions/10750057/how-do-i-print-out-the-contents-of-a-vector?page=1&tab=modifieddesc#tab-top
        // Author: Azmyin
        // Uses some memory but maybe not very memory intensive
        int row_cnt = 0;
        for (const std::vector<float>& i: vec1)
        {
            std::cout <<"Row: "<<row_cnt<< ' ';
            for (const float& j: i) std::cout << j << ' ';
            std::cout<<"\n"; 
            row_cnt++;
        } 
    }
    
    void printSE3fMatrix(sophusSE3f &mat){
        // Prints out the contents of a 4x4 Sophus::SE3f matrix
        //std::cout<<"\nContents of this SE3f: \n";
        std::cout<<mat.matrix()<<std::endl;
    }

    void printpsdKfObjInStdVec(std::vector<ORB_SLAM3::KeyFrame*> &kfList){
        // Prints out the SSA of Keyframes pulled inside a std::vector
        for (ORB_SLAM3::KeyFrame* ptrKF : kfList)
        {
            ptrKF->PrintDebugPSD();
            std::cout<<std::endl;
        }
    }
    
    void printFrameID(long unsigned int *fID){
        std::cout<<"Frame ID: "<<(*fID)<<std::endl;
    }

    void printKFIDsInStdVec(std::vector<ORB_SLAM3::KeyFrame*> &vpKFs){
        // Prints KeyFrame ID in the std::vector<KeyFrame*> container
        if (vpKFs.size()!= 0){
            for (ORB_SLAM3::KeyFrame* ptrTest : vpKFs){
                std::cout<<"KF ID: "<<ptrTest->mnId<<std::endl;
            }
        }
        else{
            std::cout<<"std::vector<float> container empty"<<std::endl;
        }
        
    }

    // Print some comparator value between two keyframes
    void printFloatComparatorValueBtnTwoKFs(ORB_SLAM3::KeyFrame* pKFF, ORB_SLAM3::KeyFrame* ptrKF, float &value){
        std::cout<<"For KF pairs " << "("<<pKFF->psdKfObj.kfID<<","<<ptrKF->psdKfObj.kfID<<"), Value:  "<<value<<"\n";

    }

    void printDoubleTimeValueWithMsg(std::string msg, double &val){
        std::cout<<msg<<" "<<val<<" ms\n"<<std::endl;
    }

    void printDatabase(std::unordered_map<float, std::string> &DB, std::string db_name){
        std::cout<<"Database name: "<<db_name<<"\n";
        for (auto x : DB){
            std::cout<<"Class ID: " << x.first << " Class Name: " << x.second << std::endl;
        }     
    }

    void extractClassIdsInStdVecFloat(std::unordered_map<float, std::string> &STAT_DB,std::unordered_map<float, std::string> &DYNA_DB,std::vector<float> &static_obj_db, std::vector<float> &dynamic_obj_db){
        // Reserve sizes
        static_obj_db.reserve(STAT_DB.size());
        dynamic_obj_db.reserve(DYNA_DB.size());
        
        // Populate the static_obj_db
        for (auto x: STAT_DB){
            static_obj_db.push_back(x.first);
        }
        // Populate the dynamic_obj_db
        for (auto y: DYNA_DB){
            dynamic_obj_db.push_back(y.first);
        }
        // Make sure they are in order
        std::sort(static_obj_db.begin(), static_obj_db.end());
        std::sort(dynamic_obj_db.begin(), dynamic_obj_db.end());
    }

    //* Helper functions
    // TODO copy the one liner description above each of the function

    std::vector<float> filterFromStdVecFloat(const std::vector<float> &vec1, const std::vector<float> &vec2)
    {
        /*
            Parameters:
            Inputs:
                vars -- Type, Size, Description
                vec1 -- Input
                vec2 -- Comparator
        
            Outputs:
                vars -- Type, Size, Description
                vec3 -- List of elements that found in vec1 that matches with the elements in vec2
        
            Notes:
                * Bullet point 1
                * Bullet point 2
        */
       
       std::vector<float> vec3; // Return list

       // Loop through elements in vector vec1
       //* vec1 is a proxy for ls_obj list???
       //* Introduced in C++ 11 very useful way of iterating through a vector object
       for (float key : vec1) 
       {
            // Check 'key' matches with an element in the vec2 std::vector object
            if (std::find(vec2.begin(), vec2.end(), key) != vec2.end()) 
            {
                // Match found, push element
                vec3.push_back(key);

            }
            else 
            {
                pass; // (void)0, Python's "pass" equivalent
        
            }
            
       } // EOF for loop
    
        return vec3;
    }

    eigenMat4f sophusToEigenMat4f(sophusSE3f mat){
        eigenMat4f Tout(4,4);
        Tout = mat.matrix(); // Sophus to Eigen matrix conversion
        return Tout;
    }

    stdVecInVecsFloat extractBBoxCordsForKnownList(const eigenMatXf &semMat, const std::vector<float> &vec2){
        //* Function that creates a 2D vector in vectors to contain bounding box coordinates of certain type {static, dynamic}
        //* Expects a known list of objects {i.e static or dynamic}
        //* Input eigen matrix is the full semantic matrix
        
        // Initialize output variable
        stdVecInVecsFloat vec1;
        // TODO figure out a way to avoid creating this matrix
        const eigenMatXf eigenBBoxMat = semMat.rightCols(4); // Row major

        // Create a new matrix which contains all rows of sem_mat except the first row 
        // i.e all bounding box coordinates
        
        //Cycle through each row of Eigen semMat matrix
        for (int r = 0; r < semMat.rows(); r++){
            
            // Take first element of this vector as input key
            float in_key = semMat.coeff(r, 0); // [cls_id, x1, y1, x2, y2]

            //std::cout << "in_key before test: "<< in_key <<" "<<std::endl;

            if (std::find(vec2.begin(), vec2.end(), in_key) != vec2.end()) // Look through each of the elements in vec2 list 
                {
                    // Match found, push row into vector in vectors variable
                    // TODO find a way to access elements directly without the need to create eigenBBoxMat variable
                    const std::vector<float> row1_vec(eigenBBoxMat.row(r).data(), eigenBBoxMat.row(r).data() + eigenBBoxMat.cols());
                    
                    // std::cout << "in_key after test: "<< in_key <<" ";
                    // printStdVecFloat(row1_vec);
                    // std::cout << "\n";

                    vec1.push_back(row1_vec);

                }
            else 
                {
                    pass; // (void)0, Python's "pass" equivalent
                }
        }
        return vec1;
    }

    std::vector<sophusSE3f> reverseStdVecSophusMats(std::vector<sophusSE3f> &vec_in){
        // Function that reverses the order of frame poses in the vector
        std::vector<sophusSE3f> vec_out; // Output vector
        vec_out.reserve(vec_in.size()); // Allocate enough memory

        for (int i = vec_in.size() - 1; i >= 0; i--){
            vec_out.push_back(vec_in[i]);
        }

        return vec_out;
    }

    //* Calculate IoU score for one pair of bounding boxes
    float intersectionOverUnion(std::vector<float> &box1, std::vector<float> &box2){
        // Adapted from https://github.com/lukaswals/cpp-iout/blob/master/cppIOUT/IOUT.cpp
        
        //* In our version we have the bbox as [x1,y1,x2,y2] -- idx[0,1,2,3]
        // float minx1 = box1.x;
        // float maxx1 = box1.x + box1.w;
        // float miny1 = box1.y;
        // float maxy1 = box1.y+ box1.h;

        // float minx2 = box2.x;
        // float maxx2 = box2.x + box2.w;
        // float miny2 = box2.y;
        // float maxy2 = box2.y + box2.h;

        // Math works fine
        float minx1 = box1.at(0);
        float maxx1 = minx1 + (box1.at(2) - box1.at(0));
        float miny1 = box1.at(1);
        float maxy1 = miny1+ (box1.at(3) - box1.at(1));

        float minx2 = box2.at(0);
        float maxx2 = minx2 + (box2.at(2) - box2.at(0));
        float miny2 = box2.at(1);
        float maxy2 = miny2 + (box2.at(3) - box2.at(1));

        
        //* My version but does not seem to work
        // float minx1 = box1.at(0); // x1
        // float maxx1 = box1.at(2); // x2
        // float miny1 = box1.at(1); // y1
        // float maxy1 = box1.at(3); // y2

        // float minx2 = box2.at(0); // x1
        // float maxx2 = box2.at(2); // x2
        // float miny2 = box2.at(1); // y1
        // float maxy2 = box2.at(3); // y2

        if (minx1 > maxx2 || maxx1 < minx2 || miny1 > maxy2 || maxy1 < miny2)
            return 0.0f;
        else
        {
            float dx = std::min(maxx2, maxx1) - std::max(minx2, minx1);
            float dy = std::min(maxy2, maxy1) - std::max(miny2, miny1);
            float area1 = (maxx1 - minx1)*(maxy1 - miny1);
            float area2 = (maxx2 - minx2)*(maxy2 - miny2);
            float inter = dx*dy; // Intersection
            float uni = area1 + area2 - inter; // Union
            float IoU = inter / uni;
            return IoU;
        }
    //	return 0.0f
        
    }

    // Function that returns how many bboxes in query frame is >90% area match with bboxes in the candidate frame
    // * Original, fixed threshold
    int boxMatchedBtwnQueryCandidate(stdVecInVecsFloat &query_boxes, stdVecInVecsFloat &candidate_boxes){
        // stdVecInVecsFloat -- std::vector<std::vector<float>>
        int nmatchedBoxes = 0;
        int objsInQueryKf = query_boxes.size(); // Number of boxes (objects) in the container
        float matchThres = 0.9; //* Parameter

        //* Sequentially go through each of the bounding boxes in query frame
        // ! This code really needs a lot of optimization and parallel processing
        for (std::vector<float> &box1: query_boxes)
        {
            //* Cycle through each box in candidate_boxes and compute their IoU score
            //* If a query-candidate box pairs have IoU score greater than matchThres, increment a match counter
            //std::cout<<"Object no: "<<cnt<<"\n";
            for (std::vector<float> &box2: candidate_boxes)
            {
                float IoU = intersectionOverUnion(box1, box2);
                if (IoU >= matchThres) {
                    nmatchedBoxes = nmatchedBoxes + 1;
                }
                // else ignore
            }
        } 

        return nmatchedBoxes;
    }
    
    int boxMatchedBtwnQueryCandidate(stdVecInVecsFloat &query_boxes, stdVecInVecsFloat &candidate_boxes, float matchThres){
        // Overloaded version
        // stdVecInVecsFloat -- std::vector<std::vector<float>>
        int nmatchedBoxes = 0;
        int objsInQueryKf = query_boxes.size(); // Number of boxes (objects) in the container
        // float matchThres = 0.9; //* Parameter

        //* Sequentially go through each of the bounding boxes in query frame
        // ! This code really needs a lot of optimization and parallel processing
        for (std::vector<float> &box1: query_boxes)
        {
            //* Cycle through each box in candidate_boxes and compute their IoU score
            //* If a query-candidate box pairs have IoU score greater than matchThres, increment a match counter
            //std::cout<<"Object no: "<<cnt<<"\n";
            for (std::vector<float> &box2: candidate_boxes)
            {
                float IoU = intersectionOverUnion(box1, box2);
                //std::cout<<"IoU score: "<< IoU <<std::endl;
                if (IoU >= matchThres) {
                    nmatchedBoxes = nmatchedBoxes + 1;
                }
                else{
                    continue;
                }
            }
        } 

        return nmatchedBoxes;
    }

    // Generate a list of Frame ids to simulate relocalization problem
    std::vector<int> ChooseFramesToInduceRelocalization(int nToChoose, int nLowerBound, int nUpperBound){
        
        /*
            Pseudocode
            * Randomly choose N frames to induce relocalization
            * Input upper bound, starting lower bound and number of samples to draw
            * After choice is done, increase lower bound by 20 frames
            * Add guard, if lower bound crosses upper bound return choices
            * If number of choices found, return choices
        */

        // Code adopted from https://stackoverflow.com/questions/5008804/generating-a-random-integer-from-a-range
        //! TODO make a small algorithm showing what is happening here

        // Initialize
        std::vector<int> mvFrameList;
        mvFrameList.reserve(nToChoose);
        std::random_device rd;  // Only used once to initialise (seed) engine
        std::mt19937 rng(rd()); // Random-number engine used (Mersenne-Twister in this case)
        int cnt = 0;
        int nNumMoveForward = 60; // After choosing 1 frame, wwait 60 frames before choosing another relocalization

        for (int i =0; i<nToChoose; i++){
            
            // Create a uniform integer distribution
            std::uniform_int_distribution<int> uni(nLowerBound, nUpperBound); // Guaranteed unbiased

            auto nFId = uni(rng);
            // std::cout<<"nFId: "<<nFId<<std::endl;
            mvFrameList.push_back(nFId);
            nLowerBound = nLowerBound + nNumMoveForward; // 20 frames is the default windows by which the system needs to reinitialize
            //nLowerBound = nFId + nNumMoveForward; // 20 frames is the default windows by which the system needs to reinitialize

            if (nLowerBound > nUpperBound){
                break; // Return the frames that were found
            }

            // std::cout<<"Lower bound: "<<nLowerBound<<std::endl;
            // std::cout<<"Upper bound: "<<nUpperBound<<std::endl;
            // std::cout<<"\n";
            
        }

        //Sort in ascending order, default behavior of std::sort()
        std::sort(mvFrameList.begin(), mvFrameList.end());
        
        return mvFrameList;
    }

    std::string findClassNameFromBD(float &cls_id, std::unordered_map<float, std::string> &pSTAT_DB, std::unordered_map<float, std::string> &pDYNA_DB){
        std::string class_name = ""; // Initialize
        
        // Search the static database first
        for (auto& itr:pSTAT_DB){
            if (itr.first == cls_id){
                class_name = itr.second;
                return class_name;
            }
        }

        // Search through the dynamic object database
        for (auto& itr:pDYNA_DB){
            if (itr.first == cls_id){
                class_name = itr.second;
                return class_name;
            }
        }
        
        return class_name; // To prevent pid -11 fault
    }

    // Function that checks if an integer is present in a std::vector<int> container
    bool inStdIntVec(int item,std::vector<int> &vec){
        if (std::find(vec.begin(), vec.end(), item) != vec.end())
            return true;
        else
            return false;
    }

    std::vector<std::string> splitString(const std::string str, char splitter){
        // source: https://stackoverflow.com/questions/68396962/how-to-split-strings-in-c-like-in-python
        std::vector<std::string> result;
        std::string current = ""; 
        for(int i = 0; i < str.size(); i++){
            if(str[i] == splitter){
                if(current != ""){
                    result.push_back(current);
                    current = "";
                } 
                continue;
            }
            current += str[i];
        }
        if(current.size() != 0)
            result.push_back(current);
        return result;
    }

    //* Linear Algebra tools
    
    // Calculate Frobenius norm using singular values from a SVD decomposition
    float calcFrobeniusNorm(eigenMatXf &mat)
    {
        // * Method 1: SVD version may use a lot more computation time than necessary
        // Compute Frobenius Norm of a given eigenMatrix
        eigenJacobiMatXf kfSVD(mat); // SVD decomposition
        //std::cout << "Its singular values are:" << std::endl << kfSVD.singularValues() << std::endl;

        const Eigen::VectorXf &S = kfSVD.singularValues(); //  All singular values are const type
        float frob_norm = sqrt(S.cwiseProduct(S).sum()); // sqrt( (S.array() * S.array()).sum())
        //* https://stackoverflow.com/questions/34373757/piece-wise-square-of-vector-piece-wise-product-of-two-vectors-in-c-eigen

        return frob_norm;
    }
    
    float calcFrobeniusNorm(eigenMatXf &mat1, eigenMatXf &mat2, double &nTime){
        std::chrono::steady_clock::time_point time_Start = std::chrono::steady_clock::now();
        eigenJacobiMatXf kfSVD(mat1 - mat2); // ~ 1 flop operation
        const Eigen::VectorXf &S = kfSVD.singularValues(); //  All singular values are const type
        float frob_norm = sqrt(S.cwiseProduct(S).sum()); // sqrt( (S.array() * S.array()).sum())
        
        nTime = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::steady_clock::now() - time_Start).count();
        
        //printDoubleTimeValueWithMsg("calcFrobeniusNorm took:", timeDiff);

        return frob_norm;
    }

    // David, S Fundamentals of Matrix Computations,7E, pg 115
    float calcFrobeniusNormDirect(eigenMatXf &mat){
        return sqrt(mat.cwiseProduct(mat).sum());  
    }

    float calcFrobeniusNormDirect(eigenMatXf &mat1, eigenMatXf &mat2, double &nTime){
        std::chrono::steady_clock::time_point time_Start = std::chrono::steady_clock::now();
        eigenMatXf mat = mat1 - mat2;
        float frob_norm = sqrt(mat.cwiseProduct(mat).sum());
        nTime = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(std::chrono::steady_clock::now() - time_Start).count();
        return frob_norm; // David, S Fundamentals of Matrix Computations,7E, pg 115 
    }

    // Version with no time keeper codes
    float calcFrobeniusNormDirect(eigenMatXf &mat1, eigenMatXf &mat2){
        eigenMatXf mat = mat1 - mat2;
        float frob_norm = sqrt(mat.cwiseProduct(mat).sum());
        return frob_norm; // David, S Fundamentals of Matrix Computations,7E, pg 115 
    }

    float calcL2NormFromStdVecFloat(std::vector<float> &vec){
        // Eigen::Map always creates a new memory hence vec is safe
        return eigenMapToEigenVecFloat(vec.data(), vec.size()).norm();
    }

    //* Statistics Tools
    // Calculate the average of a std::vector<double> container
    double calcAverage(std::vector<double> v_times)
    {
        
        // double can store floating point values twice the size of floats
        double accum = 0;
        for(double value : v_times)
        {
            accum += value;
        }
        //std::cout<<"accum: "<<accum<<" num: "<<v_times.size()<<"\n";
        return accum / v_times.size();
    }

    // Calculate deviation for a std::vector<double> container
    double calcDeviation(std::vector<double> v_times, double average)
    {
        double accum = 0;
        for(double value : v_times)
        {
            accum += pow(value - average, 2);
        }
        return sqrt(accum / v_times.size());
    }

    // Overloaded version uses a std::vector<int> container
    double calcAverage(std::vector<int> v_values)
    {
        double accum = 0;
        int total = 0;
        for(double value : v_values)
        {
            if(value == 0)
                continue;
            accum += value;
            total++;
        }

        return accum / total;
    }

    // Calculate deviation for a std::vector<int> container with a double average
    double calcDeviation(std::vector<int> v_values, double average)
    {
        double accum = 0;
        int total = 0;
        for(double value : v_values)
        {
            if(value == 0)
                continue;
            accum += pow(value - average, 2);
            total++;
        }
        return sqrt(accum / total);
    }


}//End SystemUtils namespace
