/*********************************************************************
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author: Min Sun
#include"color_match.h"

using namespace std;

bool read2DFloatText( string filename, vector< vector<float> > & data)
{
    ifstream fp(filename.c_str(), ios::in);
    if (!fp)
        return false; // Failure

    std::string line;
    vector<float> tmp_float;
    // For each line...
    while (std::getline(fp, line)) {
        stringstream iss(line);
        tmp_float.clear();
        float ff;

        // For each character token...
        while (iss >> ff) {
            // Do something with c
            tmp_float.push_back( ff);
        }
        data.push_back( tmp_float);
    }
    fp.close();
    return true;
}

float DistGRB(float R, float G, float B, vector<float> exemplar){
   return (R-exemplar[0])*(R-exemplar[0])+(G-exemplar[1])*(G-exemplar[1])+(B-exemplar[2])*(B-exemplar[2]);
}

void color_match_neighbor( vector<vector<float> > exemplars, IplImage* rcalimage, vector<robot_msgs::Point> candidates, vector< int> cand_ids, double threshold, int radius){

    for (unsigned int cand_id=0; cand_id < candidates.size(); cand_id++){
        float min_dist =0.;
        unsigned int best_cand = 0;
        
        for (unsigned int y= std::max((int)candidates[cand_id].y-radius,0);
            y< std::min((int)candidates[cand_id].y+radius, rcalimage->height); y++){
            float* ptr =  (float*)( rcalimage->imageData + y*rcalimage->widthStep);
            for (unsigned int x= std::max( (int)candidates[cand_id].x-radius,0);
                x< std::min( (int)candidates[cand_id].x+radius, rcalimage->width); x++){
                for (unsigned int exe_id=0; exe_id < exemplars.size(); exe_id++){
                    float dist = DistGRB( ptr[3*x], ptr[3*x+1], ptr[3*x+2], exemplars[exe_id]); 
                    if (dist< min_dist)
                        best_cand = exe_id;
                }
            }
        }

        if (min_dist < (float)(threshold*threshold)){
            cand_ids.push_back( best_cand);
        }else{
            cand_ids.push_back( -1);
        }
        
    }

}

