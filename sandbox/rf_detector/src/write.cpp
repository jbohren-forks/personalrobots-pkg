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

#include <fstream>
#include <sstream>
#include "write.h"

using namespace std;

void writeFloat2text( string filename, vector<float> & data)
{
    ofstream fout;
    fout.open(filename.c_str());
//    if (!fout)
//        return; // Failure
    int vector_size = data.size();
    for (int k = 0; k < vector_size; k++){
        fout << data[k] << " ";
    }
    fout.close();
}
void writeInt2text( string filename, vector<int> & data)
{
    ofstream fout;
    fout.open(filename.c_str());
//    if (!fout)
//        return; // Failure

    int vector_size = data.size();
    for (int k = 0; k < vector_size; k++){
        fout << data[k] << " ";
    }
    fout.close();
}
void writePoint2text( string filename, cv::Vector<cv::Point> & points)
{
    ofstream fout;
    fout.open(filename.c_str());
//    if (!fout)
//        return; // Failure

    int vector_size = points.size();
    for (int k = 0; k < vector_size; k++){
        fout << points[k].x << " " << points[k].y << endl;
    }
    fout.close();
}
void writeRect2text( string filename, cv::Vector<cv::Rect> & Rect_)
{
    ofstream fout;
    fout.open(filename.c_str());
//    if (!fout)
//        return; // Failure

    int vector_size = Rect_.size();
    for (int k = 0; k < vector_size; k++){
        fout << Rect_[k].x << " " << Rect_[k].y << " "
            << Rect_[k].width << " " << Rect_[k].height << endl;
    }
    fout.close();
}
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
bool readFloattext( string filename, vector< vector<float> > & data)
{
    ifstream fp(filename.c_str(), ios::in | ios::binary);
    if (!fp)
        return false; // Failure
//    fp.seekg(0, ios::end);
//    const streamsize size = fp.tellg();
//    fp.seekg(0);


    float tmp_float;
    fp.read((char *)&tmp_float,sizeof(float));
    float NDim = tmp_float;
    data.resize(NDim);
    int dim_count = 0;
//    for (int i=0; i< size; i++){
    while (fp.good()){
        fp.read((char *)&tmp_float,sizeof(float));
        data.at(dim_count).push_back(tmp_float);
        dim_count++;
        if (dim_count >= NDim)
            dim_count = 0;
        }
//      cout << data.at(dim_count-1).size() << data.at(dim_count).size() << endl;
        data.at(dim_count-1).pop_back();
        fp.close();
}
bool readIntegertext( string filename, vector< int > & data)
{
    ifstream fin(filename.c_str(), ios::in | ios::binary);;
    if (!fin)
        return false; // Failure
    int tmp;
    fin.read((char *)&tmp,sizeof(int));
    int NDim = tmp;
    while (fin.good()){
        fin.read((char *)&tmp,sizeof(int));
        data.push_back(tmp);
    }
    fin.close();
    // pop_back one dummy element
    data.pop_back();
}
