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
bool readFloatBinPlain( string filename, int NDim, vector< vector<float> > & data)
{
    ifstream fp(filename.c_str(), ios::in | ios::binary);
    if (!fp)
        return false; // Failure

    float tmp_float;
    data.resize(NDim);
    int dim_count = 0;
    while (fp.good()){
        fp.read((char *)&tmp_float,sizeof(float));
        data.at(dim_count).push_back(tmp_float);
        dim_count++;
        if (dim_count >= NDim)
            dim_count = 0;
    }
    // pop_back one dummy element
    data.at(dim_count-1).pop_back();
    fp.close();
}
bool readIntegerBinPlain( string filename, vector< int > & data)
{
    ifstream fin(filename.c_str(), ios::in | ios::binary);
    if (!fin)
        return false; // Failure
    int tmp;
    while (fin.good()){
        fin.read((char *)&tmp,sizeof(int));
        data.push_back(tmp);
    }
    fin.close();
    // pop_back one dummy element
    data.pop_back();
}
void readClassId2ViewObjClassMapFile( string filename, vector<string>& ObjClassStr, vector<int>& ViewMap){

    ifstream fp(filename.c_str(), ios::in);
    if (!fp){
	cout << filename.c_str() << "no such file" <<endl;
        return; // Failure
    }

    while ( fp.good()){
    	std::string line;
    	std::getline(fp, line);
    	stringstream iss(line);
    	int classId;
    	iss >> classId; 
	//cout <<"classId" << classId <<endl;
        if (classId >= ObjClassStr.size()){
		ObjClassStr.resize(classId+1);
		ViewMap.resize(classId+1);
	}
	int viewId;
    	iss >> viewId;
	//cout <<"viewId" << viewId <<endl;
	ViewMap[ classId] = viewId;	

	// get objClass string
	int objClassId;
    	iss >> objClassId;
	//cout <<"objClassId" << objClassId <<endl;
	switch ( objClassId){
	case 1 :
		ObjClassStr[classId] = "shoe";
		break;
	case 2 :
		ObjClassStr[classId] = "cellphone";
		break;
	case 3 :
		ObjClassStr[classId] = "iron";
		break;
	case 4 :
		ObjClassStr[classId] = "mouse";
		break;
	case 5 :
		ObjClassStr[classId] = "stapler";
		break;
	case 6 :
		ObjClassStr[classId] = "toaster";
		break;
	case 7 :
		ObjClassStr[classId] = "car";
		break;
	case 8 :
		ObjClassStr[classId] = "bicycle";
		break;
	case 9 :
		ObjClassStr[classId] = "mug";
		break;
	}
	//cout << ObjClassStr[classId] <<endl;
    }
    //cout << "out of readClassId2ViewObjClassMapFile" <<endl;
	
}
