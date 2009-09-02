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
//
#include <iostream>
#include <math.h>
#include <algorithm>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "cvaux.h"
#include "point_cloud_mapping/cloud_io.h"
#include "t2obj.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"

using namespace cv;
using namespace std;

inline int LoadPcdAndPrune( string pcdfilename, sensor_msgs::PointCloud& pcd_pc){
    sensor_msgs::PointCloud pcd_pc_tmp;
    int load_pc_flag = cloud_io::loadPCDFile( pcdfilename.c_str(), pcd_pc_tmp);
    if (load_pc_flag == -1){
        cout << "error when loading pcd file" <<endl;
        return -1;
    }

    // prune out all zero points
    unsigned int count = 0;
    pcd_pc.header = pcd_pc_tmp.header;
    pcd_pc.channels.resize(pcd_pc_tmp.channels.size());
    for (unsigned int q =0; q< pcd_pc_tmp.channels.size(); q++){
        pcd_pc.channels[q].name = pcd_pc_tmp.channels[q].name;
    }
    for (unsigned int k=0;k<pcd_pc_tmp.points.size();k++){
         if (pcd_pc_tmp.points[k].x!=0 || pcd_pc_tmp.points[k].y!=0 ||
                 pcd_pc_tmp.points[k].z!=0){
                pcd_pc.points.push_back(pcd_pc_tmp.points[k]);
            for (unsigned int q =0; q< pcd_pc_tmp.channels.size(); q++){
                pcd_pc.channels[q].values.push_back( pcd_pc_tmp.channels[q].values[k]);
            }
                count ++;
         }
    }

    // debug information
    cout <<" count" << count << endl;
    cout << "num of points" << pcd_pc.points.size() <<endl;
    cout << "num of channels" << pcd_pc.channels.size() <<endl;
    for (unsigned int q =0; q< pcd_pc.channels.size(); q++){
        cout << pcd_pc.channels[q].name;
        cout << pcd_pc.channels[q].values.size()<<endl;
    }

    return 1;
}

inline int LoadPcd2CvMat( string pcdfilename, Size ImgSize, CvMat* pcd_pc){

    sensor_msgs::PointCloud pcd_pc_tmp;
    int load_pc_flag = cloud_io::loadPCDFile( pcdfilename.c_str(), pcd_pc_tmp);
    if (load_pc_flag == 0){
        return -1;
    }

    // Debug 
    cout << "num of points" << pcd_pc_tmp.points.size() <<endl;
    cout << "num of channels" << pcd_pc_tmp.channels.size() <<endl;
    for (unsigned int q =0; q< pcd_pc_tmp.channels.size(); q++){
        cout << pcd_pc_tmp.channels[q].name;
        cout << pcd_pc_tmp.channels[q].values.size()<<endl;
    }

    // construct pcd_pc as CvMat
    cout << "ImgHeight" << ImgSize.height << "ImgWidth"<< ImgSize.width<<endl;
    float* ptr;
    unsigned int count = 0;
    unsigned int count_valid_points = 0;
    pcd_pc = cvCreateMat( ImgSize.height, ImgSize.width, CV_32FC4);
    unsigned int nChannels = 4;
    for ( int y=0; y< ImgSize.height; y++){
        for ( int x=0; x< ImgSize.width; x++){
            ptr = (float*) ( pcd_pc->data.ptr + y*pcd_pc->step);
            ptr[x*nChannels] = pcd_pc_tmp.points[count].x;
            ptr[x*nChannels+1] = pcd_pc_tmp.points[count].y;
            ptr[x*nChannels+2] = pcd_pc_tmp.points[count].z;
            if (pcd_pc_tmp.points[count].x ==0 &&
                pcd_pc_tmp.points[count].y ==0 &&
                pcd_pc_tmp.points[count].z == 0){
                ptr[x*nChannels+3] = 0;
            }else{
                ptr[x*nChannels+3] = 1.;//pcd_pc_tmp.channels[0].values[count];
                count_valid_points++;
            }
            count ++;
        }
    }

    //Debug
    cout << "count" << count <<endl;
    cout <<"count_valid_points" << count_valid_points <<endl;

    return 1;

}

inline unsigned int dist2d( unsigned int x, unsigned int y){
	return x*x+y*y;
}

// this need to use t2obj lib to get pc to image pixel transformation
inline void Sample_interest_point( sensor_msgs::PointCloud& object_pc, sensor_msgs::PointCloud& plane_pc,
    Vector<geometry_msgs::Point>& object_pp, Vector<geometry_msgs::Point>& plane_pp,
    Vector< geometry_msgs::Point32>& interest_pts, 
    vector< cv::Point>& image_patch_centers,
    const sensor_msgs::CameraInfo& cinfo, tf::TransformListener& tf,
    cv::Size SampleSteps, cv::Size ImgSize){

    // sample grid parameters
    cv::Size gridSize( ImgSize.width/SampleSteps.width, ImgSize.height/SampleSteps.height);

    // construct a valid map of pp
    CvMat * valid_pp;
    unsigned int chandim = 3;
    unsigned int neighbor_grid = 0;
    unsigned int neighbor_grid_object = 5;
    valid_pp = cvCreateMat( gridSize.height, gridSize.width, CV_16UC3);
	// three chan: interest_pts_ind, image_x, image_y
    cvZero(valid_pp);
    unsigned short int* uchar_ptr;
    unsigned short int* uchar_ptr_tmp;
    unsigned int gridx_ori;
    unsigned int gridy_ori;
    for (unsigned int q = 0; q < object_pp.size(); q++){
	gridx_ori = object_pp[q].x/SampleSteps.width;
	gridy_ori = object_pp[q].y/SampleSteps.height;
	for ( int gridx = std::max((int)(gridx_ori-neighbor_grid),0);		 
		gridx < std::min( (int)(gridx_ori+neighbor_grid+1),gridSize.width); gridx++){
		for ( int gridy = std::max((int)(gridy_ori-neighbor_grid),0); 
			gridy < std::min((int)(gridy_ori+neighbor_grid+1),gridSize.height); gridy++){
			uchar_ptr = (unsigned short int*) (valid_pp->data.ptr + gridy*valid_pp->step);
			if (uchar_ptr[ gridx*chandim] ==0){
				uchar_ptr[ gridx*chandim] = q;
				uchar_ptr[ gridx*chandim+1] = 
					dist2d( gridx*SampleSteps.width-object_pp[q].x, gridy*SampleSteps.height-object_pp[q].y);
				uchar_ptr[ gridx*chandim+2] = 1;
			}else if( dist2d( object_pp[q].x-gridx*SampleSteps.width, object_pp[q].y-gridy*SampleSteps.height)
				  < uchar_ptr[ gridx*chandim+1] ){
				uchar_ptr[ gridx*chandim] = q;
				uchar_ptr[ gridx*chandim+1] = 
					dist2d( gridx*SampleSteps.width-object_pp[q].x, gridy*SampleSteps.height-object_pp[q].y);	
				uchar_ptr[ gridx*chandim+2] = 1;
			}
		}
	}
    }
    for (unsigned int q = 0; q < plane_pp.size(); q++){
	gridx_ori = plane_pp[q].x/SampleSteps.width;
	gridy_ori = plane_pp[q].y/SampleSteps.height;
	for ( int gridx = std::max((int)(gridx_ori-neighbor_grid),0);		 
		gridx < std::min( (int)(gridx_ori+neighbor_grid+1),gridSize.width); gridx++){
		for ( int gridy = std::max((int)(gridy_ori-neighbor_grid),0); 
			gridy < std::min((int)(gridy_ori+neighbor_grid+1),gridSize.height); gridy++){
			uchar_ptr = (unsigned short int*) (valid_pp->data.ptr + gridy*valid_pp->step);
			if (uchar_ptr[ gridx*chandim] ==0){
				uchar_ptr[ gridx*chandim] = q;
				uchar_ptr[ gridx*chandim+1] = 
					dist2d( gridx*SampleSteps.width-plane_pp[q].x, gridy*SampleSteps.height-plane_pp[q].y);
				uchar_ptr[ gridx*chandim+2] = 2;
			}else if( dist2d( plane_pp[q].x-gridx*SampleSteps.width, plane_pp[q].y-gridy*SampleSteps.height)
				  < uchar_ptr[ gridx*chandim+1] ){
				uchar_ptr[ gridx*chandim] = q;
				uchar_ptr[ gridx*chandim+1] = 
					dist2d( gridx*SampleSteps.width-plane_pp[q].x, gridy*SampleSteps.height-plane_pp[q].y);	
				uchar_ptr[ gridx*chandim+2] = 2;
			}
		}
	}
    }

    // Sample interest_pts
    geometry_msgs::Point32 point32_tmp;
    bool nei_valid_pp_object_flag;
    for ( int y=0, gridy=0; y< ImgSize.height; gridy++, y+=SampleSteps.height){
        uchar_ptr = (unsigned short int*) (valid_pp->data.ptr + gridy*valid_pp->step);
        for ( int x=0, gridx=0; x< ImgSize.width; gridx++, x+=SampleSteps.width){
	    //test if neighbor valid_pp is from object
	    nei_valid_pp_object_flag = false;
            if ( uchar_ptr[chandim*gridx] != 0){
		for ( int gridy_nei = std::max((int)(gridy-neighbor_grid_object),0); 
			gridy_nei < std::min((int)(gridy+neighbor_grid_object+1),gridSize.height); gridy_nei++){
			uchar_ptr_tmp = (unsigned short int*) (valid_pp->data.ptr + gridy_nei*valid_pp->step);
			for ( int gridx_nei = std::max((int)(gridx-neighbor_grid_object),0);		 
				gridx_nei < std::min( (int)(gridx+neighbor_grid_object+1),gridSize.width); gridx_nei++){
				if (uchar_ptr_tmp[chandim*gridx_nei+2] == 1){
					nei_valid_pp_object_flag = true;
					break;
				}
			}
			if (nei_valid_pp_object_flag)
			break;
		}
	    }
            if ( nei_valid_pp_object_flag){
		if (uchar_ptr[chandim*gridx+2] == 1){
                	point32_tmp.x = object_pc.points[ uchar_ptr[chandim*gridx]].x;
                	point32_tmp.y = object_pc.points[ uchar_ptr[chandim*gridx]].y;
                	point32_tmp.z = object_pc.points[ uchar_ptr[chandim*gridx]].z;
		}else if (uchar_ptr[chandim*gridx+2] == 2){
                	point32_tmp.x = plane_pc.points[ uchar_ptr[chandim*gridx]].x;
                	point32_tmp.y = plane_pc.points[ uchar_ptr[chandim*gridx]].y;
                	point32_tmp.z = plane_pc.points[ uchar_ptr[chandim*gridx]].z;
		}
                interest_pts.push_back(point32_tmp);
		cv::Point tmp_point;
		tmp_point.x = x;
		tmp_point.y = y;
                image_patch_centers.push_back( tmp_point);
            }
        }
    }
}

inline unsigned int GetSingleROI( geometry_msgs::Point32 pp, float Radius, 
        unsigned int winStep, const sensor_msgs::CameraInfo& cinfo){

    geometry_msgs::Point32 pp_l;
    pp_l.x = pp.x-Radius;
    pp_l.y = pp.y;
    pp_l.z = pp.z;

    geometry_msgs::Point pp_tmp; // projected point
    pp_tmp.x = cinfo.P[0]*pp_l.x+
		cinfo.P[1]*pp_l.y+
		cinfo.P[2]*pp_l.z+
		cinfo.P[3];
    pp_tmp.z = cinfo.P[8]*pp_l.x+
		cinfo.P[9]*pp_l.y+
		cinfo.P[10]*pp_l.z+
		cinfo.P[11];
    pp_tmp.x /= pp_tmp.z;
    float u_l = pp_tmp.x;
	//cout <<"u_l"<< u_l <<endl;
	
    geometry_msgs::Point32 pp_r;
    pp_r.x = pp.x+Radius;
    pp_r.y = pp.y;
    pp_r.z = pp.z;
    pp_tmp.x = cinfo.P[0]*pp_r.x+
		cinfo.P[1]*pp_r.y+
		cinfo.P[2]*pp_r.z+
		cinfo.P[3];
    pp_tmp.z = cinfo.P[8]*pp_r.x+
		cinfo.P[9]*pp_r.y+
		cinfo.P[10]*pp_r.z+
		cinfo.P[11];
    pp_tmp.x /= pp_tmp.z;
    float u_r = pp_tmp.x;
	//cout <<"u_r"<< u_r <<endl;

    // descritize it
    unsigned int ind = (unsigned int) ((u_r-u_l)/winStep);

//    cout << "ind" << ind << " u_l-u_r" << (u_l-u_r) << endl;
    return ind;
}

// this need to know the camerinfo to get the roi from interest_pts
inline void Interest_point2Rois( Vector< geometry_msgs::Point32>& interest_pts, 
        vector< cv::Point>& image_patch_centers,
        vector< Vector< cv::Point> >& image_patches, vector<cv::Size>& winSize,
        const sensor_msgs::CameraInfo& cinfo,
        float Radius, unsigned int winStep){

    // find raw roi with descritized winStep
    vector< vector< unsigned int> > cache_winSize_ind;
    unsigned int ind_tmp;
    for (unsigned int q = 0; q< interest_pts.size(); q++){
        ind_tmp = GetSingleROI( interest_pts[q], Radius, winStep, cinfo); 
        if (cache_winSize_ind.size()<= ind_tmp)
            cache_winSize_ind.resize( ind_tmp+1);
        cache_winSize_ind[ind_tmp].push_back( q);
    }
    
    // copy interest_pts to interest_pts_tmp
    Vector< geometry_msgs::Point32> interest_pts_tmp( interest_pts.size());
    for (unsigned int q = 0; q< interest_pts.size(); q++){
        interest_pts_tmp[q] = interest_pts[q];
    }
    interest_pts.clear();

    // reordering the image_patches and interest_pts from samll size to big size
    unsigned int count =0;
    unsigned int scale_count = 0;
    // skip scale_id ==0 since the width will be zero
    for (unsigned int scale_id=1; scale_id < cache_winSize_ind.size(); scale_id++){
        if (cache_winSize_ind[scale_id].size()!=0){
 	    scale_count++;
            Vector< cv::Point> image_patches_single_scale( cache_winSize_ind[scale_id].size());
            unsigned int width = scale_id*winStep;
            for (unsigned int q = 0; q < cache_winSize_ind[scale_id].size(); q++){
                image_patches_single_scale[q] = 
                    cv::Point(image_patch_centers[ count].x-(width/2), 
                    image_patch_centers[ count].y-(width/2));
                interest_pts.push_back( interest_pts_tmp[ count]);
                count++;
            }
            image_patches.push_back( image_patches_single_scale);
            winSize.push_back( cv::Size(width,width));
        }
    }
    cout << "scale_count" << scale_count <<endl;
}

