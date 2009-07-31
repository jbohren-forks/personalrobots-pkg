/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*
* Author: Alexander Sorokin
*********************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <boost/numeric/ublas/matrix.hpp>

#include "ros/node.h"
#include "ros/publisher.h"

#include "ros/ros.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cv.h>


#include <mapping_msgs/PolygonalMap.h>
#include <sensor_msgs/StereoInfo.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_mech_turk/ExternalAnnotation.h>
#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

#include "annotated_planar_patch_map/annotated_map_lib.h"
#include "annotated_planar_patch_map/projection.h"

#include <boost/thread/mutex.hpp>


using namespace std;
using namespace tf;


class Annotation2DLifterToTaggedPatchMap 
{

public:
  Annotation2DLifterToTaggedPatchMap() 
  {

    out_topic_name_=std::string("poly_object_map");

    tf_ = new tf::TransformListener( ros::Duration(30.0));

    n_.param( std::string("~fixed_frame"), target_frame_, std::string("odom"));

    n_.param( std::string("~dist_tolerance"), dist_tolerance_, -10.0);
    n_.param( std::string("~min_num_indist_tolerance"), min_num_indist_tolerance_, 1);
    n_.param( std::string("~max_allowed_num_outdist_tolerance"), max_allowed_num_outdist_tolerance_, 10000);

    n_.param( std::string("~max_depth"), max_depth_, 10.0);
    n_.param( std::string("~min_depth"), min_depth_, 0.0);

  

    caminfo_sub_ = n_.subscribe<sensor_msgs::CameraInfo>( std::string("cam_info"), 100, boost::bind(&Annotation2DLifterToTaggedPatchMap::handleCameraInfo, this,_1));
    annotation_sub_ = n_.subscribe<cv_mech_turk::ExternalAnnotation>( std::string("annotations_2d"), 100, boost::bind(&Annotation2DLifterToTaggedPatchMap::handleAnnotation, this,_1));
    unlabeled_map_sub_ = n_.subscribe<annotated_map_msgs::TaggedPolygonalMap>( std::string("planar_map"), 100, boost::bind(&Annotation2DLifterToTaggedPatchMap::handleUnlabeledMap, this,_1));


    lifted_pub_ = n_.advertise<annotated_map_msgs::TaggedPolygonalMap>(out_topic_name_,1);

    double tf_tolerance_secs ;
    ros::Node::instance()->param("~tf_tolerance_secs", tf_tolerance_secs, 0.0) ;
    if (tf_tolerance_secs < 0)
      ROS_ERROR("Parameter tf_tolerance_secs<0 (%f)", tf_tolerance_secs) ;
    ROS_INFO("tf Tolerance: %f seconds", tf_tolerance_secs) ;    
    tf_->setExtrapolationLimit(ros::Duration(tf_tolerance_secs)) ;
  };
  void handleUnlabeledMap(const annotated_map_msgs::TaggedPolygonalMapConstPtr ptrMap)
  {
    boost::mutex::scoped_lock lock(lift_mutex_);
    unlabeled_map_=ptrMap;
    printf("Unlabeled map\n");
  }
  void handleCameraInfo(const sensor_msgs::CameraInfoConstPtr ptrInfo)
  {
    boost::mutex::scoped_lock lock(lift_mutex_);
    cam_info_=ptrInfo;
  }
  void handleAnnotation(const cv_mech_turk::ExternalAnnotationConstPtr annotation)
  {
    boost::mutex::scoped_lock lock(lift_mutex_);

    annotation2d_object_=annotation;

    if(!cam_info_)
      return ;
    if(!unlabeled_map_)
      return;

    try
    {

      printf("Annotation\n");
      for (unsigned int i=0;i<annotation2d_object_->get_polygons_size();i++)
      {
	const cv_mech_turk::AnnotationPolygon &poly=annotation2d_object_->polygons[i];
	for (unsigned int iP=0;iP<poly.get_control_points_size();iP++)
        {
	  printf("\t\tAnnotation (%f, %f)\n",
		 poly.control_points[iP].x,
		 poly.control_points[iP].y);
	}
      }


      annotated_map_msgs::TaggedPolygonalMap polymapOut;
      liftAnnotation(annotation2d_object_,polymapOut);

      printf("Sending polymap with %d polygons\n", polymapOut.get_polygons_size());

      if(polymapOut.get_polygons_size()>0)
      {
	printf("\tfirst polygon has %d pts\n", polymapOut.polygons[0].polygon.get_points_size());
	lifted_pub_.publish( polymapOut);     
      }

    }
    catch (TransformException& ex)
    {
      ROS_ERROR("Failure to transform detected object:: %s\n", ex.what());
    }


  };
  void liftAnnotation(cv_mech_turk::ExternalAnnotationConstPtr annotation2d_object_,annotated_map_msgs::TaggedPolygonalMap& polymapOut)
  {

    annotated_map_msgs::TaggedPolygonalMap transformed_map_3D;
    annotated_map_msgs::TaggedPolygonalMap transformed_map_3D_fixed_frame;
    annotated_map_msgs::TaggedPolygonalMap transformed_map_2D;

    ROS_INFO_STREAM("Lifting annotation for " << annotation2d_object_->reference_time << " at caminfo frame " <<cam_info_->header.stamp << " unlabeled map has " << unlabeled_map_->polygons.size() << " polygons");
    //Get the 3D map into the coordinate frame of the camera
    ROS_DEBUG("Transform 3D map to frame: %s",annotation2d_object_->reference_frame.c_str());

    annotated_map_lib::transformAnyObject(annotation2d_object_->reference_frame,annotation2d_object_->reference_time,tf_,*unlabeled_map_,transformed_map_3D);
    //annotated_map_lib::transformAnyObject(cam_info_->header.frame_id,annotation2d_object_->reference_time,tf_,*unlabeled_map_,transformed_map_3D);

    ROS_DEBUG("Transform 3D map to frame: %s",target_frame_.c_str());
    annotated_map_lib::transformAnyObject(target_frame_,annotation2d_object_->reference_time,tf_,*unlabeled_map_,transformed_map_3D_fixed_frame);

    ROS_DEBUG("Project map");
    //Project the 3D map into the image coordinates
    annotated_planar_patch_map::projection::projectAnyObject(*cam_info_,transformed_map_3D,transformed_map_2D);


    ROS_DEBUG("Bind map");    
    //Bind 2D annotations to the projected 3D map and lift the annotations into 3D
    bindAnnotationsToMap(annotation2d_object_,transformed_map_3D_fixed_frame,transformed_map_2D,polymapOut);
    polymapOut.header.stamp=annotation2d_object_->reference_time;
    polymapOut.header.frame_id=target_frame_;
      
  }

  void bindAnnotationsToMap(const cv_mech_turk::ExternalAnnotationConstPtr annotation, 
                            const annotated_map_msgs::TaggedPolygonalMap& transformed_map_3D, 
                            const annotated_map_msgs::TaggedPolygonalMap &transformed_map_2D,
                            annotated_map_msgs::TaggedPolygonalMap &polymapOut)
  {

    //CvMemStorage* storage = cvCreateMemStorage();

    ROS_DEBUG("\tAnnotation has %d polygons",annotation->get_polygons_size());
    int removed_by_viewport=0;
    int removed_by_outlier_count=0;
    int removed_pts_by_depth=0;

    for(unsigned int iAnnotatedPolygon=0;iAnnotatedPolygon<annotation->get_polygons_size();iAnnotatedPolygon++)
    {
      const cv_mech_turk::AnnotationPolygon &poly=annotation->polygons[iAnnotatedPolygon];    
	
      unsigned int pt_count=poly.get_control_points_size();
      if(pt_count<3)
      {
        removed_by_viewport++;
        continue;
      }
      CvMat* poly_annotation = cvCreateMat( 1, pt_count , CV_32FC2 );

      for (unsigned int iP=0;iP<pt_count;iP++){
        CvPoint2D32f pt;
        pt.x=poly.control_points[iP].x;
        pt.y=poly.control_points[iP].y;

        CV_MAT_ELEM( *poly_annotation, CvPoint2D32f, 0, iP ) = pt;

      }

      unsigned int num_3D_poly=transformed_map_3D.get_polygons_size();
      ROS_DEBUG("\t3D map has %d polygons",num_3D_poly);

      std::vector<int> overlap;
      overlap.reserve(num_3D_poly);
	
      int num_overlap=0;
      for(unsigned int iPoly = 0; iPoly<num_3D_poly; iPoly++)
      {
        //we're checking this one
        const robot_msgs::Polygon3D &map_poly=transformed_map_2D.polygons[iPoly].polygon;
        int num_in=0,num_out=0;
        for(unsigned int iPt=0;iPt<map_poly.get_points_size();iPt++)
        {

          bool in_depth= (map_poly.points[iPt].z <= max_depth_) && (map_poly.points[iPt].z >= min_depth_);
          if(! in_depth)
          {
            removed_pts_by_depth++;
            num_out++;
            continue;
          }
          CvPoint2D32f pt;
          pt.x=map_poly.points[iPt].x;
          pt.y=map_poly.points[iPt].y;
          double dist = cvPointPolygonTest( poly_annotation, pt, 0 );
          if(dist>dist_tolerance_)
          {
            num_in++;
          }
          else
          {
            num_out++;
          }
	    
        }
        if(num_in>= min_num_indist_tolerance_ && 
           num_out<= max_allowed_num_outdist_tolerance_)
        {
          overlap[iPoly]=1;
          num_overlap++;
          printf("in %d out %d\n",num_in,num_out);
        }
        else
        {
          removed_by_outlier_count++;

          overlap[iPoly]=0;
        }
      }

      unsigned int num_polygons_to_add = num_overlap;
      unsigned int old_num_polygons = polymapOut.get_polygons_size();
      polymapOut.set_polygons_size(old_num_polygons+num_polygons_to_add);

      printf("Num to add: %d\n",num_polygons_to_add);

      int iPolyAdd=0;
      for(unsigned int iPoly = 0; iPoly<num_3D_poly; iPoly++)
      {
        if(!overlap[iPoly])
          continue;

        printf("%d\n",iPoly);
        //create new tagged polygon
        annotated_map_msgs::TaggedPolygon3D newPoly;
        newPoly.set_tags_size(1);
        newPoly.tags[0]=poly.object_name;
        newPoly.set_tags_chan_size(1);
        newPoly.tags_chan[0].name=std::string("hits"); //num labeled
        newPoly.tags_chan[0].set_vals_size(1);
        newPoly.tags_chan[0].vals[0]=1.0;

        newPoly.polygon=transformed_map_3D.polygons[iPoly].polygon;
	    
        //append polygon to the map
        polymapOut.polygons[iPolyAdd+old_num_polygons]=newPoly;
        //printf("\tAdded polygon with %d points\n", polymapOut.polygons[iPoly+old_num_polygons].polygon.get_points_size());      

        iPolyAdd++;

      }
      cvReleaseMat( &poly_annotation );

      ROS_DEBUG_STREAM("\tRemoved by viewport " << double(removed_by_viewport));
      ROS_DEBUG_STREAM("\tRemoved by outlier " << double(removed_pts_by_depth));
      ROS_DEBUG_STREAM("\tRemoved by outlier " << double(removed_by_outlier_count));

    } 
    //cvClearMemStorage( storage );

    printf("Polymap size %d\n",polymapOut.get_polygons_size());
  }


  void printAnnotations2D(cv_mech_turk::ExternalAnnotation annotation)
  {

    FILE* fOut=fopen("/u/sorokin/bags/run_may_21/dump/annotations2D.txt","w");
    ROS_DEBUG("\tAnnotation map has %d polygons",annotation.get_polygons_size());    
    for(unsigned int iAnnotatedPolygon=0;iAnnotatedPolygon<annotation.get_polygons_size();iAnnotatedPolygon++)
    {
      cv_mech_turk::AnnotationPolygon &poly=annotation.polygons[iAnnotatedPolygon];    
	
      unsigned int pt_count=poly.get_control_points_size();
      ROS_DEBUG("\tPoly %d has %d points\n",iAnnotatedPolygon,pt_count);

      if(pt_count<3)
      {
        continue;
      }
      for (unsigned int iP=0;iP<pt_count;iP++)
      {
        fprintf(fOut,"%f %f\n",
                poly.control_points[iP].x,  
                poly.control_points[iP].y);

      }
      fprintf(fOut,"0 0\n");
    } 
    ROS_DEBUG("\tClosing the file");
    fclose(fOut);
  }



  void printPolygon3D(mapping_msgs::PolygonalMap transformed_map,std::string tag)
  {

    std::string fname=std::string("/u/sorokin/bags/run_may_21/dump/polygons3D__")+tag+std::string(".txt");
    FILE* fOut=fopen(fname.c_str(),"w");

    unsigned int num_3D_poly=transformed_map.get_polygons_size();
    std::vector<int> overlap;
    overlap.reserve(num_3D_poly);
    
    for(unsigned int iPoly = 0; iPoly<num_3D_poly; iPoly++)
    {
      //we're checking this one
      robot_msgs::Polygon3D &map_poly=transformed_map.polygons[iPoly];
      for(unsigned int iPt=0;iPt<map_poly.get_points_size();iPt++){
        fprintf(fOut,"%f %f %f\n",map_poly.points[iPt].x,
                map_poly.points[iPt].y,
                map_poly.points[iPt].z);

      }
      fprintf(fOut,"0 0 0\n");
    } 
    fclose(fOut);
  }



protected:

  boost::mutex lift_mutex_;

  ros::NodeHandle n_;

  ros::Publisher lifted_pub_;

  ros::Subscriber unlabeled_map_sub_;
  ros::Subscriber annotation_sub_;
  ros::Subscriber caminfo_sub_;

  tf::TransformListener *tf_;

  cv_mech_turk::ExternalAnnotationConstPtr annotation2d_object_;
  annotated_map_msgs::TaggedPolygonalMapConstPtr unlabeled_map_;
  sensor_msgs::CameraInfoConstPtr cam_info_;




  std::string target_frame_;

  std::string out_topic_name_;


  double dist_tolerance_; //in pixels
  double min_depth_; //in meters?
  double max_depth_; //in meters?
  int min_num_indist_tolerance_; //in vertices
  int max_allowed_num_outdist_tolerance_; //in vertices

};

int main(int argc, char **argv)
{
  ros::init(argc, argv,"annotation_lifter");

  try
  {
    Annotation2DLifterToTaggedPatchMap lifter;

    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  
  return 0;
}







