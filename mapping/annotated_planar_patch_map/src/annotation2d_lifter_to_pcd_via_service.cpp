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
#include <std_msgs/ColorRGBA.h>
#include "tf/message_notifier.h"

#include <cv_mech_turk/ExternalAnnotation.h>
#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

#include "annotated_planar_patch_map/annotated_map_lib.h"
#include "annotated_planar_patch_map/projection.h"

#include "annotated_planar_patch_map/BuildAnnotatedMap.h"
#include "annotated_planar_patch_map/rolling_history.h"

#include "bagserver/History.h"

#include "object_names/Name2Float.h"
#include "object_names/Name2Color.h"

#include "point_cloud_assembler/BuildCloud.h"

#include "point_cloud_mapping/normal_estimation_in_proc.h"


using namespace std;
using namespace tf;


class AnnotationLifterToPcdViaService
{

public:
  AnnotationLifterToPcdViaService() 
  {
  }


  void init()
  {
    boost::mutex::scoped_lock lock(lift_mutex_);


    out_topic_name_=std::string("annotated_cloud");


    n_.param( std::string("~fixed_frame"), fixed_frame_, std::string("map"));
    ROS_INFO_STREAM("Fixed frame is " <<fixed_frame_);

    n_.param( std::string("~local_fixed_frame"), local_fixed_frame_, std::string("odom_combined"));
    ROS_INFO_STREAM("Local fixed frame is " <<local_fixed_frame_);

    /*Geometry location tolerance -ignored now*/
    n_.param( std::string("~dist_tolerance"), dist_tolerance_, -10.0); 

    n_.param( std::string("~max_depth"), max_depth_, 5.0);
    n_.param( std::string("~min_depth"), min_depth_, 0.01);


    n_.param( std::string("~use_color"), use_colors_, false);
    n_.param( std::string("~annotate_reprojection"), annotate_reprojection_, false);
    n_.param( std::string("~annotate_image_id"), annotate_image_id_, false);


    /*How much to wait with lifting*/
    double delay;
    n_.param( std::string("~lifting_delay"), delay, 0.0);
    lifting_delay_=ros::Duration(delay);
    ROS_INFO_STREAM("Lifting delay:"<< lifting_delay_);
        

    /*Geometry querying config */
    double interval;
    n_.param( std::string("~interval_before"), interval, 1.0);
    interval_before_image_=ros::Duration(interval);

    n_.param( std::string("~interval_after"), interval, 1.0);
    interval_after_image_=ros::Duration(interval);




    // **** Get the TF Notifier Tolerance ****
    /*double tf_tolerance_secs ;
    n_.param("~tf_tolerance_secs", tf_tolerance_secs, 0.0) ;
    if (tf_tolerance_secs < 0)
      ROS_ERROR("Parameter tf_tolerance_secs<0 (%f)", tf_tolerance_secs) ;
      ROS_INFO("tf Tolerance: %f seconds", tf_tolerance_secs) ;    */

    tf_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());

    lifted_pub_=n_.advertise<sensor_msgs::PointCloud>(out_topic_name_,1);
    
    annotation_notifier_=new tf::MessageNotifier<cv_mech_turk::ExternalAnnotation>(*tf_,
                                                                                   boost::bind(&AnnotationLifterToPcdViaService::handleAnnotation, this,_1),
                                                                                   std::string("annotations_2d"),
                                                                                   fixed_frame_,
                                                                                   4000);

    annotation_notifier_->setTolerance(lifting_delay_+ros::Duration(0.1));

    typedef annotated_planar_patch_map::RollingHistory<sensor_msgs::CameraInfo> cam_hist_type;


    boost::shared_ptr<cam_hist_type> cam_hist_lookup(new cam_hist_type("cam_info","~cam_info_hist/"));
    cam_hist_ = cam_hist_lookup;


  };


  void handleAnnotation(const boost::shared_ptr<cv_mech_turk::ExternalAnnotation> annotation)
  {
    try{
    ROS_INFO_STREAM("Annotation " << annotation->header.stamp);

    //const image_msgs::CameraInfoConstPtr cam_info = cam_hist_->getMsgAtExactTime(annotation->header.stamp);
    const sensor_msgs::CameraInfoConstPtr cam_info = cam_hist_->getMsgNearTime(annotation->header.stamp,ros::Duration(0.01));

    if(cam_info==NULL)
    {
      ROS_WARN_STREAM("Ignoring annotation at "<< annotation->header.stamp << ". Failed to get cam info. ");
      return;
    }

    point_cloud_assembler::BuildCloud::Request req;
    point_cloud_assembler::BuildCloud::Response resp;
    req.begin = annotation->reference_time-interval_before_image_;
    req.end = annotation->reference_time+interval_after_image_;

    ros::service::call("build-cloud",req,resp);

    liftAndSend(annotation, cam_info, resp.cloud);
    }
    catch(std::runtime_error& e)
    {
      ROS_ERROR_STREAM("Caught exception while processing annotation " << annotation->header.stamp );
      ROS_ERROR_STREAM("\t" << e.what() );
    }
  }

  void liftAndSend(const boost::shared_ptr<cv_mech_turk::ExternalAnnotation const> annotation, 
                   const boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info, 
                   const sensor_msgs::PointCloud& cloud)
  {
    sensor_msgs::PointCloud transformed_map_3D;
    sensor_msgs::PointCloud transformed_map_3D_fixed;
    sensor_msgs::PointCloud transformed_map_3D_fixed_w_normals;
    sensor_msgs::PointCloud transformed_map_2D;
    sensor_msgs::PointCloud map_final;

    ROS_INFO_STREAM("Lifting \n\tannotation "<< annotation->header.frame_id);
    ROS_INFO_STREAM("\t cam_info "<< cam_info->header.frame_id);
    ROS_INFO_STREAM("\t cloud "<< cloud.header.frame_id);
    tf_->transformPointCloud(annotation->reference_frame,annotation->reference_time,cloud,local_fixed_frame_,transformed_map_3D);

    tf_->transformPointCloud(fixed_frame_,annotation->reference_time,cloud,local_fixed_frame_,transformed_map_3D_fixed);

    annotated_planar_patch_map::projection::projectAnyObject(*cam_info, transformed_map_3D,transformed_map_2D);

    normal_estimator_.process_cloud(transformed_map_3D_fixed,transformed_map_3D_fixed_w_normals);

    ROS_INFO("\t cloud sizes %d/%d/%d/%d ",cloud.pts.size(),transformed_map_3D.pts.size(),transformed_map_3D.pts.size(),transformed_map_3D_fixed.pts.size());
    bindAnnotations(annotation,transformed_map_2D,transformed_map_3D_fixed_w_normals,map_final);

    if(map_final.pts.size()>0)
    {
      ROS_INFO("Lifting produced point cloud with %d points", map_final.pts.size());
      lifted_pub_.publish(map_final);
    }
    else
    {
      ROS_WARN("Lifting produced empty point cloud");
    }
  }


  void bindAnnotations(cv_mech_turk::ExternalAnnotationConstPtr annotation, 
                       const sensor_msgs::PointCloud& map_2D, const sensor_msgs::PointCloud& map_3D, 
                       sensor_msgs::PointCloud& map_final)
  {

    // Allocate overlap buffer to store 1-1 correspondence
    unsigned int num_3D_pts=map_3D.pts.size();
    printf("%d n3dp\n",num_3D_pts);	


    std::vector<int> overlap;
    overlap.reserve(num_3D_pts);

    // Initialize it to 0 - no correspondence
    for(unsigned int iPt = 0; iPt<num_3D_pts; iPt++)
    {
      overlap[iPt]=0;
    }

    // Go over all annotated polygons and assign points to polygons on first-come-first-served basis
    ROS_INFO("Num annotations %d\n",annotation->polygons.size());
    unsigned int num_poly=annotation->polygons.size();

    std::vector<float> object_labels;
    object_labels.reserve(num_poly);

    std::vector<std_msgs::ColorRGBA> object_colors;
    if(use_colors_)
    {
      object_colors.reserve(num_poly);
    }

    int num_in=0;
    for(unsigned int iAnnotatedPolygon=0;iAnnotatedPolygon<num_poly;iAnnotatedPolygon++)
    {
      const cv_mech_turk::AnnotationPolygon &poly=annotation->polygons[iAnnotatedPolygon];    
	
      unsigned int pt_count=poly.get_control_points_size();
      if(pt_count<3)
      {
        //Ignore these
        continue;
      }

      float object_id=1.0;
      object_names::Name2Float::Request req;
      object_names::Name2Float::Response resp;
      req.name=poly.object_name;

      ros::service::call("name_to_float",req,resp);

      object_id=resp.id;
      if(object_id<0.1) //i.e. ==0
      {
        ROS_WARN_STREAM("Unknown object " << poly.object_name );
        continue;
      }
      object_labels[iAnnotatedPolygon]=object_id;

      if(use_colors_)
      {
        object_names::Name2Color::Request req;
        object_names::Name2Color::Response resp;
        req.name=poly.object_name;
        
        ros::service::call("name_to_color",req,resp);
        object_colors[iAnnotatedPolygon]=resp.color;
      }


      //Convert polygon to CV MAT
      ROS_DEBUG_STREAM("cvCreateMat( 1, " << pt_count <<" , CV_32FC2 );");
      CvMat* poly_annotation = cvCreateMat( 1, pt_count , CV_32FC2 );

      for (unsigned int iP=0;iP<pt_count;iP++){
        CvPoint2D32f pt;
        pt.x=poly.control_points[iP].x;
        pt.y=poly.control_points[iP].y;
        CV_MAT_ELEM( *poly_annotation, CvPoint2D32f, 0, iP ) = pt;
      }

      //Test each point, whether it's inside the polygon or not.
      for(unsigned int iPt = 0; iPt<num_3D_pts; iPt++)
      {
        if(overlap[iPt])
          continue;

        bool in_depth=(map_2D.pts[iPt].z <= max_depth_) && (map_2D.pts[iPt].z >= min_depth_);
        if(! in_depth)
        {
          continue;
        }
        CvPoint2D32f pt;
        pt.x=map_2D.pts[iPt].x;
        pt.y=map_2D.pts[iPt].y;
        ROS_DEBUG_STREAM("cvPointPolygonTest");
        double dist = cvPointPolygonTest( poly_annotation, pt, 1 );
        
        if(dist>dist_tolerance_)
        {
          //If the point is inside the polygon, assign it to this annotation.
          num_in++;
          overlap[iPt]=iAnnotatedPolygon+1;
        }
      }      
      cvReleaseMat( &poly_annotation );
    }


    map_final.header=map_3D.header;
    map_final.pts.resize(num_in);
    std::string new_channel_name=std::string("ann-")+annotation->task;
    unsigned int nC=map_3D.chan.size();
    unsigned int nCout=nC+1;
    if(annotate_reprojection_)
    {
      nCout+=3;
    }
    if(use_colors_)
    {
      nCout+=1;      
    }
    if(annotate_image_id_)
    {
      nCout+=1;
    }
    map_final.chan.resize(nCout);

    for(unsigned int iC=0;iC<nC;iC++)
    {
      map_final.chan[iC].vals.resize(num_in);
      map_final.chan[iC].name=map_3D.chan[iC].name;
    }

    unsigned int free_channel=nC;
    unsigned int new_channel_id = free_channel;
    free_channel++;
    unsigned int chan_X_id = -1;
    unsigned int chan_Y_id = -1;
    unsigned int chan_Z_id = -1;
    unsigned int chan_RGB_id = -1;
    unsigned int chan_IMG_id = -1;
    if(annotate_reprojection_)
    {
       chan_X_id = free_channel;
       free_channel++;
       chan_Y_id = free_channel;
       free_channel++;
       chan_Z_id = free_channel;
       free_channel++;
    }
    if(use_colors_)
    {
       chan_RGB_id = free_channel;
       free_channel++;
    }
    if(annotate_image_id_)
    {
       chan_IMG_id = free_channel;
       free_channel++;
    }



    map_final.chan[new_channel_id].name=new_channel_name;
    map_final.chan[new_channel_id].vals.resize(num_in);

    if(annotate_reprojection_)
    {
      map_final.chan[chan_X_id].name="imgX";
      map_final.chan[chan_X_id].vals.resize(num_in);
      map_final.chan[chan_Y_id].name="imgY";
      map_final.chan[chan_Y_id].vals.resize(num_in);
      map_final.chan[chan_Z_id].name="imgZ";
      map_final.chan[chan_Z_id].vals.resize(num_in);
    }
    if(use_colors_)
    {
      map_final.chan[chan_RGB_id].name="rgb";
      map_final.chan[chan_RGB_id].vals.resize(num_in);
    }

    if(annotate_image_id_)
    {
      map_final.chan[chan_IMG_id].name="img_id";
      map_final.chan[chan_IMG_id].vals.resize(num_in);
    }

    unsigned int iOut=0;
    for(unsigned int iPt = 0; iPt<num_3D_pts; iPt++)
    {
        if(overlap[iPt]==0)
          continue;

        for(unsigned int iC=0;iC<nC;iC++)
        {
          map_final.chan[iC].vals[iOut]=map_3D.chan[iC].vals[iPt];
        }        
        map_final.chan[new_channel_id].vals[iOut] = object_labels[overlap[iPt]-1];
        if(annotate_reprojection_)
        {        
          map_final.chan[chan_X_id].vals[iOut] = map_2D.pts[iPt].x;
          map_final.chan[chan_Y_id].vals[iOut] = map_2D.pts[iPt].y;
          map_final.chan[chan_Z_id].vals[iOut] = map_2D.pts[iPt].z;
        }
        if(use_colors_)
        {
          const std_msgs::ColorRGBA& color=object_colors[overlap[iPt]-1];
          int r=int(round(color.r*255));
          int g=int(round(color.g*255));
          int b=int(round(color.b*255));
          int rgb = (r << 16) | (g << 8) | b;
          map_final.chan[chan_RGB_id].vals[iOut] = *reinterpret_cast<float*>(&rgb);

        }
        if(annotate_image_id_)
        {
          ROS_ERROR("Image IDs are not supported yet");
        }
        map_final.pts[iOut] = map_3D.pts[iPt];
        iOut++;
    }
  }


protected:


  ros::NodeHandle n_;
  ros::Publisher lifted_pub_;

  boost::shared_ptr<tf::TransformListener> tf_;
  tf::MessageNotifier<cv_mech_turk::ExternalAnnotation>* annotation_notifier_ ;
  boost::shared_ptr<annotated_planar_patch_map::RollingHistory<sensor_msgs::CameraInfo> > cam_hist_;

  boost::mutex lift_mutex_;



  point_cloud_mapping::NormalEstimationInProc normal_estimator_;


  bool annotate_reprojection_;
  bool use_colors_;
  bool annotate_image_id_;

  /** @brief The frame to which the map is transformed */
  std::string fixed_frame_;

  /** @brief The frame used to aggregate local snapshots */
  std::string local_fixed_frame_;

  /** @brief The topic name to send the resulting cloud */
  std::string out_topic_name_;

  /** @brief intervals in which to collect geometry for the annotations */
  ros::Duration interval_after_image_;
  ros::Duration interval_before_image_;

  ros::Duration lifting_delay_;

  /** @brief Reject anything outside of this range */
  double min_depth_; //in meters?
  double max_depth_; //in meters?


  double dist_tolerance_; //in pixels
  int min_num_indist_tolerance_; //in vertices
  int max_allowed_num_outdist_tolerance_; //in vertices

};

int main(int argc, char **argv)
{
  ros::init(argc, argv,"annotation_lifter");

  try
  {
    AnnotationLifterToPcdViaService lifter;
    lifter.init();

    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  
  return 0;
}







