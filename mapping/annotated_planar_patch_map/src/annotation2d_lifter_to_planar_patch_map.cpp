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


#include <robot_msgs/PolygonalMap.h>
#include <image_msgs/StereoInfo.h>

#include <cv_mech_turk/ExternalAnnotation.h>
#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

#include "annotated_planar_patch_map/annotated_map_lib.h"

using namespace std;
using namespace tf;


class Annotation2DLifterToPlanarPatchMap : public ros::Node
{
public:
  Annotation2DLifterToPlanarPatchMap() : ros::Node("annotation2d_lifter_to_planar_patch_map") 
  {

    out_topic_name_=std::string("poly_object_map");

    tf_ = new tf::TransformListener( *this, true);

    param( std::string("target_frame"), target_frame_, std::string("odom"));

    param( std::string("dist_tolerance"), dist_tolerance_, -10.0);
    param( std::string("min_num_indist_tolerance"), min_num_indist_tolerance_, 1);
    param( std::string("max_allowed_num_outdist_tolerance"), max_allowed_num_outdist_tolerance_, 10000);

    param( std::string("max_depth"), max_depth_, 10.0);
    param( std::string("min_depth"), min_depth_, 0.0);


    subscribe( std::string("stereo_info"), stereo_info_, &Annotation2DLifterToPlanarPatchMap::handleStereoInfo, 500);
    subscribe( std::string("annotations_2d"), annotation2d_object_, &Annotation2DLifterToPlanarPatchMap::handleAnnotation, 500);
    subscribe( std::string("planar_map"), unlabeled_map_, &Annotation2DLifterToPlanarPatchMap::handleUnlabeledMap, 500);


    advertise<annotated_map_msgs::TaggedPolygonalMap>(out_topic_name_,1);
  };
  void handleUnlabeledMap()
  {
    printf("Unlabeled map\n");
  }
  void handleStereoInfo()
  {
    //printf("StereoInfo\n");
  }
  void handleAnnotation()
  {
    try
    {

      printf("Annotation\n");
      for (unsigned int i=0;i<annotation2d_object_.get_polygons_size();i++)
      {
	cv_mech_turk::AnnotationPolygon &poly=annotation2d_object_.polygons[i];
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
	publish( out_topic_name_, polymapOut);     
      }
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("Failure to transform detected object:: %s\n", ex.what());
    }


  };
  void liftAnnotation(cv_mech_turk::ExternalAnnotation annotation2d_object_,annotated_map_msgs::TaggedPolygonalMap& polymapOut)
  {

    robot_msgs::PolygonalMap transformed_map_3D;
    robot_msgs::PolygonalMap transformed_map_3D_fixed_frame;
    robot_msgs::PolygonalMap transformed_map_2D;

    //Get the 3D map into the coordinate frame of the camera
    ROS_DEBUG("Transform 3D map to frame: %s",annotation2d_object_.reference_frame.c_str());
    annotated_map_lib::transformAnyObject(annotation2d_object_.reference_frame,tf_,unlabeled_map_,transformed_map_3D);

    ROS_DEBUG("Transform 3D map to frame: %s",target_frame_.c_str());
    annotated_map_lib::transformAnyObject(target_frame_,tf_,unlabeled_map_,transformed_map_3D_fixed_frame);

    ROS_DEBUG("Project map");
    //Project the 3D map into the image coordinates
    projectPolygonalMap(stereo_info_,transformed_map_3D,transformed_map_2D);

    /*
      ROS_DEBUG("Print annotations");    
      printAnnotations2D(annotation2d_object_);
      ROS_DEBUG("Print transformed 3D map");
      printPolygon3D(transformed_map_3D,"map3D");
      ROS_DEBUG("Print transformed 2D map");
      printPolygon3D(transformed_map_2D,"map2D");
    */
    ROS_DEBUG("Bind map");    
    //Bind 2D annotations to the projected 3D map and lift the annotations into 3D
    bindAnnotationsToMap(annotation2d_object_,transformed_map_3D_fixed_frame,transformed_map_2D,polymapOut);
    polymapOut.header.stamp=annotation2d_object_.reference_time;
    polymapOut.header.frame_id=target_frame_;
      
  }

  void projectPolygonalMap(image_msgs::StereoInfo stereo_info_, robot_msgs::PolygonalMap transformed_map_3D, robot_msgs::PolygonalMap &transformed_map_2D)
  {

    //Get projections matrix
    //tf::Transform projection;
    //btScalar tmp_proj[16];
    double projection[16];
    for(int i=0;i<16;i++)
      projection[i]=stereo_info_.RP[i];
      //tmp_proj[i]=stereo_info_.RP[i];

    //projection.setFromOpenGLMatrix(tmp_proj);

    //Project all points of all polygons
    unsigned int num_polygons = transformed_map_3D.get_polygons_size();
    transformed_map_2D.set_polygons_size(num_polygons);
    for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
    {
      //create new polygon 2D (z=1)
      robot_msgs::Polygon3D newPoly;
      projectPolygonPoints(projection,transformed_map_3D.polygons[iPoly],newPoly);

      //put the polygon into 2D map
      transformed_map_2D.polygons[iPoly]=newPoly;
    }
  }
  

  void projectPolygonPoints(double* projection,robot_msgs::Polygon3D polyIn,robot_msgs::Polygon3D& polyOut)
  {
    //Project all points of all polygons
    unsigned int num_pts = polyIn.get_points_size();
    polyOut.set_points_size(num_pts);
    for(unsigned int iPt = 0; iPt<num_pts; iPt++)
    {
      robot_msgs::Point32 &mpt=polyIn.points[iPt];
      Vector3 pt(-mpt.y,-mpt.z,mpt.x);
      //Vector3 projected_pt=projection * pt;

      robot_msgs::Point32 projected_pt;
      projected_pt.x=
        projection[0]*pt.x()+
        projection[4]*pt.y()+
        projection[8]*pt.z()+
        projection[12]*1;
      projected_pt.y=
        projection[1]*pt.x()+
        projection[5]*pt.y()+
        projection[9]*pt.z()+
        projection[13]*1;
      projected_pt.z=
        projection[2]*pt.x()+
        projection[6]*pt.y()+
        projection[10]*pt.z()+
        projection[14]*1;
      /*double s=
        projection[3]*pt.x()+
        projection[7]*pt.y()+
        projection[11]*pt.z()+
        projection[15]*1;*/

      robot_msgs::Point32 &new_pt=polyOut.points[iPt];
      new_pt.x= projected_pt.x;
      new_pt.y= projected_pt.y;
      new_pt.z = projected_pt.z;
      new_pt.z = pt.z();
      //new_pt.x = projected_pt.x();
      //new_pt.y = projected_pt.y();
      //new_pt.z = projected_pt.z();

      new_pt.x= new_pt.x*(stereo_info_.width/2) +stereo_info_.width/2;
      new_pt.y= new_pt.y*(stereo_info_.height/2) +stereo_info_.height/2;
      //new_pt.x= new_pt.x*stereo_info_.width;
      //new_pt.y= new_pt.y*stereo_info_.height;	
      /*printf("\t\tProjected from (%f, %f, %f) to (%f, %f)\n",
        mpt.x,mpt.y,mpt.z,
        new_pt.x,new_pt.y);*/
    }
  }

  void bindAnnotationsToMap(cv_mech_turk::ExternalAnnotation annotation2d_object, robot_msgs::PolygonalMap transformed_map_3D, robot_msgs::PolygonalMap transformed_map_2D,annotated_map_msgs::TaggedPolygonalMap &polymapOut)
  {

    //CvMemStorage* storage = cvCreateMemStorage();

    printf("Num annotations %d\n",annotation2d_object_.get_polygons_size());
    for(unsigned int iAnnotatedPolygon=0;iAnnotatedPolygon<annotation2d_object_.get_polygons_size();iAnnotatedPolygon++)
    {
      cv_mech_turk::AnnotationPolygon &poly=annotation2d_object_.polygons[iAnnotatedPolygon];    
	
      unsigned int pt_count=poly.get_control_points_size();
      if(pt_count<3)
      {
        continue;
      }
      CvMat* poly_annotation = cvCreateMat( 1, pt_count , CV_32FC2 );
      //CvSeq* poly_annotation = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32FC2, 
      //				      sizeof(CvSeq),
      //				      sizeof(CvPoint2D32f), storage );
      //printf("%0x\n",(unsigned int)(void*)poly_annotation);

      for (unsigned int iP=0;iP<pt_count;iP++){
        CvPoint2D32f pt;
        pt.x=poly.control_points[iP].x;
        pt.y=poly.control_points[iP].y;
        //cvSeqPush( poly_annotation, &pt );

        CV_MAT_ELEM( *poly_annotation, CvPoint2D32f, 0, iP ) = pt;

      }

      //CvPoint2D32f* pt1=(CvPoint2D32f*)cvGetSeqElem( poly_annotation, 1 );
      //printf("p: %f, %f\n",pt1->x,pt1->y);

      unsigned int num_3D_poly=transformed_map_3D.get_polygons_size();
      std::vector<int> overlap;
      overlap.reserve(num_3D_poly);
	
      int num_overlap=0;
      for(unsigned int iPoly = 0; iPoly<num_3D_poly; iPoly++)
      {
        //we're checking this one
        robot_msgs::Polygon3D &map_poly=transformed_map_2D.polygons[iPoly];
        int num_in=0,num_out=0;
        for(unsigned int iPt=0;iPt<map_poly.get_points_size();iPt++)
        {

          bool in_depth=(map_poly.points[iPt].z <= max_depth_) && (map_poly.points[iPt].z >= min_depth_);
          if(! in_depth)
          {
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
           num_out< max_allowed_num_outdist_tolerance_)
        {
          overlap[iPoly]=1;
          num_overlap++;
          printf("in %d out %d\n",num_in,num_out);
        }
        else
        {
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

        newPoly.polygon=transformed_map_3D.polygons[iPoly];
	    
        //append polygon to the map
        polymapOut.polygons[iPolyAdd+old_num_polygons]=newPoly;
        //printf("\tAdded polygon with %d points\n", polymapOut.polygons[iPoly+old_num_polygons].polygon.get_points_size());      

        iPolyAdd++;

      }
      cvReleaseMat( &poly_annotation );

    } 
    //cvClearMemStorage( storage );
 
    printf("Polymap size %d\n",polymapOut.get_polygons_size());
  }


  void printAnnotations2D(cv_mech_turk::ExternalAnnotation annotation2d_object)
  {

    FILE* fOut=fopen("/u/sorokin/bags/run_may_21/dump/annotations2D.txt","w");
    ROS_DEBUG("\tAnnotation map has %d polygons",annotation2d_object_.get_polygons_size());    
    for(unsigned int iAnnotatedPolygon=0;iAnnotatedPolygon<annotation2d_object_.get_polygons_size();iAnnotatedPolygon++)
    {
      cv_mech_turk::AnnotationPolygon &poly=annotation2d_object_.polygons[iAnnotatedPolygon];    
	
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



  void printPolygon3D(robot_msgs::PolygonalMap transformed_map,std::string tag)
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
  tf::TransformListener *tf_;

  cv_mech_turk::ExternalAnnotation annotation2d_object_;
  robot_msgs::PolygonalMap unlabeled_map_;
  image_msgs::StereoInfo stereo_info_;

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
    Annotation2DLifterToPlanarPatchMap lifter;

    lifter.spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  
  return 0;
}

