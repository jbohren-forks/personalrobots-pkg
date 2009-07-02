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

#include <boost/numeric/ublas/matrix.hpp>

#include <opencv/cv.h>

#include "point_cloud_mapping/geometry/areas.h"
#include "annotated_planar_patch_map/projection.h"
#include "annotated_planar_patch_map/annotated_map_lib.h"


using namespace annotated_planar_patch_map::projection;


/* !
 * \brief Project 3D map using stereoinfo projection matrix
 * 
 */

void annotated_planar_patch_map::projection::projectPolygonalMap(const sensor_msgs::StereoInfo& stereo_info, const mapping_msgs::PolygonalMap& transformed_map_3D, mapping_msgs::PolygonalMap &transformed_map_2D)
{

  //Get projections matrix
  //tf::Transform projection;
  //btScalar tmp_proj[16];
  double projection[16];
  for(int i=0;i<16;i++)
    projection[i]=stereo_info.RP[i];
  //tmp_proj[i]=stereo_info.RP[i];

  //projection.setFromOpenGLMatrix(tmp_proj);

  //Project all points of all polygons
  unsigned int num_polygons = transformed_map_3D.get_polygons_size();
  transformed_map_2D.set_polygons_size(num_polygons);
  for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
  {
    //create new polygon 2D (z=1)
    robot_msgs::Polygon3D newPoly;
    projectPolygonPoints(projection,double(stereo_info.width),double(stereo_info.height),transformed_map_3D.polygons[iPoly],newPoly);

    //put the polygon into 2D map
    transformed_map_2D.polygons[iPoly]=newPoly;
  }
}

void annotated_planar_patch_map::projection::projectAnyObject(const sensor_msgs::CamInfo& cam_info,robot_msgs::Polygon3D polyIn,robot_msgs::Polygon3D& polyOut)  
{
  //Projection setup
  CvMat *K_ = cvCreateMat(3, 3, CV_64FC1);
  memcpy((char*)(K_->data.db), (char*)(&cam_info.K[0]), 9 * sizeof(double));

  double zeros[] = {0, 0, 0};
  CvMat rotation_vector = cvMat(3, 1, CV_64FC1, zeros);
  CvMat translation_vector = cvMat(3, 1, CV_64FC1, zeros);
  double point[2] = {0.0, 0.0};


  //Input/output points
  unsigned int num_pts = polyIn.get_points_size();
  polyOut.set_points_size(num_pts);
  if(num_pts==0)
    return;

  CvMat* object_points = cvCreateMat( 1, num_pts , CV_64FC3 );
  CvMat* image_points = cvCreateMat( 1, num_pts , CV_64FC2 );

  for (unsigned int iP=0;iP<num_pts;iP++){
    CvPoint3D64f pt;
    pt.x=-polyIn.points[iP].y;
    pt.y=-polyIn.points[iP].z;
    pt.z= polyIn.points[iP].x;
    
    CV_MAT_ELEM( *object_points, CvPoint3D64f, 0, iP ) = pt;

  }
  //Project them
  cvProjectPoints2(object_points, &rotation_vector, &translation_vector,
                   K_, NULL, image_points);

  //Put them back
  for(unsigned int iPt = 0; iPt<num_pts; iPt++)
  {
    CvPoint2D64f &img_pt=CV_MAT_ELEM( *image_points, CvPoint2D64f, 0, iPt );

    robot_msgs::Point32 &old_pt=polyIn.points[iPt];
    robot_msgs::Point32 &new_pt=polyOut.points[iPt];
    new_pt.x=img_pt.x;
    new_pt.y=img_pt.y;
    new_pt.z=old_pt.x; //I mean, this is the "z" - distance from the image
  }

  cvReleaseMat(&K_);
  cvReleaseMat(&object_points);
  cvReleaseMat(&image_points);
}


void annotated_planar_patch_map::projection::projectAnyObject(const sensor_msgs::CamInfo& cam_info,const robot_msgs::PointCloud& source_3D,robot_msgs::PointCloud& target_2D)
{
  //Projection setup
  CvMat *K_ = cvCreateMat(3, 3, CV_64FC1);
  memcpy((char*)(K_->data.db), (char*)(&cam_info.K[0]), 9 * sizeof(double));

  double zeros[] = {0, 0, 0};
  CvMat rotation_vector = cvMat(3, 1, CV_64FC1, zeros);
  CvMat translation_vector = cvMat(3, 1, CV_64FC1, zeros);
  //double point[2] = {0.0, 0.0};


  //Input/output points
  unsigned int num_pts = source_3D.pts.size();
  target_2D.set_pts_size(num_pts);

  CvMat* object_points = cvCreateMat( 1, num_pts , CV_64FC3 );
  CvMat* image_points = cvCreateMat( 1, num_pts , CV_64FC2 );

  for (unsigned int iP=0;iP<num_pts;iP++){
    CvPoint3D64f pt;
    pt.x=-source_3D.pts[iP].y;
    pt.y=-source_3D.pts[iP].z;
    pt.z= source_3D.pts[iP].x;
    
    CV_MAT_ELEM( *object_points, CvPoint3D64f, 0, iP ) = pt;

  }
  //Project them
  cvProjectPoints2(object_points, &rotation_vector, &translation_vector,
                   K_, NULL, image_points);

  //Put them back
  for(unsigned int iPt = 0; iPt<num_pts; iPt++)
  {
    CvPoint2D64f &img_pt=CV_MAT_ELEM( *image_points, CvPoint2D64f, 0, iPt );

    const robot_msgs::Point32 &old_pt=source_3D.pts[iPt];
    robot_msgs::Point32 &new_pt=target_2D.pts[iPt];
    new_pt.x=img_pt.x;
    new_pt.y=img_pt.y;
    new_pt.z=old_pt.x; //I mean, this is the "z" - distance from the image
  }

  cvReleaseMat(&K_);
  cvReleaseMat(&object_points);
  cvReleaseMat(&image_points);
}












void annotated_planar_patch_map::projection::projectPolygonPoints(double* projection,double img_w,double img_h,robot_msgs::Polygon3D polyIn,robot_msgs::Polygon3D& polyOut)
{
  //Project all points of all polygons
  unsigned int num_pts = polyIn.get_points_size();
  polyOut.set_points_size(num_pts);
  for(unsigned int iPt = 0; iPt<num_pts; iPt++)
  {
    robot_msgs::Point32 &mpt=polyIn.points[iPt];
    tf::Vector3 pt(mpt.y,-mpt.z,mpt.x);
    //Vector3 projected_pt=projection * pt;
    
    robot_msgs::Point32 projected_pt;
    projected_pt.x=
      projection[0]*pt.x()/pt.z()+
      projection[4]*pt.y()/pt.z()+
      projection[8];

    projected_pt.y=
      projection[1]*pt.x()/pt.z()+
      projection[5]*pt.y()/pt.z()+
      projection[9];
    projected_pt.z=pt.z();
    //projection[2]*pt.x()/pt.z()+
    // projection[6]*pt.y()/pt.z()+
    // projection[10];

    robot_msgs::Point32 &new_pt=polyOut.points[iPt];
    if(projected_pt.z!=0)
    {
      new_pt.x= projected_pt.x;
      new_pt.y= projected_pt.y;
      new_pt.z = pt.z();
    }
    else
    {
      new_pt.x= 1e10;
      new_pt.y= 1e10;
    }
    //new_pt.x= new_pt.x*stereo_info_.width;
    //new_pt.y= new_pt.y*stereo_info_.height;	
    /*printf("\t\tProjected from (%f, %f, %f) to (%f, %f)\n",
      mpt.x,mpt.y,mpt.z,
      new_pt.x,new_pt.y);*/
  }
}

void annotated_planar_patch_map::projection::projectPolygonPointsNOP(double* projection,double img_w,double img_h,robot_msgs::Polygon3D polyIn,robot_msgs::Polygon3D& polyOut)
{
  //Project all points of all polygons
  unsigned int num_pts = polyIn.get_points_size();
  polyOut.set_points_size(num_pts);
  for(unsigned int iPt = 0; iPt<num_pts; iPt++)
  {
    robot_msgs::Point32 &mpt=polyIn.points[iPt];
    tf::Vector3 pt(-mpt.y,-mpt.z,mpt.x);
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


    new_pt.x= -mpt.x*(img_w/2) +img_w/2;
    new_pt.y= -mpt.z*(img_h/2) +img_h/2;
    //new_pt.x= new_pt.x*stereo_info_.width;
    //new_pt.y= new_pt.y*stereo_info_.height;	
    printf("\t\tProjected from (%f, %f, %f) to (%f, %f)\n",
      mpt.x,mpt.y,mpt.z,
      new_pt.x,new_pt.y);
  }
}

void annotated_planar_patch_map::projection::projectAnyObject(const sensor_msgs::StereoInfo& stereo_info, const annotated_map_msgs::TaggedPolygonalMap& transformed_map_3D, annotated_map_msgs::TaggedPolygonalMap &transformed_map_2D)
{
  bool bSame = (&transformed_map_3D == &transformed_map_2D);

  //Get projections matrix
  double projection[16];
  for(int i=0;i<16;i++)
    projection[i]=stereo_info.RP[i];


  //Project all points of all polygons
  unsigned int num_polygons = transformed_map_3D.get_polygons_size();
  transformed_map_2D.set_polygons_size(num_polygons);
  for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
  {
    //create new polygon 2D (z=1)
    robot_msgs::Polygon3D newPoly;
    projectPolygonPoints(projection,double(stereo_info.width),double(stereo_info.height),transformed_map_3D.polygons[iPoly].polygon,newPoly);

    //put the polygon into 2D map
    transformed_map_2D.polygons[iPoly].polygon=newPoly;
    if (!bSame)
    {
      transformed_map_2D.polygons[iPoly].tags=transformed_map_3D.polygons[iPoly].tags;
      transformed_map_2D.polygons[iPoly].tags_chan=transformed_map_3D.polygons[iPoly].tags_chan;
      transformed_map_2D.polygons[iPoly].name=transformed_map_3D.polygons[iPoly].name;
    }

  }
  if (! bSame )
  {
    transformed_map_2D.header = transformed_map_3D.header;
  }
}



void annotated_planar_patch_map::projection::projectAnyObjectNOP(const sensor_msgs::CamInfo& stereo_info, const annotated_map_msgs::TaggedPolygonalMap& transformed_map_3D, annotated_map_msgs::TaggedPolygonalMap &transformed_map_2D)
{
  bool bSame = (&transformed_map_3D == &transformed_map_2D);

  //Get projections matrix
  //tf::Transform projection;
  //btScalar tmp_proj[16];
  double projection[16];
  int i;
  for(i=0;i<12;i++)
    projection[i]=stereo_info.P[i];
  //for(i=0;i<16;i++)
  //    projection[i]=stereo_info.P[i];

  for(;i<16;i++)
    projection[i]=0;

  //tmp_proj[i]=stereo_info.RP[i];

  //projection.setFromOpenGLMatrix(tmp_proj);

  //Project all points of all polygons
  unsigned int num_polygons = transformed_map_3D.get_polygons_size();
  transformed_map_2D.set_polygons_size(num_polygons);
  for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
  {
    //create new polygon 2D (z=1)
    robot_msgs::Polygon3D newPoly;
    projectPolygonPointsNOP(projection,double(stereo_info.width),double(stereo_info.height),transformed_map_3D.polygons[iPoly].polygon,newPoly);

    //put the polygon into 2D map
    transformed_map_2D.polygons[iPoly].polygon=newPoly;
    if (!bSame)
    {
      transformed_map_2D.polygons[iPoly].tags=transformed_map_3D.polygons[iPoly].tags;
      transformed_map_2D.polygons[iPoly].tags_chan=transformed_map_3D.polygons[iPoly].tags_chan;
      transformed_map_2D.polygons[iPoly].name=transformed_map_3D.polygons[iPoly].name;
    }

  }
  if (! bSame )
  {
    transformed_map_2D.header = transformed_map_3D.header;
  }
}

void annotated_planar_patch_map::projection::projectAnyObject(const sensor_msgs::CamInfo& cam_info, const annotated_map_msgs::TaggedPolygonalMap& transformed_map_3D, annotated_map_msgs::TaggedPolygonalMap &transformed_map_2D)
{
  bool bSame = (&transformed_map_3D == &transformed_map_2D);

  //Project all points of all polygons
  unsigned int num_polygons = transformed_map_3D.get_polygons_size();
  transformed_map_2D.set_polygons_size(num_polygons);
  for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
  {
    //create new polygon 2D (z=1)
    robot_msgs::Polygon3D newPoly;

    projectAnyObject(cam_info,transformed_map_3D.polygons[iPoly].polygon,newPoly);

    //put the polygon into 2D map
    transformed_map_2D.polygons[iPoly].polygon=newPoly;
    if (!bSame)
    {
      transformed_map_2D.polygons[iPoly].tags=transformed_map_3D.polygons[iPoly].tags;
      transformed_map_2D.polygons[iPoly].tags_chan=transformed_map_3D.polygons[iPoly].tags_chan;
      transformed_map_2D.polygons[iPoly].name=transformed_map_3D.polygons[iPoly].name;
    }

  }
  if (! bSame )
  {
    transformed_map_2D.header = transformed_map_3D.header;
  }

}
  



bool annotated_planar_patch_map::projection::checkPolyInside(const robot_msgs::Polygon3D& poly,const std::vector<double>& viewport)
{
  unsigned int num_pts = poly.get_points_size();
  for(unsigned int iPt = 0; iPt<num_pts; iPt++)
  {
    const robot_msgs::Point32& pt=poly.points[iPt];

    if( (pt.x<viewport[0]) || (pt.x>=viewport[1]))
       return false;
    if( (pt.y<viewport[2]) ||(pt.y>=viewport[3]))
        return false;
    if(viewport.size()>4 && (pt.z<viewport[4] ||pt.z>=viewport[3]))
      return false;
  }
  return true;
}

std::vector<int> annotated_planar_patch_map::projection::getVisibleProjectedPolygons(const annotated_map_msgs::TaggedPolygonalMap& map,
                                             const std::vector<double>& viewport)
{

  double grid_scale=0.1;
  int nX=ceil(-viewport[0]/0.1);
  int nY=ceil(-viewport[0]/0.1);
  int oX=ceil(viewport[1]-viewport[0])/0.1;
  int oY=ceil(viewport[1]-viewport[0])/0.1;

  CvMat* depth_buffer=cvCreateMat(nX,nY,CV_32FC1);
  CvMat* depth_buffer_id=cvCreateMat(nX,nY,CV_32SC1);
  for(int i=0;i<nX;i++)
  {
    for(int j=0;j<nY;j++)
    {
      cvmSet(depth_buffer   ,i,j,  1e10);
      cvmSet(depth_buffer_id,i,j, -1);
    }
  }

  unsigned int num_polygons = map.get_polygons_size();
  for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
  {
    if(checkPolyInside(map.polygons[iPoly].polygon,viewport))
    {
      robot_msgs::Point32 center=annotated_map_lib::computeMean(map.polygons[iPoly].polygon);
      int cX=round(center.x/grid_scale)+oX;
      int cY=round(center.y/grid_scale)+oY;
      float d=cvmGet(depth_buffer,cX,cY);
      if(d>center.z)
      {
        cvmSet(depth_buffer,cX,cY,center.z);
        cvmSet(depth_buffer_id,cX,cY,iPoly);
      }
    }
  }
  std::vector<int> visible_polygons_by_id;
  visible_polygons_by_id.resize(num_polygons);
  for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
  {
    visible_polygons_by_id[iPoly]=-1;
  }
  int num_poly_out=0;
  for(int i=0;i<nX;i++)
  {
    for(int j=0;j<nY;j++)
    {
      int poly_id=cvmGet(depth_buffer_id,i,j);
      if(poly_id>-1)
      {
        if(visible_polygons_by_id[poly_id] == -1)
        {
          visible_polygons_by_id[poly_id] = poly_id;
          num_poly_out++;
        }        
      }
    }
  }

  std::vector<int> visible_polygons;
  visible_polygons.resize(num_poly_out);
  int i_poly_out;
  for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
  {
    if(visible_polygons_by_id[iPoly]!=-1)
    {
      visible_polygons[i_poly_out]=iPoly;
      i_poly_out++;
    }
  }

  cvReleaseMat(&depth_buffer);
  cvReleaseMat(&depth_buffer_id);

  return visible_polygons;
}
