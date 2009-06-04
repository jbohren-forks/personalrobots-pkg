
/* Author: Alexander Sorokin */


#define USAGE "USAGE: annotation2d_lifter_to_planar_patch_map \n"


#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <boost/numeric/ublas/matrix.hpp>
#include "ros/node.h"
#include "ros/publisher.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cv.h>


#include <robot_msgs/PolygonalMap.h>
#include <image_msgs/StereoInfo.h>

#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

using namespace std;
using namespace tf;


class EmptyAnnotatedMap : public ros::Node
{
  public:
    EmptyAnnotatedMap() : ros::Node("empty_annotated_map") {

      out_topic_name_=std::string("empty_poly_map");

      tf_ = new tf::TransformListener( *this, true);

      param( std::string("target_frame"), target_frame_, std::string("odom"));

      subscribe( std::string("planar_map"), unlabeled_map_, &EmptyAnnotatedMap::handleUnlabeledMap, 500);

      advertise<annotated_map_msgs::TaggedPolygonalMap>(out_topic_name_,1);
    };


  void handleUnlabeledMap()
  {

    try{
      annotated_map_msgs::TaggedPolygonalMap polymapOut;

      robot_msgs::PolygonalMap transformed_map_3D;
      transformPolygonalMap(target_frame_,unlabeled_map_,transformed_map_3D);


      unsigned int num_polygons = transformed_map_3D.get_polygons_size();
      if(num_polygons==0)
	{
	  return;
	}

      polymapOut.set_polygons_size(num_polygons);

      for(unsigned int iPoly = 0; iPoly<num_polygons; iPoly++)
	{
	  //create new tagged polygon
	  annotated_map_msgs::TaggedPolygon3D newPoly;
	  newPoly.set_tags_size(0);
	  newPoly.polygon=transformed_map_3D.polygons[iPoly];
	    
	  //append polygon to the map
	  polymapOut.polygons[iPoly]=newPoly;
	}
      polymapOut.header.frame_id=target_frame_;
      polymapOut.header.stamp=unlabeled_map_.header.stamp;

      publish( out_topic_name_, polymapOut);     

    }catch (TransformException& ex)
    {
      ROS_ERROR("Failure to transform detected object:: %s\n", ex.what());
    }


  };


  //From tf
void transformPolygonalMap(const std::string & target_frame, const robot_msgs::PolygonalMap & polymapIn, robot_msgs::PolygonalMap & polymapOut)
  {
  Stamped<Transform> transform;
  tf_->lookupTransform(target_frame, polymapIn.header.frame_id, polymapIn.header.stamp, transform);

  transformPolygonalMap(target_frame, transform, polymapIn.header.stamp, polymapIn, polymapOut);
};
void transformPolygonalMap(const std::string& target_frame, const ros::Time& target_time,
    const robot_msgs::PolygonalMap& polymapIn,
    const std::string& fixed_frame, robot_msgs::PolygonalMap& polymapOut)
{
  Stamped<Transform> transform;
  tf_->lookupTransform(target_frame, target_time,
      polymapIn.header.frame_id, polymapIn.header.stamp,
      fixed_frame,
      transform);

  transformPolygonalMap(target_frame, transform, target_time, polymapIn, polymapOut);


};

  
  void transformPolygonalMap(const std::string & target_frame, const Transform& net_transform, const ros::Time& target_time, const robot_msgs::PolygonalMap & polymapIn, robot_msgs::PolygonalMap & polymapOut)
  {
  boost::numeric::ublas::matrix<double> transform = transformAsMatrix(net_transform);


  typedef std::vector<robot_msgs::Polygon3D> poly_vec;

  bool bSame = (&polymapIn == &polymapOut);
  unsigned int num_polygons = polymapIn.get_polygons_size();
  if (!bSame)
    {
      polymapOut.set_polygons_size(polymapIn.get_polygons_size());
    }

  for(unsigned int iPoly=0;iPoly<num_polygons;iPoly++)  
  {    
    const robot_msgs::Polygon3D* p=&polymapIn.polygons[iPoly];

    unsigned int length = p->get_points_size();

    boost::numeric::ublas::matrix<double> matIn(4, length);

    //  double * matrixPtr = matIn.Store();

    for (unsigned int i = 0; i < length ; i++)
      {
	matIn(0,i) = p->points[i].x;
	matIn(1,i) = p->points[i].y;
	matIn(2,i) = p->points[i].z;
	matIn(3,i) = 1;
      };

    boost::numeric::ublas::matrix<double> matOut = prod(transform, matIn);

    robot_msgs::Polygon3D *polyOut;
    if (!bSame)
    {
      polyOut = &(polymapOut.polygons[iPoly]);
      polyOut->set_points_size(length);
      polyOut->color=p->color;
    }else{
      polyOut = &(polymapOut.polygons[iPoly]);
    }
    for (unsigned int i = 0; i < length ; i++)
    {
      polyOut->points[i].x = matOut(0,i);
      polyOut->points[i].y = matOut(1,i);
      polyOut->points[i].z = matOut(2,i);
    };
    
  }
  // Copy relevant data from polymapIn, if needed
  if (! bSame )
  {
    polymapOut.header = polymapIn.header;
    polymapOut.set_chan_size(polymapIn.get_chan_size());
    for (unsigned int i = 0 ; i < polymapIn.get_chan_size() ; ++i)
      polymapOut.chan[i] = polymapIn.chan[i];
  }

  //Override the positions
  polymapOut.header.stamp = target_time;
  polymapOut.header.frame_id = target_frame;

  };

  /** \brief Convert the transform to a Homogeneous matrix for large operations */
      static boost::numeric::ublas::matrix<double> transformAsMatrix(const Transform& 
      bt)
      {
	boost::numeric::ublas::matrix<double> outMat(4,4);

	double mv[12];
	bt.getBasis().getOpenGLSubMatrix(mv);
	
	Vector3 origin = bt.getOrigin();
	
	outMat(0,0)= mv[0];
	outMat(0,1)  = mv[4];
	outMat(0,2)  = mv[8];
	outMat(1,0)  = mv[1];
	outMat(1,1)  = mv[5];
	outMat(1,2)  = mv[9];
	outMat(2,0)  = mv[2];
	outMat(2,1)  = mv[6];
	outMat(2,2) = mv[10];
	
	outMat(3,0)  = outMat(3,1) = outMat(3,2) = 0;
	outMat(0,3) = origin.x();
	outMat(1,3) = origin.y();
	outMat(2,3) = origin.z();
	outMat(3,3) = 1;
	
	
	return outMat;
      };


protected:
  tf::TransformListener *tf_;

  robot_msgs::PolygonalMap unlabeled_map_;

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
  ros::init(argc, argv);

  try
  {
    EmptyAnnotatedMap map_converter;

    map_converter.spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  
  return 0;
}

