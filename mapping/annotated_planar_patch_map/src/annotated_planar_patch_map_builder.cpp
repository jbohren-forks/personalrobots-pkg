
/* Author: Alexander Sorokin */


#define USAGE "USAGE: annotated_planar_patch_map_builder \n"


#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "ros/node.h"
#include "ros/publisher.h"
#include <tf/transform_listener.h>
#include "robot_msgs/PolygonalMap.h"
#include <annotated_map_msgs/TaggedPolygonalMap.h>
#include <annotated_map_msgs/TaggedPolygon3D.h>

using namespace std;
using namespace tf;

class AnnotatedMapBuilder : public ros::Node
{
  public:
    AnnotatedMapBuilder() : ros::Node("annotated_map_builder") {

      tf_ = new tf::TransformListener( *this, true, (uint64_t)10000000000000ULL);

      param( std::string("target_frame"), target_frame_, std::string("map"));
      param( std::string("broadcast_set"), broadcast_set_, std::string("door,outlet,table,chair"));

      param( std::string("broadcast_freq"), broadcast_freq_, (uint64_t)10000000000000ULL);

      subscribe( std::string("annotated_map"), polymap_in_, &ObjectCollector::handlePolygon, 500);
      
      advertise_maps();
    };

  void advertise_maps(){
    
    advertise<robot_msgs::PolygonalMap>(std::string("global_object_map"),1);
  }
  
  void handlePolygon(){
    try{

      printf("Polygon\n");
      robot_msgs::PolygonalMap polymapOut;
      transformPolygonalMap(target_frame_, poly_object_, polymapOut);

      add_to_map(polymapOut,global_map_);
    
      global_map_.header.frame_id=target_frame_;
      global_map_.header.stamp = poly_object_.header.stamp;
      global_map_.header.seq ++;

      publish( std::string("global_object_map"), global_map_);     
    }catch (TransformException& ex)
    {
      ROS_ERROR("Failure to transform detected object:: %s\n", ex.what());
    }


  };

  void add_to_map(robot_msgs::PolygonalMap newMap,robot_msgs::PolygonalMap& output_map)
  {
    unsigned int num_polygons_to_add = newMap.get_polygons_size();
    unsigned int old_num_polygons = output_map.get_polygons_size();
    output_map.set_polygons_size(old_num_polygons+num_polygons_to_add);

    for(unsigned int iPoly = 0; iPoly<num_polygons_to_add; iPoly++)
    {
      output_map.polygons[iPoly+old_num_polygons]=newMap.polygons[iPoly];
    }
  }



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



protected:
  //Our parameters
  std::string target_frame_;

  //Our message inputs
  tf::TransformListener *tf_;

  annotated_map_msgs::TaggedPolygonalMap polymap_in_;

  //Node internal storage
  // 1. list of all annotated maps
  // 2. maps per keywords




};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  try
  {
    ObjectCollector oc;

    oc.spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  
  return 0;
}

