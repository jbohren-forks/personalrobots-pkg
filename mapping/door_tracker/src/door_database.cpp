//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

/**
@mainpage

@htmlinclude manifest.html

\author Sachin Chitta

@b door_tracker tracks a door that is in front of the robot

 **/

// ROS core
#include <ros/node.h>
// ROS messages

#include <geometry_msgs/Point.h>
#include <door_msgs/Door.h>
#include <visualization_msgs/Marker.h>
#include <angles/angles.h>

#include <sys/time.h>

#include <door_tracker/DoorDatabaseObject.h>
#include <door_tracker/CoeffArray.h>
#include <door_tracker/DoorQuery.h>

#include <Eigen/Array>
#include <Eigen/SVD>

#include <std_srvs/Empty.h>

#include <fstream>

#define HINGE_POINT_CONST 200
#define DOOR_TRACKER_MAX 20
using namespace std;
using namespace door_tracker;
using namespace Eigen;

class DoorDatabase
{
  public:

    ros::NodeHandle node_handle_;

    door_msgs::Door door_msg_;

    boost::mutex door_msg_mutex_ ;

    std::vector<DoorDatabaseObject> database_;

    ros::Publisher viz_pub_;

    ros::Subscriber door_sub_;

    ros::ServiceServer door_srv_; 

    ros::ServiceServer upload_srv_;

    ros::ServiceServer initialize_srv_;

    int hinge_number_;

    /*** Parameters to be updated from the param server ***/
    std::string door_msg_topic_;
    std::string door_service_name_;
    std::string upload_service_name_;
    std::string door_database_frame_;
    std::string initialize_database_service_name_;

    int min_angles_per_door_;
    double angle_difference_threshold_;
    double door_point_distance_threshold_;

    DoorDatabase()
    {
      node_handle_.param<std::string>("~p_door_msg_topic_", door_msg_topic_,"/door_tracker_node/door_message");
      node_handle_.param<std::string>("~door_database_frame", door_database_frame_,"map");
      node_handle_.param<int>("~p_min_angles_per_door",min_angles_per_door_, 4);
      node_handle_.param<double>("~p_angle_difference_threshold",angle_difference_threshold_,M_PI/12.0);
      node_handle_.param<double>("~p_door_point_distance_threshold",door_point_distance_threshold_,0.25);

      viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker",1);

      double tmp; int tmp2;

      node_handle_.param("~p_door_frame_p1_x", tmp, 0.5); 
      door_msg_.frame_p1.x = tmp;
      node_handle_.param("~p_door_frame_p1_y", tmp, -0.5); 
      door_msg_.frame_p1.y = tmp;
      node_handle_.param("~p_door_frame_p2_x", tmp, 0.5); 
      door_msg_.frame_p2.x = tmp;
      node_handle_.param("~p_door_frame_p2_y", tmp, 0.5); 
      door_msg_.frame_p2.y = tmp;
      node_handle_.param("~p_door_hinge" , tmp2, -1); 
      door_msg_.hinge = tmp2;
      node_handle_.param("~p_door_rot_dir" , tmp2, -1); 
      door_msg_.rot_dir = tmp2;

      node_handle_.param<std::string>("~door_service_name", door_service_name_,"~door_service");
      node_handle_.param<std::string>("~upload_service_name", upload_service_name_,"~upload_service");
      node_handle_.param<std::string>("~initialize_service_name", initialize_database_service_name_,"~initialize_service");

      door_msg_.header.frame_id = "base_link";
      door_sub_ = node_handle_.subscribe(door_msg_topic_,1,&DoorDatabase::doorMsgCallBack,this);
      door_srv_ = node_handle_.advertiseService(door_service_name_,&DoorDatabase::doorQuery,this);
      upload_srv_ = node_handle_.advertiseService(upload_service_name_,&DoorDatabase::uploadDatabase,this);
      initialize_srv_ = node_handle_.advertiseService(initialize_database_service_name_,&DoorDatabase::initializeDatabaseSrv,this);
      hinge_number_ = 0;
    };

    ~DoorDatabase()
    {
    }

    void doorMsgCallBack(const door_msgs::DoorConstPtr &door_msg_in)
    {
      door_msgs::Door door_msg = *door_msg_in;
      updateDatabase(door_msg);
    }

    void initializeDatabase()
    {
      int num_doors;
      node_handle_.param<int>("~number_doors",num_doors,0);
      if(num_doors <= 0)
      {
        ROS_INFO("No doors in database");
        return;
      }
      else
      {
        ROS_INFO("%d doors in database",num_doors);
      }
      door_msgs::Door door;
      for(int i=0; i<num_doors; i++)
      {
        char tmp_char[DOOR_TRACKER_MAX];
        sprintf(tmp_char,"%d",i);
        std::string key = node_handle_.getName() + "/door[" + std::string(tmp_char) + "]/";
        std::string value;
        if(!node_handle_.searchParam(key+"frame_id",value))
        {
          ROS_INFO("Could not find frame_id %s",(key+"frame_id").c_str());
          continue;
        }
        node_handle_.getParam(key+"frame_id",door.header.frame_id,"odom_combined");
        double tmp;
        node_handle_.getParam(key+"frame_p1/x",tmp); door.frame_p1.x = tmp;
        node_handle_.getParam(key+"frame_p1/y",tmp); door.frame_p1.y = tmp;
        node_handle_.getParam(key+"frame_p1/z",tmp); door.frame_p1.z = tmp;

        node_handle_.getParam(key+"frame_p2/x",tmp); door.frame_p2.x = tmp;
        node_handle_.getParam(key+"frame_p2/y",tmp); door.frame_p2.y = tmp;
        node_handle_.getParam(key+"frame_p2/z",tmp); door.frame_p2.z = tmp;

        node_handle_.getParam(key+"door_p1/x",tmp); door.door_p1.x = tmp;
        node_handle_.getParam(key+"door_p1/y",tmp); door.door_p1.y = tmp;
        node_handle_.getParam(key+"door_p1/z",tmp); door.door_p1.z = tmp;

        node_handle_.getParam(key+"door_p2/x",tmp); door.door_p2.x = tmp;
        node_handle_.getParam(key+"door_p2/y",tmp); door.door_p2.y = tmp;
        node_handle_.getParam(key+"door_p2/z",tmp); door.door_p2.z = tmp;

        node_handle_.getParam(key+"handle/x",tmp); door.handle.x = tmp;
        node_handle_.getParam(key+"handle/y",tmp); door.handle.y = tmp;
        node_handle_.getParam(key+"handle/z",tmp); door.handle.z = tmp;

        node_handle_.getParam(key+"handle/hinge",door.hinge);
        node_handle_.getParam(key+"handle/rot_dir",door.rot_dir);
        addToDatabase(database_,door);
        ROS_DEBUG("Loaded door information from database");
      }
    }

    geometry_msgs::Point32 findHingePosition(door_tracker::DoorDatabaseObject &db)
    {
      double x1(0.0),y1(0.0);

      x1 = (db.door.door_p1.x+db.door.door_p2.x)/2.0;
      y1 = (db.door.door_p1.y+db.door.door_p2.y)/2.0;

      geometry_msgs::Point32 hinge;
      MatrixXf lhs;
      lhs = MatrixXf::Zero((int) db.angles.size(),3);
      for(int i=0; i< (int) db.angles.size(); i++)
      {
//        db.angles[i] = angles::normalize_angle(db.angles[i]-db.angles[0]);
        lhs(i,0) = db.coeff[i].data[0]*cos(db.angles[i])-db.coeff[i].data[0]+db.coeff[i].data[1]*sin(db.angles[i]);
        lhs(i,1) = -db.coeff[i].data[0]*sin(db.angles[i])-db.coeff[i].data[1]+db.coeff[i].data[1]*cos(db.angles[i]);
        lhs(i,2) = -x1*db.coeff[i].data[0]*cos(db.angles[i]) + y1*db.coeff[i].data[0]*sin(db.angles[i])-y1*db.coeff[i].data[1]*cos(db.angles[i])-x1*db.coeff[i].data[1]*sin(db.angles[i]);
        ROS_DEBUG(" Matrix elements are: %f %f %f %f",lhs(i,0),lhs(i,1),lhs(i,2),db.angles[i]);
      }

      VectorXf rhs;
      rhs = VectorXf::Zero((int) db.angles.size());
      for(int i=0; i< (int) db.angles.size(); i++)
      {
        rhs(i) = db.coeff[i].data[2];
      }

      VectorXf sol = VectorXf::Zero(3);
      ROS_DEBUG("lhs is a %d x 3 matrix, rhs is a %d vector",(int) db.angles.size(), (int) db.angles.size());
      lhs.svd().solve(rhs,&sol);
      hinge.x = sol(0);
      hinge.y = sol(1);
      hinge.z = 0;

      ROS_DEBUG("Setting hinge at: %f %f %f",hinge.x,hinge.y,hinge.z);
      publishPoint(hinge,HINGE_POINT_CONST+hinge_number_,db.door.header.frame_id);
      hinge_number_++;
      return hinge;
    }

    door_tracker::CoeffArray generateLinearCoeff(const geometry_msgs::Point32 p1, const geometry_msgs::Point32 p2)
    {
      door_tracker::CoeffArray cf;
      cf.set_data_size(3);
      double dx = p2.x - p1.x;
      double dy = p2.y - p1.y;
      cf.data[0] =  -dy;
      cf.data[1] = dx;
      cf.data[2] = p1.x*dy-p1.y*dx;

      return cf;
    }

    void updateDatabase(const door_msgs::Door &door)
    {
      // (a) check if it's already in the database
      // (b) Add it in if it's not
      if(!inDatabase(database_,door))
        addToDatabase(database_,door);
    }

    void addToDatabase(std::vector<door_tracker::DoorDatabaseObject> &database, const door_msgs::Door &door)
    {
      database.resize(database.size()+1);

      int index = database.size()-1;
      database[index].door = door;
      database[index].door.hinge = door_msgs::Door::HINGE_P1;

      double dx = door.door_p2.x - door.door_p1.x;
      double dy = door.door_p2.y - door.door_p1.y;
      double door_angle = atan2(dy,dx);

      database[index].set_angles_size(1);
      database[index].angles[0] = door_angle;
      database[index].set_coeff_size(1);
      database[index].coeff[0] = generateLinearCoeff(door.door_p1,door.door_p2);
      ROS_DEBUG("Adding new door to database: p1: %f, %f, %f, p2: %f, %f, %f",door.door_p1.x,door.door_p1.y,door.door_p1.z,door.door_p2.x,door.door_p2.y,door.door_p2.z);
      ROS_DEBUG("There are %d candidate doors in the database now.",index+1);
    }

    double distance(const geometry_msgs::Point32 &d1, const geometry_msgs::Point32 &d2)
    {
      double dist = sqrt(pow(d1.x - d2.x,2) + pow(d1.y - d2.y,2));
      return dist;
    }

    void updateDoorInfo(door_tracker::DoorDatabaseObject &db, const door_msgs::Door door, int index_p1, int index_p2)
    {
      double dx = door.door_p2.x - door.door_p1.x;
      double dy = door.door_p2.y - door.door_p1.y;

      if(index_p2 == door_msgs::Door::HINGE_P2)
      {
        dx = -dx;
        dy = -dy;
      }
      double new_door_angle = atan2(dy,dx);

      bool different_angle = true;

      if((int) db.angles.size() > 0)
      {
        for(int i=0; i< (int) db.angles.size(); i++)
        {
          if(fabs(angles::normalize_angle(new_door_angle - db.angles[i])) < angle_difference_threshold_)
          {
            different_angle = false;
            break;
          }
        }
      }

      if(different_angle && (int) db.angles.size() < min_angles_per_door_)
      {
        db.door.weight = (double)db.angles.size()/(double) min_angles_per_door_;
        if(db.angles.size() == 1)
        {
          //check the hinge point
          if(index_p1 != db.door.hinge)
          {
            ROS_DEBUG("Switching angles since index_p1: %d does not match: %d",index_p1,db.door.hinge);
            db.door.hinge = index_p1;
            db.angles[0] = angles::normalize_angle(db.angles[0]+M_PI);
          }
        }

        db.angles.resize(db.angles.size()+1);
        ROS_DEBUG("Door has a different angle: %f \nNumber of angles for door is now %d", new_door_angle, (int) db.angles.size());
        db.coeff.resize(db.coeff.size()+1);
        db.angles[db.angles.size()-1] = new_door_angle;
        db.coeff[db.coeff.size()-1] = generateLinearCoeff(door.door_p1,door.door_p2);
        if((int) db.angles.size() == min_angles_per_door_)
        {
          db.door.frame_p1 = findHingePosition(db);
          db.door.hinge = door_msgs::Door::HINGE_P1;
//          publishDoors();
        }
      }
    }

    bool inDatabase(std::vector<door_tracker::DoorDatabaseObject> &database, const door_msgs::Door &door)
    {
      int index_p1 = -1;
      int index_p2 = -1;
      for(int i=0; i < (int) database.size(); i++)
      {
        if(sameDoor(database[i].door,door,index_p1,index_p2))
        {
          updateDoorInfo(database[i],door,index_p1,index_p2);
          ROS_DEBUG("Adding new point to door[%d] p1: %f, %f, %f, p2: %f, %f, %f",i,door.door_p1.x,door.door_p1.y,door.door_p1.z,door.door_p2.x,door.door_p2.y,door.door_p2.z);
          ROS_DEBUG("Door is in the database with index %d",i);
          return true;
        }
      }
      ROS_INFO("Door is not in the database");
      return false;
    }

    void publishPoint(const geometry_msgs::Point32 &point, const int &id, const std::string &frame_id)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time();
      marker.ns = "door_database";
      marker.id = id;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = point.x;
      marker.pose.position.y = point.y;
      marker.pose.position.z = point.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.g = 1.0;
//      ROS_DEBUG("Publishing line between: p1: %f %f %f, p2: %f %f %f",marker.points[0].x,marker.points[0].y,marker.points[0].z,marker.points[1].x,marker.points[1].y,marker.points[1].z);

      viz_pub_.publish(marker);
    }

    void publishDoors()
    {
      for(int i=0; i < (int) database_.size(); i++)
      {
        if((int) database_[i].angles.size() == min_angles_per_door_)
        {
          publishPoint(database_[i].door.frame_p1,i+HINGE_POINT_CONST,database_[i].door.header.frame_id);
          ROS_DEBUG("Publishing hinge point (%f,%f,%f) in frame id %s",database_[i].door.frame_p1.x,database_[i].door.frame_p1.y,database_[i].door.frame_p1.z,database_[i].door.header.frame_id.c_str());
        }
      }
    }

    bool sameDoor(const door_msgs::Door &door1, const door_msgs::Door &door2, int &hinge_index_p1, int &hinge_index_p2)
    {
      if(distance(door1.door_p1,door2.door_p1) < door_point_distance_threshold_)
      {
        hinge_index_p1 = door_msgs::Door::HINGE_P1;
        hinge_index_p2 = door_msgs::Door::HINGE_P1;
        return true;
      }
      else if(distance(door1.door_p1,door2.door_p2) < door_point_distance_threshold_)
      {
        hinge_index_p1 = door_msgs::Door::HINGE_P1;
        hinge_index_p2 = door_msgs::Door::HINGE_P2;
        return true;
      }
      else if(distance(door1.door_p2,door2.door_p1) < door_point_distance_threshold_)
      {
        hinge_index_p1 = door_msgs::Door::HINGE_P2;
        hinge_index_p2 = door_msgs::Door::HINGE_P1;
        return true;
      }
      else if(distance(door1.door_p2,door2.door_p2) < door_point_distance_threshold_)
      {
        hinge_index_p1 = door_msgs::Door::HINGE_P2;
        hinge_index_p2 = door_msgs::Door::HINGE_P2;
        return true;
      }
      return false;
    }

    bool findNearestDoor(const door_msgs::Door &door_in, door_msgs::Door &door)
    {   
      int h1,h2;
      if(database_.empty())
        return false;
      std::vector<std::pair<double,int> > distance_pair;
      for(int i=0; i < (int) database_.size(); i++)
      {
        distance_pair.push_back(std::make_pair(distance(database_[i].door.frame_p1,door_in.frame_p1),i));
        distance_pair.push_back(std::make_pair(distance(database_[i].door.frame_p1,door_in.frame_p2),i));
        distance_pair.push_back(std::make_pair(distance(database_[i].door.frame_p2,door_in.frame_p1),i));
        distance_pair.push_back(std::make_pair(distance(database_[i].door.frame_p2,door_in.frame_p2),i));
      }
      std::sort(distance_pair.begin(),distance_pair.end());
      if(sameDoor(door_in,database_[0].door,h1,h2))
      {
        door = database_[0].door;
        return true;
      }
      return false;
    }

    bool doorQuery(door_tracker::DoorQuery::Request &request, door_tracker::DoorQuery::Response &response)
    {
      door_msgs::Door door;
      if(findNearestDoor(request.door,door))
      {
        response.door = door;
        return true;
      }
      return false;
    }

    bool initializeDatabaseSrv(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
      initializeDatabase();
      return true;
    }


    bool uploadDatabase(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
      if(database_.empty())
      {
        ROS_ERROR("No entries in the database");
        return false;
      }

      node_handle_.setParam("~number_doors",(int)database_.size());
      for(unsigned int i=0; i < database_.size(); i++)
      {
        char tmp_char[DOOR_TRACKER_MAX];
        sprintf(tmp_char,"%d",i);
        std::string key = "~door[" + std::string(tmp_char) + "]/";
        node_handle_.setParam(key+"frame_id",database_[i].door.header.frame_id);

        node_handle_.setParam(key+"frame_p1/x",database_[i].door.frame_p1.x);
        node_handle_.setParam(key+"frame_p1/y",database_[i].door.frame_p1.y);
        node_handle_.setParam(key+"frame_p1/z",database_[i].door.frame_p1.z);

        node_handle_.setParam(key+"frame_p2/x",database_[i].door.frame_p2.x);
        node_handle_.setParam(key+"frame_p2/y",database_[i].door.frame_p2.y);
        node_handle_.setParam(key+"frame_p2/z",database_[i].door.frame_p2.z);

        node_handle_.setParam(key+"door_p1/x",database_[i].door.door_p1.x);
        node_handle_.setParam(key+"door_p1/y",database_[i].door.door_p1.y);
        node_handle_.setParam(key+"door_p1/z",database_[i].door.door_p1.z);

        node_handle_.setParam(key+"door_p2/x",database_[i].door.door_p2.x);
        node_handle_.setParam(key+"door_p2/y",database_[i].door.door_p2.y);
        node_handle_.setParam(key+"door_p2/z",database_[i].door.door_p2.z);

        node_handle_.setParam(key+"handle/x",database_[i].door.handle.x);
        node_handle_.setParam(key+"handle/y",database_[i].door.handle.y);
        node_handle_.setParam(key+"handle/z",database_[i].door.handle.z);

        node_handle_.setParam(key+"handle/hinge",database_[i].door.hinge);
        node_handle_.setParam(key+"handle/rot_dir",database_[i].door.rot_dir);
      }
      return true;
    }
};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "door_database");
  DoorDatabase p;
  ROS_INFO("Started door database node");
  ros::spin();
  return (0);
}
