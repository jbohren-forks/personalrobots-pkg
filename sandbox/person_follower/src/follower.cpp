#include <iostream>

#include <ros/ros.h>
#include <ros/common.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <costmap_2d/VoxelGrid.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <laser_scan/laser_scan.h>

using namespace std;

class Follower {
  tf::TransformListener tf_client_;
  ros::NodeHandle node_;
  ros::Publisher goal_pub_;
  ros::Publisher base_scan_goal_pub_;
  ros::Publisher vis_pub_;
  ros::Subscriber leg_detection_cloud_sub_;
  ros::Subscriber voxel_sub_;
  tf::MessageNotifier<geometry_msgs::PoseStamped> transformed_goal_note_;
  tf::MessageNotifier<sensor_msgs::PointCloud> transformed_cloud_note_;
  //tf::MessageNotifier<sensor_msgs::PointCloud> transformed_scan_note_;
  string global_frame_id;
  ros::Rate goalRate;
  //boost::shared_ptr<Follower> follower_object;
  //costmap_2d::Costmap2DROS cmROS;

  //Adds or modifies a visualization marker for the goal pose
  void visualizeGoalPose(const geometry_msgs::PoseStamped* const  msg)
  {
      cout<<"Sending vizualization message!"<<endl;
    visualization_msgs::Marker marker;
    marker.header = msg->header;
    marker.ns = "follower";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; //Add and update are the same enum
    marker.pose = msg->pose;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0.7;
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.3;

    ROS_INFO("setting viz: Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)\n",
	     marker.pose.position.x,
	     marker.pose.position.y,
	     marker.pose.position.z,
	     marker.pose.orientation.x,
	     marker.pose.orientation.y,
	     marker.pose.orientation.z,
	     marker.pose.orientation.w);
    
    vis_pub_.publish( marker );
  }

  //distance from point defined by x1,y1 and x2,y2 to the point x3,y3
  double pointLineDist(double x1, double y1, double x2, double y2, double x3, double y3)
  {
      double numerator = (y1-y2)*x3+(x2-x1)*y3+(x1*y2-x2*y1);
      double denom = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
      return abs(numerator/denom);
  }

  static const double freeSpaceThresh = 0.25;
  void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud)
  {
    cout<<"I heard a cloud with: "<<cloud->pts.size()<<" points."<<endl;
    double closestDistSq = 99999.0;
    int closestIdx = -1;
    sensor_msgs::PointCloud robotRelativeCloud;
    tf_client_.transformPointCloud("/base_footprint", *cloud, robotRelativeCloud);
    updateFreeSpaceVoxels();
    for(unsigned int i=0; i<robotRelativeCloud.pts.size(); i++)
      {
	const double &x = robotRelativeCloud.pts[i].x;
	const double &y = robotRelativeCloud.pts[i].y;
        if(true)//!isBelowFreeSpace(x,y))
        {
	    if(receivedTiltCloud)
	      {
			boost::mutex::scoped_lock lock(tiltMutex);
			for(unsigned int j=0; j<lastTiltScan.pts.size(); j++)
			  {
				if(pointLineDist(0,0,lastTiltScan.pts[j].x,lastTiltScan.pts[j].y,x,y) < freeSpaceThresh)
				  goto continueOuter; //As a rule GOTO considered harmful, however I like the ability to continue to arbitrary loops like java, since c++ lacks this syntax, I have this slight ugliness
			  }
	      }

            if(closestIdx == -1 || closestDistSq > x*x+y*y)
              {
                closestIdx = i;
                closestDistSq = x*x+y*y;
              }
        }
        else

        cout<<"\tFREE SPACE PT:";
        cout<<"\t["<<cloud->pts[i].x<<","<<cloud->pts[i].y<<","<<cloud->pts[i].z<<"]"<<endl;

        continueOuter: ;
      }

    cout<<"\tClosest is point #"<<closestIdx<<", heading towards it!"<<endl;
    const double &x = cloud->pts[closestIdx].x;
    const double &y = cloud->pts[closestIdx].y;
    //cout<<"\t\tLeg cloud is in: "<<cloud->header.frame_id<<endl;
    geometry_msgs::PointStamped in;
    in.header.frame_id = "/base_footprint";
    in.header.stamp = cloud->header.stamp;
    in.point.x = in.point.y = in.point.z = 0;
    geometry_msgs::PointStamped out;
    tf_client_.transformPoint("/odom_combined", in, out);
    cout<<"Zero is at: "<<out.point.x<<","<<out.point.y<<endl;
    setGoal(x, y, atan2(y-out.point.y,x-out.point.x), cloud->header.stamp);
  }

  bool receivedVoxels;
  boost::mutex voxelMutex;
  void voxelCallback(const costmap_2d::VoxelGridConstPtr& grid)
  {
      boost::mutex::scoped_lock lock(voxelMutex);

      receivedVoxels = true;
  }

  sensor_msgs::PointCloud lastTiltScan;
  bool receivedTiltCloud;
  boost::mutex tiltMutex;
  void tiltCloudCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
      boost::mutex::scoped_lock lock(tiltMutex);

      receivedTiltCloud = true;
      laser_scan::LaserProjection proj;
      proj.transformLaserScanToPointCloud("/base_footprint", lastTiltScan, *scan, tf_client_, 0, false);
  }

  void updateFreeSpaceVoxels()
  {

  }

  void transformedGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    geometry_msgs::PoseStamped tgoal;
    tf_client_.transformPose("/odom_combined", *goal, tgoal);
    cout<<"Received transformed goal: "<<endl;//<<tgoal<<endl;
    goalRate.sleep();
    goal_pub_.publish( tgoal );
  }

public:

  Follower() :
          transformed_goal_note_(tf_client_, boost::bind(&Follower::transformedGoalCallback, this, _1), "base_scan_goal", "/odom_combined", 1),
          transformed_cloud_note_(tf_client_, boost::bind(&Follower::cloudCallback, this, _1), "kalman_filt_cloud", "/base_footprint", 1),
          //transformed_scan_note_(tf_client_, boost::bind(&Follower::tiltCloudCallback, this, _1), "tilt_scan", "/base_footprint", 1),
          goalRate(0.1)//,
	  //          cmROS("follower_costmap", tf_client_)
  {
    goal_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/move_base_local/activate", 1);
    base_scan_goal_pub_ = node_.advertise<geometry_msgs::PoseStamped>("base_scan_goal", 1);
    vis_pub_ = node_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    receivedVoxels = false;
    //leg_detection_cloud_sub_ = node_.subscribe("particle_filt_cloud", 1, &Follower::cloudCallback, this);//follower_object);
    voxel_sub_ = node_.subscribe("voxel_grid", 1, &Follower::voxelCallback, this);//follower_object);
    node_.param("/global_frame_id", global_frame_id, string("/odom_combined"));
  }

  ~Follower()
  {
  }
    
private:

    
  void setGoal(double X, double Y, double angle, ros::Time captureTime)
  {
    //    tf::Stamped<tf::Pose> pRobot = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(angle, 0.0, 0.0), 
    //								  tf::Point(X, Y, 0.0)), 
    //							 captureTime,
    //							 "/odom_combined");
    
    tf::Pose posRobot = tf::Pose(tf::Quaternion(angle, 0.0, 0.0), tf::Point(X, Y, 0.0));
    
    //tf::Stamped<tf::Pose> p;
    //tf_client_.transformPose(global_frame_id, pRobot, p);
    
    geometry_msgs::PoseStamped goal;
    //goal.header.frame_id = "/odom_combined";
    goal.pose.position.x = posRobot.getOrigin().getX();
    goal.pose.position.y = posRobot.getOrigin().getY();
    goal.pose.position.z = posRobot.getOrigin().getZ();
    goal.pose.orientation.x = posRobot.getRotation().x();
    goal.pose.orientation.y = posRobot.getRotation().y();
    goal.pose.orientation.z = posRobot.getRotation().z();
    goal.pose.orientation.w = posRobot.getRotation().w();


    ROS_INFO("setting goal: Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", 
	     goal.pose.position.x, 
	     goal.pose.position.y, 
	     goal.pose.position.z, 
	     goal.pose.orientation.x, 
	     goal.pose.orientation.y, 
	     goal.pose.orientation.z, 
	     goal.pose.orientation.w, 
	     angle);
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "/odom_combined";

    visualizeGoalPose( &goal );
    base_scan_goal_pub_.publish( goal );
  }

public:
  void run()
  {
    ros::Rate loop_rate(5);
    while(node_.ok())
      {
	ros::spinOnce();
	loop_rate.sleep();
      }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, string("follower"));

  Follower follower;

  cout<<"Follower created!  Waiting for some TF stuff."<<endl;
  sleep(5);

  follower.run();

  return 0;
}
