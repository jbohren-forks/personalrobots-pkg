#include <iostream>

#include <ros/ros.h>
#include <ros/common.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <robot_msgs/PointCloud.h>

using namespace std;

class Follower {
  tf::TransformListener tf_client_;
  ros::NodeHandle node_;
  ros::Publisher goal_pub_;
  ros::Publisher base_scan_goal_pub_;
  ros::Subscriber leg_detection_cloud_sub_;
  tf::MessageNotifier<robot_msgs::PoseStamped> transformed_goal_note_;
  string global_frame_id;
  ros::Rate goalRate;
  //boost::shared_ptr<Follower> follower_object;

  void cloudCallback(const robot_msgs::PointCloud::ConstPtr& cloud)
  {
    cout<<"I heard a cloud with: "<<cloud->pts.size()<<" points."<<endl;
    double closestDistSq = 99999.0;
    int closestIdx = -1;
    for(unsigned int i=0; i<cloud->pts.size(); i++)
      {
	const double &x = cloud->pts[i].x;
	const double &y = cloud->pts[i].y;
	const double &z = cloud->pts[i].z;
	if(closestIdx == -1 || closestDistSq > x*x+y*y+z*z)
	  {
	    closestIdx = i;
	    closestDistSq = x*x+y*y+z*z;
	  }
	
	cout<<"\t["<<cloud->pts[i].x<<","<<cloud->pts[i].y<<","<<cloud->pts[i].z<<"]"<<endl;
      }

    cout<<"\tClosest is point #"<<closestIdx<<", heading towards it!"<<endl;
    const double &x = cloud->pts[closestIdx].x;
    const double &y = cloud->pts[closestIdx].y;
    setGoal(x, y, atan2(y, x));
  }

  void transformedGoalCallback(const robot_msgs::PoseStamped::ConstPtr& goal)
  {
    robot_msgs::PoseStamped tgoal;
    tf_client_.transformPose("/map", *goal, tgoal);
    cout<<"Received transformed goal: "<<endl;//<<tgoal<<endl;
    goalRate.sleep();
    goal_pub_.publish( tgoal );
  }

public:

  Follower() : transformed_goal_note_(tf_client_, boost::bind(&Follower::transformedGoalCallback, this, _1), "base_scan_goal", "/map", 1), goalRate(0.2) 
  {
    goal_pub_ = node_.advertise<robot_msgs::PoseStamped>("/move_base/activate", 1);
    base_scan_goal_pub_ = node_.advertise<robot_msgs::PoseStamped>("base_scan_goal", 1);
    leg_detection_cloud_sub_ = node_.subscribe("particle_filt_cloud", 1, &Follower::cloudCallback, this);//follower_object);
    node_.param("/global_frame_id", global_frame_id, string("/map"));
  }

  ~Follower()
  {
  }
    
private:

    
  void setGoal(double X, double Y, double angle)
  {
    tf::Stamped<tf::Pose> pRobot = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(angle, 0.0, 0.0), 
								  tf::Point(X, Y, 0.0)), 
							 ros::Time::now(), 
							 "/base_laser");
    
    
    //tf::Stamped<tf::Pose> p;
    //tf_client_.transformPose(global_frame_id, pRobot, p);
    
    robot_msgs::PoseStamped goal;
    tf::PoseStampedTFToMsg(pRobot, goal);
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
    goal.header.frame_id = "/base_laser";
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
