
#include <ros/node.h>
#include <geometry_msgs/PoseStamped.h>
#include <manipulation_srvs/IKService.h>

#include <pr2_ik/FKService.h>


double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

bool computeIK(ros::ServiceClient client, const geometry_msgs::PoseStamped &pose_stamped_msg, std::vector<double> &solution)
{
  // define the service messages
  manipulation_srvs::IKService::Request request;
  manipulation_srvs::IKService::Response response;
	
  request.data.pose_stamped = pose_stamped_msg;

  request.data.joint_names.resize(7);
  request.data.joint_names[0] = "r_shoulder_pan_joint";
  request.data.joint_names[1] = "r_shoulder_lift_joint";
  request.data.joint_names[2] = "r_upper_arm_roll_joint";
  request.data.joint_names[3] = "r_elbow_flex_joint";
  request.data.joint_names[4] = "r_forearm_roll_joint";
  request.data.joint_names[5] = "r_wrist_flex_joint";
  request.data.joint_names[6] = "r_wrist_roll_joint";

  request.data.positions.resize(7);
  srand(time(NULL));
  request.data.positions[2] = gen_rand(-M_PI,M_PI);

  if (client.call(request, response))
  { 
    ROS_DEBUG("Obtained IK solution");
    solution = response.solution;
    if (solution.size() != request.data.positions.size())
    {
      ROS_ERROR("Incorrect number of elements in IK output");
      return false;
    }
    for(unsigned int i = 0; i < solution.size() ; ++i)
      ROS_DEBUG("IK[%d] = %f", (int)i, solution[i]);
  }
  else
  {
    ROS_ERROR("IK service failed");
    return false;
  }
  return true;
}

int main( int argc, char** argv )
{
  /*********** Initialize ROS  ****************/
  ros::init(argc,argv,"ik_node_client");
  ros::spinOnce();
//  ros::Node *node = new ros::Node("ik_node_client"); 

  ros::NodeHandle node_handle_;

  ros::ServiceClient ik_client = node_handle_.serviceClient<manipulation_srvs::IKService>("/pr2_ik_right_arm/ik_service", true);

  std::vector<double> solution;

  geometry_msgs::PoseStamped p;

  p.header.frame_id = "torso_lift_link";
  p.pose.position.x = 0.518015851998;
  p.pose.position.y = 0.493807790311;
  p.pose.position.z = 0.00977711480488;

  p.pose.orientation.x = 0.499945894949;
  p.pose.orientation.y = 0.500054099197;
  p.pose.orientation.z = 0.500054099197;
  p.pose.orientation.w = 0.499945894949;

  for(int i=0; i<10; i++)
  {
    computeIK(ik_client,p,solution);
    ros::ServiceClient fk_client = node_handle_.serviceClient<pr2_ik::FKService>("pr2_ik_right_arm/fk_service", true);
    pr2_ik::FKService::Request  req;
    pr2_ik::FKService::Response res;

    req.header.frame_id = "torso_lift_link";
    req.header.stamp = ros::Time();
    req.set_angles_vec(solution);
    fk_client.call(req,res);

    double x_e = fabs(res.pose.pose.position.x-p.pose.position.x);
    double y_e = fabs(res.pose.pose.position.y-p.pose.position.y);
    double z_e = fabs(res.pose.pose.position.z-p.pose.position.z);

    ROS_DEBUG("FK: %f %f %f",res.pose.pose.position.x,res.pose.pose.position.y,res.pose.pose.position.z);
    ROS_DEBUG("Error: %f, %f, %f",x_e, y_e, z_e);
    if(x_e > 1e-4 || y_e > 1e-4 || z_e > 1e-3)
    {
      ROS_ERROR("IK incorrect");
      ROS_INFO("Error: %f %f %f",x_e,y_e,z_e);
      for(int j=0; j < req.angles.size(); j++)
        ROS_INFO("Angles: %f",req.angles[j]);
      break;
    }
  }
  ROS_INFO("IK passed test");
}
