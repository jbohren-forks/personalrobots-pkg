#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include "ros/node.h"
#include "joy/Joy.h"
#include "geometry_msgs/PoseDot.h"


using namespace ros;

class TeleopBase : public Node
{
   public:
  geometry_msgs::PoseDot cmd, cmd_passthrough;
  joy::Joy joy;
  double req_vx, req_vy, req_vw;
  double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
  int axis_vx, axis_vy, axis_vw;
  int deadman_button, run_button, passthrough_button;
  bool deadman_no_publish_;
  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;

  TeleopBase(bool deadman_no_publish = false) : Node("teleop_base"), max_vx(0.6), max_vy(0.6), max_vw(0.8), max_vx_run(0.6), max_vy_run(0.6), max_vw_run(0.8), deadman_no_publish_(deadman_no_publish)
      {
        cmd.vel.vx = cmd.vel.vy = cmd.ang_vel.vz = 0;
        if (!hasParam("max_vx") || !getParam("max_vx", max_vx))
          ROS_WARN("maximum linear velocity (max_vx) not set. Assuming 0.6");
        if (!hasParam("max_vy") || !getParam("max_vy", max_vy))
          ROS_WARN("maximum linear velocity (max_vy) not set. Assuming 0.6");
        if (!hasParam("max_vw") || !getParam("max_vw", max_vw))
          ROS_WARN("maximum angular velocity (max_vw) not set. Assuming 0.8");
        
        // Set max speed while running
        if (!hasParam("max_vx_run") || !getParam("max_vx_run", max_vx_run))
          ROS_WARN("maximum running linear velocity (max_vx_run) not set. Assuming 0.6");
        if (!hasParam("max_vy_run") || !getParam("max_vy_run", max_vy_run))
          ROS_WARN("maximum running linear velocity (max_vy_run) not set. Assuming 0.6");
        if (!hasParam("max_vw_run") || !getParam("max_vw_run", max_vw_run))
          ROS_WARN("maximum running angular velocity (max_vw_run) not set. Assuming 0.8");

        param<int>("axis_vx", axis_vx, 3);
        param<int>("axis_vw", axis_vw, 0);
        param<int>("axis_vy", axis_vy, 2);
        
        param<int>("deadman_button", deadman_button, 0);
        param<int>("run_button", run_button, 0);
        param<int>("passthrough_button", passthrough_button, 1);

	double joy_msg_timeout;
        param<double>("joy_msg_timeout", joy_msg_timeout, -1.0); //default to no timeout
	if (joy_msg_timeout <= 0)
	  {
	    joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
	    ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
	  }
	else
	  {
	    joy_msg_timeout_.fromSec(joy_msg_timeout);
	    ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
	  }

        ROS_DEBUG("max_vx: %.3f m/s\n", max_vx);
        ROS_DEBUG("max_vy: %.3f m/s\n", max_vy);
        ROS_DEBUG("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);
        
        ROS_DEBUG("max_vx_run: %.3f m/s\n", max_vx_run);
        ROS_DEBUG("max_vy_run: %.3f m/s\n", max_vy_run);
        ROS_DEBUG("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);
        
        ROS_DEBUG("axis_vx: %d\n", axis_vx);
        ROS_DEBUG("axis_vy: %d\n", axis_vy);
        ROS_DEBUG("axis_vw: %d\n", axis_vw);
        
        ROS_DEBUG("deadman_button: %d\n", deadman_button);
        ROS_DEBUG("run_button: %d\n", run_button);
        ROS_DEBUG("passthrough_button: %d\n", passthrough_button);
        ROS_DEBUG("joy_msg_timeout: %f\n", joy_msg_timeout);
        
        advertise<geometry_msgs::PoseDot>("cmd_vel", 1);
        subscribe("joy", joy, &TeleopBase::joy_cb, 1);
        subscribe("cmd_passthrough", cmd_passthrough, &TeleopBase::passthrough_cb, 1);
        ROS_DEBUG("done with ctor\n");
      }
  
  ~TeleopBase()
  {
    unsubscribe("joy");
    unsubscribe("cmd_passthrough");
    unadvertise("cmd_vel");

  }

      void joy_cb()
      {
	//Record this message reciept
	last_recieved_joy_message_time_ = ros::Time::now();
        
        // Base
        bool running = (((unsigned int)run_button < joy.get_buttons_size()) && joy.buttons[run_button]);
        double vx = running ? max_vx_run : max_vx;
        double vy = running ? max_vy_run : max_vy;
        double vw = running ? max_vw_run : max_vw;

         if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy.get_axes_size()))
            req_vx = joy.axes[axis_vx] * vx;
         else
            req_vx = 0.0;
         if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy.get_axes_size()))
            req_vy = joy.axes[axis_vy] * vy;
         else
            req_vy = 0.0;
         if((axis_vw >= 0) && (((unsigned int)axis_vw) < joy.get_axes_size()))
            req_vw = joy.axes[axis_vw] * vw;
         else
            req_vw = 0.0;

      }
      void passthrough_cb() { }
      void send_cmd_vel()
      {
         joy.lock();
         if(((deadman_button < 0) ||
            ((((unsigned int)deadman_button) < joy.get_buttons_size()) &&
             joy.buttons[deadman_button]))
	    &&
	    last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
         {
            if (passthrough_button >= 0 &&
                passthrough_button < (int)joy.get_buttons_size() &&
                joy.buttons[passthrough_button])
            {
               // pass through commands that we have received (e.g. from wavefront)
               cmd_passthrough.lock();
               cmd = cmd_passthrough;
               cmd_passthrough.unlock();
            }
            else
            {
               // use commands from the local sticks
               cmd.vel.vx = req_vx;
               cmd.vel.vy = req_vy;
               cmd.ang_vel.vz = req_vw;
            }
            publish("cmd_vel", cmd);

            fprintf(stderr,"teleop_base:: %f, %f, %f\n",cmd.vel.vx,cmd.vel.vy,cmd.ang_vel.vz);
         }
         else
         {
           cmd.vel.vx = cmd.vel.vy = cmd.ang_vel.vz = 0;
           if (!deadman_no_publish_)
           {
             publish("cmd_vel", cmd);//Only publish if deadman_no_publish is enabled

           }
         }
         joy.unlock();
      }
};

int main(int argc, char **argv)
{
   ros::init(argc, argv);
   const char* opt_no_publish    = "--deadman_no_publish";

   bool no_publish = false;
   for(int i=1;i<argc;i++)
   {
     if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
       no_publish = true;
   }
   TeleopBase teleop_base(no_publish);
   while (teleop_base.ok())
   {
      usleep(50000);
      teleop_base.send_cmd_vel();
   }
   
   exit(0);
   return 0;
}

