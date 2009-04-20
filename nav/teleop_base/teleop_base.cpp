#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include "ros/node.h"
#include "joy/Joy.h"
#include "robot_msgs/PoseDot.h"
#include "std_msgs/Float64.h"

#define TORSO_TOPIC "/torso_lift_controller/command"

using namespace ros;

class TeleopBase : public Node
{
   public:
  robot_msgs::PoseDot cmd, cmd_passthrough;
  std_msgs::Float64 torso_eff;
  joy::Joy joy;
  double req_vx, req_vy, req_vw, max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run, req_torso;
  int axis_vx, axis_vy, axis_vw;
  int deadman_button, run_button, torso_dn_button, torso_up_button, passthrough_button;
  bool deadman_no_publish_;

  TeleopBase(bool deadman_no_publish = false) : Node("teleop_base"), max_vx(0.6), max_vy(0.6), max_vw(0.8), max_vx_run(0.6), max_vy_run(0.6), max_vw_run(0.8), deadman_no_publish_(deadman_no_publish)
      {
        torso_eff.data = 0;
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
        
        param<int>("axis_vx", axis_vx, 1);
        param<int>("axis_vw", axis_vw, 0);
        param<int>("axis_vy", axis_vy, 2);
        
        param<int>("deadman_button", deadman_button, 0);
        param<int>("run_button", run_button, 0);
        param<int>("torso_dn_button", torso_dn_button, 0);
        param<int>("torso_up_button", torso_up_button, 0);
        param<int>("passthrough_button", passthrough_button, 1);
        
        printf("max_vx: %.3f m/s\n", max_vx);
        printf("max_vy: %.3f m/s\n", max_vy);
        printf("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);
        
        printf("max_vx_run: %.3f m/s\n", max_vx_run);
        printf("max_vy_run: %.3f m/s\n", max_vy_run);
        printf("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);
        
        printf("axis_vx: %d\n", axis_vx);
        printf("axis_vy: %d\n", axis_vy);
        printf("axis_vw: %d\n", axis_vw);
        printf("deadman_button: %d\n", deadman_button);
        printf("run_button: %d\n", run_button);
        printf("torso_dn_button: %d\n", torso_dn_button);
        printf("torso_up_button: %d\n", torso_up_button);
        printf("passthrough_button: %d\n", passthrough_button);
        
        if (torso_dn_button != 0)
          advertise<std_msgs::Float64>(TORSO_TOPIC, 1);
          
        advertise<robot_msgs::PoseDot>("cmd_vel", 1);
        subscribe("joy", joy, &TeleopBase::joy_cb, 1);
        subscribe("cmd_passthrough", cmd_passthrough, &TeleopBase::passthrough_cb, 1);
        printf("done with ctor\n");
      }
  
  ~TeleopBase()
  {
    unsubscribe("joy");
    unsubscribe("cmd_passthrough");
    unadvertise("cmd_vel");
    if (torso_dn_button != 0)
      unadvertise(TORSO_TOPIC);
  }

      void joy_cb()
      {
         /*
           printf("axes: ");
           for(int i=0;i<joy.get_axes_size();i++)
           printf("%.3f ", joy.axes[i]);
           puts("");
           printf("buttons: ");
           for(int i=0;i<joy.get_buttons_size();i++)
           printf("%d ", joy.buttons[i]);
           puts("");
         */

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

         bool down = (((unsigned int)torso_dn_button < joy.get_buttons_size()) && joy.buttons[torso_dn_button]);
         bool up = (((unsigned int)torso_up_button < joy.get_buttons_size()) && joy.buttons[torso_up_button]);

         if (down && !up)
           req_torso = -10000; // Bring torso down with max effort
         else if (up && !down)
           req_torso = 10000;
         else
           req_torso = 0;
      }
      void passthrough_cb() { }
      void send_cmd_vel()
      {
         joy.lock();
         if((deadman_button < 0) ||
            ((((unsigned int)deadman_button) < joy.get_buttons_size()) &&
             joy.buttons[deadman_button]))
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

            torso_eff.data = req_torso;
	    if (torso_dn_button != 0)
	      publish(TORSO_TOPIC, torso_eff);

            if (req_torso != 0)
              fprintf(stderr,"teleop_base:: %f, %f, %f. Torso effort: %f.\n",cmd.vel.vx,cmd.vel.vy,cmd.ang_vel.vz, torso_eff.data);
            else
              fprintf(stderr,"teleop_base:: %f, %f, %f\n",cmd.vel.vx,cmd.vel.vy,cmd.ang_vel.vz);
         }
         else
         {
           cmd.vel.vx = cmd.vel.vy = cmd.ang_vel.vz = 0;
           torso_eff.data = 0;
            if (!deadman_no_publish_)
           {
             publish("cmd_vel", cmd);//Only publish if deadman_no_publish is enabled
	     if (torso_dn_button != 0)
	       publish(TORSO_TOPIC, torso_eff);
             //fprintf(stderr,"teleop_base:: deadman off\n");
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

