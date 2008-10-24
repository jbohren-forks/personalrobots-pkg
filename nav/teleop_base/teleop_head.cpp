#include <unistd.h>
#include <math.h>
#include "ros/node.h"
#include "joy/Joy.h"

#include "robot_srvs/SetJointCmd.h"

using namespace ros;

class TeleopHead : public node
{
   public:
  //      std_msgs::HeadVel cmd, cmd_passthrough;
      joy::Joy joy;
      double req_pan, req_tilt, max_pan, max_tilt;
      int axis_pan, axis_tilt;
      int deadman_button, passthrough_button;
  double pan_step, tilt_step;
      bool deadman_no_publish_;


  TeleopHead(bool deadman_no_publish = false) : node("teleop_head"), max_pan(0.6), max_tilt(0.4), pan_step(0.1), tilt_step(0.1), deadman_no_publish_(deadman_no_publish)
      {
        //     cmd.vx = cmd.vy = cmd.vw = 0;
         if (!has_param("max_pan") || !get_param("max_pan", max_pan))
            ROS_WARN("maximum pan not set. Assuming 0.6");
         if (!has_param("max_tilt") || !get_param("max_tilt", max_tilt))
            ROS_WARN("maximum tilt not set. Assuming 0.4");

         param<int>("axis_pan", axis_pan, 4);
         param<int>("axis_tilt", axis_tilt, 5);
         param<int>("deadman_button", deadman_button, 0);
         param<int>("passthrough_button", passthrough_button, 1);

         /*         printf("max_vx: %.3f m/s\n", max_vx);
         printf("max_vy: %.3f m/s\n", max_vy);
         printf("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);
         printf("axis_vx: %d\n", axis_vx);
         printf("axis_vy: %d\n", axis_vy);
         printf("axis_vw: %d\n", axis_vw);
         printf("deadman_button: %d\n", deadman_button);
         printf("passthrough_button: %d\n", passthrough_button);
         */
         //         advertise<std_msgs::HeadVel>("cmd_vel", 1);
         subscribe("joy", joy, &TeleopHead::joy_cb, 1);
         //subscribe("cmd_passthrough", cmd_passthrough, &TeleopHead::passthrough_cb, 1);
         printf("done with ctor\n");
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

         if((axis_pan >= 0) && (((unsigned int)axis_pan) < joy.get_axes_size()))
         {
           req_pan += joy.axes[axis_pan] * pan_step;
           req_pan = std::max(std::min(req_pan, max_pan), -max_pan);
         }

         if ((axis_tilt >= 0) && (((unsigned int)axis_tilt) < joy.get_axes_size()))
         {
           req_tilt += joy.axes[axis_tilt] * tilt_step;
           req_tilt = std::max(std::min(req_tilt, max_tilt), -max_tilt);
         }

         robot_srvs::SetJointCmd::request req;
         robot_srvs::SetJointCmd::response res;
         req.positions.push_back(req_pan);
         req.positions.push_back(req_tilt);
         req.velocity.push_back(0.0);
         req.velocity.push_back(0.0);
         req.acc.push_back(0.0);
         req.acc.push_back(0.0);
         req.names.push_back("head_pan_joint");
         req.names.push_back("head_tilt_joint");
         ros::service::call("head_controller/set_command_array", req, res);

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
   TeleopHead teleop_base(no_publish);
   while (teleop_base.ok())
   {
      usleep(50000);
   }
   ros::fini();
   exit(0);
   return 0;
}

