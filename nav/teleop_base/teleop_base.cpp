#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include "ros/node.h"
#include "joy/Joy.h"
#include "robot_msgs/PoseDot.h"
#include "robot_msgs/JointCmd.h"
#include "std_msgs/Float64.h"

#define TORSO_TOPIC "/torso_lift_controller/command"
#define HEAD_TOPIC "/head_controller/set_command_array"

using namespace ros;

class TeleopBase : public Node
{
   public:
  robot_msgs::PoseDot cmd, cmd_passthrough;
  std_msgs::Float64 torso_eff;
  joy::Joy joy;
  double req_vx, req_vy, req_vw, req_torso, req_pan, req_tilt;
  double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
  double max_pan, max_tilt, min_tilt, pan_step, tilt_step;
  int axis_vx, axis_vy, axis_vw, axis_pan, axis_tilt;
  int deadman_button, run_button, torso_dn_button, torso_up_button, head_button, passthrough_button;
  bool deadman_no_publish_, torso_publish, head_publish;

  // Set pan, tilt steps as params

  TeleopBase(bool deadman_no_publish = false) : Node("teleop_base"), max_vx(0.6), max_vy(0.6), max_vw(0.8), max_vx_run(0.6), max_vy_run(0.6), max_vw_run(0.8), max_pan(2.7), max_tilt(1.4), min_tilt(-0.4), pan_step(0.1), tilt_step(0.1), deadman_no_publish_(deadman_no_publish)
      {
        torso_eff.data = 0;
        cmd.vel.vx = cmd.vel.vy = cmd.ang_vel.vz = 0;
        req_pan = req_tilt = 0;
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
        
        if (!hasParam("max_pan") || !getParam("max_pan", max_pan))
          ROS_WARN("maximum pan not set. Assuming 0.6");
        if (!hasParam("max_tilt") || !getParam("max_tilt", max_tilt))
          ROS_WARN("maximum tilt not set. Assuming 0.4");

        param<int>("axis_pan", axis_pan, 0);
        param<int>("axis_tilt", axis_tilt, 2);

        param<int>("axis_vx", axis_vx, 3);
        param<int>("axis_vw", axis_vw, 0);
        param<int>("axis_vy", axis_vy, 2);
        
        param<int>("deadman_button", deadman_button, 0);
        param<int>("run_button", run_button, 0);
        param<int>("torso_dn_button", torso_dn_button, 0);
        param<int>("torso_up_button", torso_up_button, 0);
        param<int>("head_button", head_button, 0);
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
        printf("axis_pan: %d\n", axis_pan);
        printf("axis_tilt: %d\n", axis_tilt);
        
        printf("deadman_button: %d\n", deadman_button);
        printf("run_button: %d\n", run_button);
        printf("torso_dn_button: %d\n", torso_dn_button);
        printf("torso_up_button: %d\n", torso_up_button);
        printf("head_button: %d\n", head_button);
        printf("passthrough_button: %d\n", passthrough_button);
        
        if (torso_dn_button != 0)
          advertise<std_msgs::Float64>(TORSO_TOPIC, 1);
        if (head_button != 0)
          advertise<robot_msgs::JointCmd>(HEAD_TOPIC, 1);

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
    if (head_button != 0)
      unadvertise(HEAD_TOPIC);

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
        bool cmd_head = (((unsigned int)head_button < joy.get_buttons_size()) && joy.buttons[head_button]);

        bool deadman = (((unsigned int)deadman_button < joy.get_buttons_size()) && joy.buttons[deadman_button]);
        
        // Base
        bool running = (((unsigned int)run_button < joy.get_buttons_size()) && joy.buttons[run_button]);
        double vx = running ? max_vx_run : max_vx;
        double vy = running ? max_vy_run : max_vy;
        double vw = running ? max_vw_run : max_vw;

         if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy.get_axes_size()) && !cmd_head)
            req_vx = joy.axes[axis_vx] * vx;
         else
            req_vx = 0.0;
         if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy.get_axes_size()) && !cmd_head)
            req_vy = joy.axes[axis_vy] * vy;
         else
            req_vy = 0.0;
         if((axis_vw >= 0) && (((unsigned int)axis_vw) < joy.get_axes_size()) && !cmd_head)
            req_vw = joy.axes[axis_vw] * vw;
         else
            req_vw = 0.0;

         // Head
         // Update commanded position by how joysticks moving
         // Don't add commanded position if deadman off
         if((axis_pan >= 0) && (((unsigned int)axis_pan) < joy.get_axes_size()) && cmd_head && deadman)
         {
           req_pan += joy.axes[axis_pan] * pan_step;
           req_pan = std::max(std::min(req_pan, max_pan), -max_pan);
         }

         if ((axis_tilt >= 0) && (((unsigned int)axis_tilt) < joy.get_axes_size()) && cmd_head && deadman)
         {
           req_tilt += joy.axes[axis_tilt] * tilt_step;
           req_tilt = std::max(std::min(req_tilt, max_tilt), min_tilt);
         }


         // Torso
         bool down = (((unsigned int)torso_dn_button < joy.get_buttons_size()) && joy.buttons[torso_dn_button]);
         bool up = (((unsigned int)torso_up_button < joy.get_buttons_size()) && joy.buttons[torso_up_button]);

         // Bring torso up/down with max effort
         if (down && !up)
           req_torso = -10000; 
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
            
            // Torso
            torso_eff.data = req_torso;
	    if (torso_dn_button != 0)
	      publish(TORSO_TOPIC, torso_eff);

            // Head
            if (head_button != 0)
            {
              robot_msgs::JointCmd joint_cmds ;
              joint_cmds.positions.push_back(req_pan);
              joint_cmds.positions.push_back(req_tilt);
              joint_cmds.velocity.push_back(0.0);
              joint_cmds.velocity.push_back(0.0);
              joint_cmds.acc.push_back(0.0);
              joint_cmds.acc.push_back(0.0);
              joint_cmds.names.push_back("head_pan_joint");
              joint_cmds.names.push_back("head_tilt_joint");
              publish(HEAD_TOPIC, joint_cmds);
            }

            if (req_torso != 0)
              fprintf(stderr,"teleop_base:: %f, %f, %f. Head:: %f, %f. Torso effort: %f.\n",cmd.vel.vx,cmd.vel.vy,cmd.ang_vel.vz, req_pan, req_tilt, torso_eff.data);
            else
              fprintf(stderr,"teleop_base:: %f, %f, %f. Head:: %f, %f\n",cmd.vel.vx,cmd.vel.vy,cmd.ang_vel.vz, req_pan, req_tilt);
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

             // Publish head
             if (head_button != 0)
             {
               robot_msgs::JointCmd joint_cmds ;
               joint_cmds.positions.push_back(req_pan);
               joint_cmds.positions.push_back(req_tilt);
               joint_cmds.velocity.push_back(0.0);
               joint_cmds.velocity.push_back(0.0);
               joint_cmds.acc.push_back(0.0);
               joint_cmds.acc.push_back(0.0);
               joint_cmds.names.push_back("head_pan_joint");
               joint_cmds.names.push_back("head_tilt_joint");
               publish(HEAD_TOPIC, joint_cmds);
             }
             
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

