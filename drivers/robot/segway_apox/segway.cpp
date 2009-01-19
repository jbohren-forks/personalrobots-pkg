#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "ros/node.h"
#include "boost/thread/mutex.hpp"
#include "std_msgs/BaseVel.h"
#include "std_msgs/RobotBase2DOdom.h"
#include "std_msgs/String.h"
#include "rmp_frame.h"
extern "C" {
#include "usbcan.h"
}
#include "tf/transform_broadcaster.h"

using namespace ros;

class Segway : public Node
{
  public:
    Segway();
    virtual ~Segway();
    void main_loop();
    void cmd_vel_cb();
    void op_mode_cb();

		BYTE send_data[20];
		int last_raw_yaw_rate, last_raw_x_vel;
		void build_vel_pkt(float x_vel, float yaw_rate);

		float req_x_vel, req_yaw_rate;
    double req_time;
    boost::mutex req_mutex;

		static const int max_x_stepsize = 5, max_yaw_stepsize = 2;

    std_msgs::BaseVel cmd_vel;
    std_msgs::RobotBase2DOdom odom;
    std_msgs::String op_mode;
		rmp_frame_t rmp;
		dgc_usbcan_p can;
		int last_foreaft, last_yaw;
		float odom_yaw, odom_x, odom_y;
		bool odom_init;
    int op_mode_req;
    enum { RUNNING, SHUTDOWN_REQ, SHUTDOWN } pkt_mode;
    tf::TransformBroadcaster tf;
    bool req_timeout;
};

const float MAX_X_VEL = 1.2;
const float MAX_YAW_RATE = 0.4;

Segway::Segway() :
	Node("segway"),
	last_raw_yaw_rate(0),
	last_raw_x_vel(0),
	req_x_vel(0),
	req_yaw_rate(0),
  req_time(0),
	can(0),
	odom_yaw(0),
	odom_x(0),
	odom_y(0),
	odom_init(false),
  op_mode_req(0),
  pkt_mode(RUNNING),
  tf(*this),
  req_timeout(false)
{
  odom.header.frame_id = "odom";
  advertise<std_msgs::RobotBase2DOdom>("odom", 1);
  subscribe("cmd_vel", cmd_vel, &Segway::cmd_vel_cb, 1);
  subscribe("operating_mode", op_mode, &Segway::op_mode_cb, 1);
}

Segway::~Segway()
{
	if (can)
		dgc_usbcan_close(&can);
}

void Segway::op_mode_cb()
{
  if (op_mode.data == std::string("shutdown"))
  {
    printf("shutting down motors\n");
    pkt_mode = SHUTDOWN_REQ;
  }
  else if (op_mode.data == std::string("wtf"))
  {
    printf("WTF?\n");
    pkt_mode = RUNNING;
    op_mode_req = 4;
  }
  else if (op_mode.data == std::string("powerdown"))
  {
    printf("powering down\n");
    op_mode_req = 3;
    pkt_mode = RUNNING;
  }
  else if (op_mode.data == std::string("tractor"))
  {
    printf("going into tractor mode\n");
    op_mode_req = 1;
    pkt_mode = RUNNING;
  }
  else if (op_mode.data == std::string("reset_integrators"))
  {
    printf("resetting integrators\n");
    op_mode_req = -1;
    pkt_mode = RUNNING;
  }
}

void Segway::cmd_vel_cb()
{
  req_mutex.lock();
  req_time = ros::Time::now().to_double();
  req_x_vel = cmd_vel.vx;
	req_yaw_rate = cmd_vel.vw;
	req_mutex.unlock();
}

void Segway::build_vel_pkt(float x_vel, float yaw_rate)
{

	// threshold us to sane amounts!
	if (x_vel > MAX_X_VEL)
		x_vel = MAX_X_VEL;
	else if (x_vel < -MAX_X_VEL)
		x_vel = -MAX_X_VEL;

	if (yaw_rate > MAX_YAW_RATE)
		yaw_rate = MAX_YAW_RATE;
	else if (yaw_rate < -MAX_YAW_RATE)
		yaw_rate = -MAX_YAW_RATE;

	int16_t raw_x_vel = (int16_t)rint(x_vel*(float)RMP_COUNT_PER_M_PER_S);
	int16_t raw_yaw_rate = (int16_t)rint(yaw_rate*(float)415.9);//1.9); // rad/sec to RMP magic units

	// threshold us again, just to be sure...
	if (raw_x_vel > RMP_MAX_TRANS_VEL_COUNT)
		raw_x_vel = RMP_MAX_TRANS_VEL_COUNT;
	else if (raw_x_vel < -RMP_MAX_TRANS_VEL_COUNT)
		raw_x_vel = -RMP_MAX_TRANS_VEL_COUNT;

	if (raw_yaw_rate > RMP_MAX_ROT_VEL_COUNT)
		raw_yaw_rate = RMP_MAX_ROT_VEL_COUNT;
	else if (raw_yaw_rate < -RMP_MAX_ROT_VEL_COUNT)
		raw_yaw_rate = -RMP_MAX_ROT_VEL_COUNT;

	// now have it ramp to the target velocity (so it doesn't jerk around)
	if (raw_x_vel > last_raw_x_vel + max_x_stepsize)
		raw_x_vel = last_raw_x_vel + max_x_stepsize;
	else if (raw_x_vel < last_raw_x_vel - max_x_stepsize)
		raw_x_vel = last_raw_x_vel - max_x_stepsize;
	last_raw_x_vel = raw_x_vel;

	if (raw_yaw_rate > last_raw_yaw_rate + max_yaw_stepsize)
		raw_yaw_rate = last_raw_yaw_rate + max_yaw_stepsize;
	else if (raw_yaw_rate < last_raw_yaw_rate - max_yaw_stepsize)
		raw_yaw_rate = last_raw_yaw_rate - max_yaw_stepsize;
	last_raw_yaw_rate = raw_yaw_rate;

	unsigned short u_raw_x_vel = (unsigned short)raw_x_vel;
	unsigned short u_raw_yaw_rate = (unsigned short)raw_yaw_rate;
	send_data[0] = (u_raw_x_vel >> 8) & 0xff;
	send_data[1] = u_raw_x_vel & 0xff;
	send_data[2] = (u_raw_yaw_rate >> 8) & 0xff;
	send_data[3] = u_raw_yaw_rate & 0xff;
  if (op_mode_req)
  {
    //if (op_mode_req == 2)
    //  op_mode_req = 3; // NEVER NEVER NEVER let STAIR1 go into balancing mode!!!
    int omr = op_mode_req;
    if (omr == 4)
      omr = 0;
    if (omr == 0 || omr == 1 || omr == 3)
    {
      send_data[4] = (((unsigned short)RMP_CAN_CMD_SET_OPERATIONAL_MODE) >> 8) & 0xff;
      send_data[5] = ((unsigned short)RMP_CAN_CMD_SET_OPERATIONAL_MODE) & 0xff;
      send_data[6] = 0;
      send_data[7] = omr;
    }
    else if (omr == -1)
    {
      send_data[4] = (((unsigned short)RMP_CAN_CMD_RST_INT) >> 8) & 0xff;
      send_data[5] = ((unsigned short)RMP_CAN_CMD_RST_INT) & 0xff;
      send_data[6] = 0;
      send_data[7] = 0x0f; // reset all channels
    }
    op_mode_req = 0; // only send it once
  }
  else
  {
	  send_data[4] = (((unsigned short)RMP_CAN_CMD_NONE) >> 8) & 0xff;
  	send_data[5] = ((unsigned short)RMP_CAN_CMD_NONE) & 0xff;
	  send_data[6] = 0;
  	send_data[7] = 0;
  }
}

double normalize_angle(double a)
{
  if (fabs(a) > 1e8)
  {
    ROS_FATAL("woah! stupidly large angle.\n");
    ROS_BREAK();
  }
  while (a < -M_PI)
    a += 2 * M_PI;
  while (a > M_PI)
    a -= 2 * M_PI;
  return a;
}

int rmp_diff(uint32_t from, uint32_t to)
{
	int diff1, diff2;
	diff1 = to - from;
	if (to > from)
		diff2 = -(from + UINT_MAX - to);
	else
		diff2 = UINT_MAX - from + to;
	if (abs(diff1) < abs(diff2))
		return diff1;
	else
		return diff2;
}

void Segway::main_loop()
{
  ROS_DEBUG("segway apox main_loop\n");
	can = dgc_usbcan_initialize("/dev/ttyUSB1"); // pull from a param someday...

	if (!can)
	{
		ROS_FATAL("ahh couldn't open the can controller\n");
		ROS_BREAK();
	}

	unsigned char message[100];
	int message_length;
	unsigned can_id;
  double last_send_time = ros::Time::now().to_double();

/*
  tf.sendTransform(
      tf::Transform(
        tf::Quaternion(0, 0, 0), 
        tf::Point(0.25, 0.0, 0.0)).inverse(), 
        ros::Time::now(), "base_laser", "base");
  */

  while(ok())
  {
    if (ros::Time::now().to_double() - last_send_time > 0.01)
    {
      double time_since_last_cmd = ros::Time::now().to_double() - req_time;
      if (time_since_last_cmd > 0.15)
        req_timeout = true;
      else
        req_timeout = false;

      if (!req_timeout)
      {
        if (pkt_mode == RUNNING)
        {
      	  req_mutex.lock();
      		build_vel_pkt(req_x_vel, req_yaw_rate);
      		req_mutex.unlock();
      		dgc_usbcan_send_can_message(can, RMP_CAN_ID_COMMAND, send_data, 8);
        }
        else if (pkt_mode == SHUTDOWN_REQ)
        {
          printf("sending shutdown packet\n");
          pkt_mode = SHUTDOWN;
          for (int i = 0; i < 8; i++)
            send_data[i] = 0;
          dgc_usbcan_send_can_message(can, RMP_CAN_ID_SHUTDOWN, send_data, 8);
        }
      }
      last_send_time = ros::Time::now().to_double();
    }

		if (dgc_usbcan_read_message(can, &can_id, message, &message_length))
		{
			if (can_id == RMP_CAN_ID_MSG5)
			{
				if (!odom_init)
					odom_init = true;
				else
				{
					int delta_lin_raw = rmp_diff(last_foreaft, rmp.foreaft);
					int delta_yaw_raw = rmp_diff(last_yaw, rmp.yaw);
					double delta_lin = (double)delta_lin_raw / RMP_COUNT_PER_M * 0.825;
					double delta_ang = (double)delta_yaw_raw / RMP_COUNT_PER_REV * 2 * M_PI * 0.925;

					odom_x += delta_lin * cos(odom_yaw);
					odom_y += delta_lin * sin(odom_yaw);
					//odom_yaw = normalize_angle(odom_yaw + delta_ang);
					odom_yaw = odom_yaw + delta_ang;

          static int odom_count = 0;
          if (odom_count++ % 3 == 0) // send it at 5 hz or so
          {
            //						printf("(%f, %f, %f)\n", odom_x, odom_y, odom_yaw);
            odom.pos.x  = odom_x;
            odom.pos.y  = odom_y;
            odom.pos.th = odom_yaw;
            publish("odom", odom);
            tf.sendTransform(
              tf::Transform(
                tf::Quaternion(0, 0, 0), 
                tf::Point(0.25, 0.0, 0.0)).inverse(), 
              ros::Time::now(), "base_laser", "base");
            tf.sendTransform(
              tf::Transform(
                tf::Quaternion(odom.pos.th, 0, 0), 
                  tf::Point(odom.pos.x, odom.pos.y, 0.0)).inverse(),
              odom.header.stamp, "odom", "base");
          }
        }
        last_foreaft = rmp.foreaft;
        last_yaw = rmp.yaw;

        tf.sendTransform(tf::Transform(tf::Quaternion(
                odom.pos.th,
                0,
                0),
              tf::Point(
                odom.pos.x,
                odom.pos.y,
                0.0)
              ).inverse(),
            odom.header.stamp,
            "odom",
            "base");
      }
    }
		else
			usleep(5000);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  Segway s;
  s.main_loop();
	s.shutdown();
  return 0;
}
