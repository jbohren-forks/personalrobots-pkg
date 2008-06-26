#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "ros/node.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/RobotBase2DOdom.h"
#include "rmp_frame.h"
#include "usbcan.h"

using namespace ros;

class Segway : public node
{
  public:
    Segway();
    virtual ~Segway();
    void main_loop();
    void cmd_vel_cb();
    void deadman_cb();

		BYTE send_data[20];
		int last_raw_yaw_rate, last_raw_x_vel;
		void build_vel_pkt(float x_vel, float yaw_rate);

		float req_x_vel, req_yaw_rate;
    thread::mutex req_mutex;

		static const int max_x_stepsize = 5, max_yaw_stepsize = 2;

    std_msgs::BaseVel cmd_vel;
    std_msgs::RobotBase2DOdom odom;
		rmp_frame_t rmp;
		dgc_usbcan_p can;
		int last_foreaft, last_yaw;
		float odom_yaw, odom_x, odom_y;
		bool odom_init;
};

const float MAX_X_VEL = 1.2;
const float MAX_YAW_RATE = 0.4;

Segway::Segway() :
	node("segway"),
	last_raw_yaw_rate(0),
	last_raw_x_vel(0),
	req_x_vel(0),
	req_yaw_rate(0),
	can(0),
	odom_yaw(0),
	odom_x(0),
	odom_y(0),
	odom_init(false)
{
  advertise("odom", odom);
  subscribe("cmd_vel", cmd_vel, &Segway::cmd_vel_cb);
}

Segway::~Segway()
{
	if (can)
		dgc_usbcan_close(&can);
}

void Segway::cmd_vel_cb()
{
  req_mutex.lock();
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
	send_data[4] = (((unsigned short)RMP_CAN_CMD_NONE) >> 8) & 0xff;
	send_data[5] = ((unsigned short)RMP_CAN_CMD_NONE) & 0xff;
	send_data[6] = 0;
	send_data[7] = 0;
}

double normalize_angle(double a)
{
  if (fabs(a) > 1e8)
    g_node->log(FATAL, "woah! stupidly large angle.\n");
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
	can = dgc_usbcan_initialize("/dev/ttyUSB0"); // pull from a port someday...

	if (!can)
		log(FATAL, "ahh couldn't open the can controller\n");

	unsigned char message[100];
	int message_length;
	unsigned can_id;
	
	while(ok())
	{
		if (dgc_usbcan_read_message(can, &can_id, message, &message_length))
		{
			rmp.AddPacket(can_id, message);
			if (can_id == RMP_CAN_ID_MSG5)
			{
//				static int c = 0;
//				if (c++ % 100 == 0)
//					printf("%d %d\n", rmp.foreaft, rmp.yaw);
					
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
					odom_yaw = normalize_angle(odom_yaw + delta_ang);
					
					static int odom_count = 0;
					if (odom_count++ % 10 == 0) // send it at 5 hz or so
					{
//						printf("(%f, %f, %f)\n", odom_x, odom_y, odom_yaw);
            odom.pos.x  = odom_x;
            odom.pos.y  = odom_y;
            odom.pos.th = odom_yaw;
            publish("odom", odom);
					}
				}
				last_foreaft = rmp.foreaft;
				last_yaw = rmp.yaw;

				req_mutex.lock();
				build_vel_pkt(req_x_vel, req_yaw_rate);
				req_mutex.unlock();
				dgc_usbcan_send_can_message(can, RMP_CAN_ID_COMMAND, send_data, 8);
			}
		}
		else
		{
			usleep(5000);
		}
//		char c = get_key_nonblocking();
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
