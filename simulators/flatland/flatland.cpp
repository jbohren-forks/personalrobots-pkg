#include <unistd.h>
#include <math.h>
#include "ros/node.h"
#include "random_utils/random_utils.h"
#include "std_msgs/RobotBase2DOdom.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/LaserScan.h"

class FlatlandRobot
{
public:
  double x, y, th;
  double odom_x, odom_y, odom_th;
  double v, w; // linear and angular velocity
  double v_bias, w_bias;
  FlatlandRobot() : x(0), y(0), th(0), 
    odom_x(0), odom_y(0), odom_th(0),
    v(0), w(0)
  {
    v_bias = random_utils::gaussian(0, 0.01);
    w_bias = random_utils::gaussian(0, 0.01);
  }
  inline double clamp(double d, double l, double u)
  {
    return (d > u ? u : (d < l ? l : d));
  }
  double normalize_angle(double a)
  {
    while (a > M_PI)
      a -= 2 * M_PI;
    while (a < -M_PI)
      a += 2 * M_PI;
    return a;
  }
  void tic(double dt)
  {
    double xn = x + dt * v * cos(th);
    double yn = y + dt * v * sin(th);
    double vn = (fabs(v) > 0.001 ? v * (1.0 + v_bias) : v);
    double wn = (fabs(w) > 0.001 ? w * (1.0 + w_bias) : w);
    v_bias += random_utils::uniform(-0.0001, 0.0001);
    w_bias += random_utils::uniform(-0.0001, 0.0001);
    // clamp to sane values
    v_bias = clamp(v_bias, -0.1, 0.1);
    w_bias = clamp(w_bias, -0.1, 0.1);
    // todo: check against wall collisions like switchyard flatland
    x = xn;
    y = yn;
    odom_x += dt * vn * cos(odom_th);
    odom_y += dt * vn * sin(odom_th);
    th = normalize_angle(th + dt * w);
    odom_th = normalize_angle(odom_th + dt * wn);
  }
};

class Flatland : public ros::node
{
public:
  std_msgs::RobotBase2DOdom odom;
  std_msgs::BaseVel cmd_vel;
  std_msgs::LaserScan laser;

  FlatlandRobot robot;
  double last_odom_t, last_laser_t, last_t;

  Flatland() : ros::node("flatland"), 
    last_odom_t(0), last_laser_t(0), last_t(0)
  {
    laser.set_ranges_size(100);
    laser.angle_min = -M_PI/2;
    laser.angle_max =  M_PI/2;
    laser.angle_increment = M_PI/99;
    laser.range_max = 10;

    advertise<std_msgs::RobotBase2DOdom>("odom");
    advertise<std_msgs::LaserScan>("laser");
    subscribe("cmd_vel", cmd_vel, &Flatland::cmd_vel_cb);
  }
  void cmd_vel_cb()
  {
    cmd_vel.lock();
    robot.v = cmd_vel.vx;
    robot.w = cmd_vel.vw;
    cmd_vel.unlock();
  }
  void tic()
  {
    double t = ros::Time::now().to_double();
    double dt = t - last_t;
    last_t = t;
    robot.tic(dt);
    if (t > last_odom_t + 0.05)
    {
      // send odom message
      odom.pos.x  = robot.odom_x;
      odom.pos.y  = robot.odom_y;
      odom.pos.th = robot.odom_th;
      publish("odom", odom);
      last_odom_t = t;
    }
    if (t > last_laser_t + 0.5)
    {
      for (size_t i = 0; i < laser.get_ranges_size(); i++)
        laser.ranges[i] = random_utils::gaussian(2, 0.5);
      publish("laser", laser);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  Flatland flatland;
  while (flatland.ok())
  {
    usleep(10000);
    flatland.tic();
  }
  ros::fini();
  return 0;
}

