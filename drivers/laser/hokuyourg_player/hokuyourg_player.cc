#include <assert.h>

#include <libstandalone_drivers/urg_laser.h>

#include <ros/node.h>
#include <std_msgs/MsgLaserScan.h>

class HokuyoNode: public ros::node
{
  private:
    urg_laser_readings_t* readings;
    urg_laser_config_t cfg;
    bool running;
    unsigned int scanid;

  public:
    urg_laser urg;
    MsgLaserScan scan;
    double min_ang;
    double max_ang;
    string port;

    HokuyoNode() : ros::node("urglaser")
    {
      advertise<MsgLaserScan>("scan");

      // TODO: add min/max angle support
      /*
      if (!get_double_param(".min_ang", &min_ang))
	min_ang = -90;
      printf("Setting min_ang to: %g\n",min_ang);

      if (!get_double_param(".max_ang", &max_ang))
	max_ang = 90;
      printf("Setting max_ang to: %g\n",max_ang);
      */

      if (!has_param("port") || !get_param("port", port))
	port = "/dev/ttyACM0";
      printf("Setting port to: %s\n",port.c_str());

      readings = new urg_laser_readings_t;
      assert(readings);
      running = false;
      scanid = 0;
    }

    ~HokuyoNode()
    {
      stop();
      delete readings;
    }

    int start()
    {
      stop();
      if((urg.Open(port.c_str(),0,0) < 0) || (urg.GetSensorConfig(&cfg) < 0))
      {
        puts("error connecting to laser");
        return(-1);
      }
      running = true;
      return(0);
    }

    int stop()
    {
      if(running)
      {
        urg.Close();
        running = false;
      }
      return(0);
    }

    int publish_scan()
    {
      // TODO: add support for pushing readings out
      int numreadings;
      if((numreadings = urg.GetReadings(readings,-1,-1)) < 0)
      {
        puts("error getting scan");
        return(numreadings);
      }

      printf("%d readings\n", numreadings);

      scan.angle_min = cfg.min_angle;
      scan.angle_max = cfg.max_angle;
      scan.angle_increment = cfg.resolution;
      scan.range_max = cfg.max_range;
      scan.header.seq = scanid++;
      scan.set_ranges_size(numreadings);
      scan.set_intensities_size(numreadings);

      for(int i = 0; i < numreadings; ++i)
      {
        scan.ranges[i]  = readings->Readings[i] < 20 ? (scan.range_max*1000) : (readings->Readings[i]);
        scan.ranges[i] /= 1000;
        // TODO: add intensity support
        scan.intensities[i] = 0;
      }
      publish("scan", scan);
      return(0);
    }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  HokuyoNode hn;

  // Start up the laser
  if(hn.start() != 0)
    hn.self_destruct();

  while (hn.ok())
  {
    if(hn.publish_scan() < 0)
      break;
  }

  //stopping should be fine even if not running
  hn.stop();

  return(0);
}
