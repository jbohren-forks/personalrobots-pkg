
#include <assert.h>
#include <math.h>
#include <iostream>

#include "urg_laser.h"
#include "ros/time.h"
#include "ipdcmot/ipdcmot.h"
#include "kbhit.h"

const char *port = "/dev/ttyACM0";

urg_laser_scan_t *scan = NULL;
urg_laser_config_t cfg;
urg_laser urg;
IPDCMOT *mot = NULL;
const double min_ang = -70.0 * M_PI/180;
const double max_ang =  70.0 * M_PI/180;
const int cluster = 1;
const int skip = 0;
bool running = false, bail = false;
const double left_scan_extent = 5, right_scan_extent = 150;
const char *ipdcmot_ip = "192.168.1.38";

int stop();

int start()
{
  stop();
  if ((urg.open(port) < 0))
  {
    puts("error connecting to laser");
    return(-1);
  }
  printf("Connected to URG with ID: %d\n", urg.get_ID());
  urg.urg_cmd("BM");
  int status = urg.request_scans(true, min_ang, max_ang, cluster, skip);
  if (status != 0) 
  {
    printf("Failed to request scans %d.\n", status);
    return -1;
  }
  running = true;
  return 0;
}

int stop()
{
  if(running)
  {
    urg.close();
    running = false;
  }
  return 0;
}

int grab_scan()
{
  int status = urg.service_scan(scan);
  if(status != 0)
  {
    printf("error getting scan: %d\n", status);
    return -1;
  }
/*
  scan_msg.angle_min = scan->config.min_angle;
  scan_msg.angle_max = scan->config.max_angle;
  scan_msg.angle_increment = scan->config.resolution;
  scan_msg.range_max = cfg.max_range;
  scan_msg.set_ranges_size(scan->num_readings);
  scan_msg.set_intensities_size(scan->num_readings);

  for(int i = 0; i < scan->num_readings; ++i)
  {
    scan_msg.ranges[i]  = scan->ranges[i];
    scan_msg.intensities[i] = scan->intensities[i];
  }
*/
  return 0;
}

void *urg_thread_func(void *)
{
  printf("entering urg thread\n");
  FILE *urg_log = fopen("urg.txt", "w");
  if (!urg_log)
  {
    printf("couldn't open urg log\n");
    bail = true;
    return NULL;
  }
  while (!bail)
  {
    if (grab_scan() < 0)
    {
      printf("error grabbing scan\n");
      break;
    }
    ros::Time t = ros::Time::now();
    fprintf(urg_log, "%d.%09d %f %f %d ", t.sec, t.nsec, 
            min_ang, max_ang, scan->num_readings);
    for (int i = 0; i < scan->num_readings; i++)
      fprintf(urg_log, "%.3f %.0f ", scan->ranges[i], scan->intensities[i]);
    fprintf(urg_log, "\n");
  }
  printf("exiting urg thread\n");
  fclose(urg_log);
  return NULL;
}

int main(int argc, char** argv)
{
  FILE *mot_log = fopen("mot.txt", "w");
  if (!mot_log)
  {
    printf("couldn't open motor log\n");
    return 1;
  }
  mot = new IPDCMOT(ipdcmot_ip, 0, false);
  mot->set_pos_deg_blocking(left_scan_extent);
  usleep(500000);
  scan = new urg_laser_scan_t;
  if (start())
  {
    printf("bogus. couldn't start the laser.\n");
    return 1;
  }
  pthread_t urg_thread;
  pthread_create(&urg_thread, NULL, urg_thread_func, NULL);
  mot->set_patrol(left_scan_extent, right_scan_extent, 1.9, 1);
  init_keyboard();
  int count = 0;
  while (!bail && !_kbhit())
  {
    double pos;
    if (!mot->get_pos_blocking(&pos, NULL, 1))
    {
      printf("woah! couldn't get position\n");
      break;
    }
    if (mot->get_patrol_dir() < 0)
    {
      printf("scan complete\n");
      break;
    }
    ros::Time t = ros::Time::now();
    fprintf(mot_log, "%d.%09d %f\n", t.sec, t.nsec, pos);
    if (!(count++ % 100))
      printf("%f\t", pos);
  }
  printf("\n");
  mot->stop();
  fclose(mot_log);
  bail = true;
  pthread_join(urg_thread, NULL);
  stop();
  close_keyboard();
  return 0;
}

