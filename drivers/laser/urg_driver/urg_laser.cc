/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008  Willow Garage
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <stdarg.h>

#include "urg_laser.h"

#include <time.h>
#if POSIX_TIMERS <= 0
#include <sys/time.h>
#endif

unsigned long long time_helper()
{
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (unsigned long long)(curtime.tv_sec) * 1000000000 + (unsigned long long)(curtime.tv_nsec);  
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return (unsigned long long)(timeofday.tv_sec) * 1000000000 + (unsigned long long)(timeofday.tv_usec) * 1000;  
#endif
}

#define URG_WARN(msg) printf("urg_laser::%s: " msg "\n", __FUNCTION__)
#define URG_WARN_ARGS(msg, ...) printf("urg_laser::%s: " msg "\n", __FUNCTION__, __VA_ARGS__)
#define URG_RET_ERR(err_code, msg) {printf("urg_laser::%s: " msg "\n", __FUNCTION__); return err_code;}
#define URG_RET_ERR_ARGS(err_code, msg, ...) {printf("urg_laser::%s: " msg "\n", __FUNCTION__, __VA_ARGS__); return err_code;}

///////////////////////////////////////////////////////////////////////////////
urg_laser::urg_laser ()
{
  // Defaults to SCIP version 1
  SCIP_version = 1;
  laser_port   = NULL;
  laser_fd     = -1;

  dmin = 0;
  dmax = 0;
  ares = 0;
  amin = 0;
  amin = 0;
  amax = 0;
  afrt = 0;
  rate = 0;

  wrapped = 0;

  last_time = 0;
}

///////////////////////////////////////////////////////////////////////////////
urg_laser::~urg_laser ()
{
  if (port_open())
    close();
}


///////////////////////////////////////////////////////////////////////////////
int 
urg_laser::open(const char * port_name, bool use_serial, int baud)
{
  if (port_open())
    close();

  laser_port = fopen(port_name, "r+");
  if (laser_port == NULL)
    URG_RET_ERR_ARGS(-1, "Failed to open Port: %s error = %d: %s\n", port_name, errno, strerror(errno));

  laser_fd = fileno (laser_port);
  if (laser_fd == -1)
    URG_RET_ERR_ARGS(-1, "Failed to get file descriptor: error = %d:%s\n", errno, strerror(errno));

  if (use_serial)
  {
    printf("Trying to connect at 19200");
    if (this->change_baud(B19200, baud, 100) != 0)
    {
      printf("Trying to connect at 57600");
      if (this->change_baud(B57600, baud, 100) != 0)
      {
        printf("Trying to connect at 115200");
        if (this->change_baud(B115200, baud, 100) != 0)
        {
          printf("failed to connect at any baud");
          close();
          return -1;
        }
      }
    }
    printf("Successfully changed baud rate");
  }
  else
  {
    // Settings for USB?
    struct termios newtio;
    memset (&newtio, 0, sizeof (newtio));
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;

    // activate new settings
    tcflush (laser_fd, TCIFLUSH);
    tcsetattr (laser_fd, TCSANOW, &newtio);
    usleep (200000);
  }


  // Just in case a previous failure mode has left our Hokuyo
  // spewing data, we try sending the QT command.
  urg_flush();
  //  urg_write("\n");
  urg_cmd("TM2");
  urg_cmd("QT", 1000);
  urg_flush();

  if (query_SCIP_version() < 0) {
    close();
    URG_RET_ERR(-1,"Failed to find SCIP information");
  }

  if (query_sensor_config() < 0) {
    close();
    URG_RET_ERR(-1,"Failed to Query sensor configuration");
  }

  return 0;
}


///////////////////////////////////////////////////////////////////////////////
int
urg_laser::close ()
{
  int retval = 0;

  if (port_open()) {
    //Try to be a good citizen and turn off the laser before we shutdown communication
    if (SCIP_version == 2)
      urg_cmd("QT",1000);
    if (SCIP_version == 1)
      urg_cmd1("L0",1000);
    urg_flush();
    if (fclose(laser_port) != 0) {
      retval = -1;
    }

    laser_port = NULL;
    laser_fd = -1;
  }

  return retval;
}

///////////////////////////////////////////////////////////////////////////////
int urg_laser::urg_cmd(const char* cmd, int timeout)
{
  if (!port_open())
    URG_RET_ERR(-1," port not open");

  char buf[100]; 

  if (urg_write(cmd) < 0)
    URG_RET_ERR(-1, " write error");

  if (urg_write("\n") < 0)
    URG_RET_ERR(-1, " write error");

  if (urg_readline_after(buf, 100, cmd, timeout) == NULL)
    URG_RET_ERR_ARGS(-1, "CMD: '%s' readline failed while looking for command echo", cmd);

  if (urg_readline(buf,100,timeout) < 0)
    URG_RET_ERR_ARGS(-1, "CMD: '%s' readline failed while looking for status", cmd);

  if (check_sum(buf,4) != 0)
    URG_RET_ERR_ARGS(-1, "CMD: '%s' checksum of status failed.  %s", cmd, buf);

  if (buf[0] - '0' >= 0 && buf[0] - '0' <= 9 && buf[1] - '0' >= 0 && buf[1] - '0' <= 9)
    return (buf[0] - '0')*10 + (buf[1] - '0');
  else
    URG_RET_ERR_ARGS(100 + buf[0] - 'A', "CMD: '%s' invalid command.  Error code: %s", cmd, buf);

}

///////////////////////////////////////////////////////////////////////////////
int urg_laser::urg_cmd1(const char* cmd, int timeout) {

  return -1;

}

///////////////////////////////////////////////////////////////////////////////
int urg_laser::urg_write(const char* msg)
{
  if (fputs(msg, laser_port) != EOF)
    return 0;
  else
    return -1;
}


///////////////////////////////////////////////////////////////////////////////
int urg_laser::urg_flush()
{
  return tcflush(laser_fd, TCIOFLUSH);
} 


///////////////////////////////////////////////////////////////////////////////
int 
urg_laser::urg_readline(char *buf, int len, int timeout)
{
  if (!port_open())
    URG_RET_ERR(-1," port not open");

  char* ret;
  int current=0;

  struct pollfd ufd[1];
  int retval;
  ufd[0].fd = laser_fd;
  ufd[0].events = POLLIN;

  while (current < len - 1)
  {
    if (current > 0)
      if (buf[current-1] == '\n')
	return current;

    if(timeout >= 0)
    {
      if ((retval = poll(ufd, 1, timeout)) < 0)
	URG_RET_ERR_ARGS(-1, " poll error %d %s", errno, strerror(errno));

      if (retval == 0)
	URG_RET_ERR(-1, " timed out on readline");
    }

    ret = fgets(&buf[current], len-current, laser_port);

    if (ret != &buf[current])
      URG_RET_ERR(-1, " fgets returned error");

    current += strlen(&buf[current]);
  }
  URG_RET_ERR(-1, " buffer size reached without finding line termination.");
}


char*
urg_laser::urg_readline_after(char* buf, int len, const char* str, int timeout)
{
  buf[0] = 0;
  char* ind = &buf[0];

  int bytes_read = 0;
  int skipped = 0;

  while ((strncmp(buf, str, strlen(str))) != 0) {
    if ((bytes_read = urg_readline(buf,len,timeout)) < 0)
      URG_RET_ERR_ARGS(NULL, "failed on readline while searching for: '%s'", str);

    if ((skipped += bytes_read) > MAX_SKIPPED)
      URG_RET_ERR_ARGS(NULL, "skipped too many bytes while searching for: '%s'", str);
  }

  return ind += strlen(str);
}

///////////////////////////////////////////////////////////////////////////////
int
urg_laser::query_SCIP_version ()
{
  /////////////////
  // According to Hokuyo Documentation, SCIP1.0 will ignore invalid
  // commands.  SCIP2.0 may or may not respond to SCIP1.0 commands.
  // For backwards compatibility, we first try the VV (SCIP2.0)
  // command with a timeout.
  /////////////////

  //Default to 1
  SCIP_version = 1;

  if (urg_cmd("VV",1000) == 0)
  {
    SCIP_version = 2;
  }
  else if (urg_cmd1("V",1000) == 0)
  {
    // Device is only SCIP1.0 compliant
    char buf[100];

    char* num = urg_readline_after(buf, 100, "FIRM:");

    if (num == NULL)
      return 0;

    num[1] = 0;
    int firmware = atoi(num);
    if (firmware >= 3)
      if (urg_cmd1("SCIP2.0\n") == 0)
	SCIP_version = 2;

  }
  else
  {
    URG_RET_ERR(-1, "No response to Version query for SCIP2.0 or SCIP1.1");
  }
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
int
urg_laser::query_sensor_config()
{
  //////////////
  // SCIP1.0
  //////////////
  if (SCIP_version == 1)
  {
    if (urg_cmd1("V") != 0)
      URG_RET_ERR(-1, "Error checking version number\n");

    // If someone can give me the definition of how this information
    // might is encoded in the V response, I can pull it out, but
    // didn't want to reverse engineer the parsing that was being done
    // by the previous version of this code.
    return 0;
  }

  //////////////
  // SCIP2.0
  //////////////
  else if(SCIP_version == 2)
  {
    if (urg_cmd("PP") != 0)
      URG_RET_ERR(-1, "Error requesting configuration information\n");


    char buf[100];
    char* ind;

    ind = urg_readline_after(buf,100,"DMIN:",-1);
    sscanf(ind, "%d", &dmin);

    ind = urg_readline_after(buf,100,"DMAX:",-1);
    sscanf(ind, "%d", &dmax);

    ind = urg_readline_after(buf,100,"ARES:",-1);
    sscanf(ind, "%d", &ares);

    ind = urg_readline_after(buf,100,"AMIN:",-1);
    sscanf(ind, "%d", &amin);

    ind = urg_readline_after(buf,100,"AMAX:",-1);
    sscanf(ind, "%d", &amax);

    ind = urg_readline_after(buf,100,"AFRT:",-1);
    sscanf(ind, "%d", &afrt);

    ind = urg_readline_after(buf,100,"SCAN:",-1);
    sscanf(ind, "%d", &rate);

    return 0;
  }
  else
  {
    // Unknown SCIP.
    return -1;
  }
}


///////////////////////////////////////////////////////////////////////////////
int 
urg_laser::change_baud (int curr_baud, int new_baud, int timeout)
{
  struct termios newtio;
  int fd;
  fd = fileno (laser_port);

  if (tcgetattr (fd, &newtio) < 0)
  {
    perror ("urg_laser::change_baud:tcgetattr():");
    close();
    return (-1);
  }
  cfmakeraw (&newtio);
  cfsetispeed (&newtio, curr_baud);
  cfsetospeed (&newtio, curr_baud);

  if (tcsetattr (fd, TCSAFLUSH, &newtio) < 0 )
  {
    perror ("urg_laser::change_baud:tcsetattr():");
    close();
    return (-1);
  }

  char buf[100];
  memset (buf,0,sizeof (buf));
  
  if (SCIP_version == 1)
  {
    switch (new_baud)
    {
      case B19200:
	sprintf(buf,"%s","S0192000000000");
        break;
      case B57600:
	sprintf(buf,"%s","S0576000000000");
        break;
      case B115200:
	sprintf(buf,"%s","S0576000000000");
        break;
      default:
        printf ("urg_laser::change_baud: unknown baud rate %d\n", new_baud);
        return (-1);
    }
    if (urg_cmd1(buf) != 0) {
      printf("urg_laser::change_baud: failed to change baud rate");
      return -1;
    }
  }
  else if (SCIP_version == 2)
  {
    switch (new_baud)
    {
      case B19200:
	sprintf(buf,"%s","S019200");
        break;
      case B57600:
	sprintf(buf,"%s","S057600");
        break;
      case B115200:
	sprintf(buf,"%s","S115200");
        break;
      default:
        printf ("urg_laser::change_baud: unknown baud rate %d\n", new_baud);
        return (-1);
    }

    if (urg_cmd(buf) != 0)
      URG_RET_ERR(-1, "urg_laser::change_baud: failed to change baud rate");

  }

  if (tcgetattr (fd, &newtio) < 0)
  {
    perror ("urg_laser::change_baud:tcgetattr():");
    close();
    return (-1);
  }

  cfmakeraw (&newtio);
  cfsetispeed (&newtio, new_baud);
  cfsetospeed (&newtio, new_baud);

  if (tcsetattr (fd, TCSAFLUSH, &newtio) < 0 )
  {
    perror ("urg_laser::change_baud:tcsetattr():");
    close();
    return (-1);
  }

  usleep (200000);
  return (0);

}


int urg_laser::check_sum(const char* buf, int buf_len)
{
  char sum = 0;
  for (int i = 0; i < buf_len - 2; i++)
    sum += (unsigned char)(buf[i]);

  if ((sum & 63) + 0x30 == buf[buf_len - 2])
    return 0;
  else
    return -1;
}


unsigned long long urg_laser::read_time(int timeout) {

  char buf[100];

  if (urg_readline(buf, 100, timeout) < 0)
    URG_RET_ERR(0,"Reading timestamp failed");
  
  if (check_sum(buf, 6) != 0)
    URG_RET_ERR(0,"Timestamp checksum not valid");

  unsigned int urg_time = ((buf[0]-0x30) << 18) | ((buf[1]-0x30) << 12) | ((buf[2]-0x30) << 6) | (buf[3] - 0x30);

  if (urg_time == last_time)
    URG_WARN("This timestamp is same as the last timestamp.\n  Something is probably going wrong. Try decreasing data rate.");
  else if (urg_time < last_time)
    wrapped++;

  last_time = urg_time;

  return (unsigned long long)((wrapped << 24) | urg_time)*(unsigned long long)(1000000);

}

int
urg_laser::read_data(urg_laser_scan_t* scan, bool has_intensity, int timeout)
{

  scan->num_readings = 0;

  int data_size = 3;
  if (has_intensity)
    data_size = 6;

  char buf[100];

  int ind = 0;

  scan->self_time_stamp = read_time(timeout);

  if (scan->self_time_stamp == 0)
    URG_RET_ERR(-1, "Reading timestamp failed");
  
  int bytes;

  for (;;)
  {
    bytes = urg_readline(&buf[ind], 100 - ind, timeout);
    
    if (bytes < 0)
      return -1;
    
    if (bytes == 1)          // This is \n\n so we should be done
    {
      return 0;
    }
    
    
    if (check_sum(&buf[ind], bytes) != 0)
      URG_RET_ERR_ARGS(-1,"Check_sum is wrong on %d: %s", bytes, &buf[ind]);
    
    bytes += ind - 2;
    
    // Read as many ranges as we can get
    for (int j = 0; j < bytes - (bytes % data_size); j+=data_size)
    {
      if (scan->num_readings < MAX_READINGS)
      {
        scan->ranges[scan->num_readings] = (((buf[j]-0x30) << 12) | ((buf[j+1]-0x30) << 6) | (buf[j+2]-0x30)) / 1000.0;
	
        if (has_intensity)
	  scan->intensities[scan->num_readings] = (((buf[j+3]-0x30) << 12) | ((buf[j+4]-0x30) << 6) | (buf[j+5]-0x30));
	else
	  scan->intensities[scan->num_readings] = 0;
	
	scan->num_readings++;
      }
      else
	URG_RET_ERR(-1,"Got too many readings.");
    }
    // Shuffle remaining bytes to front of buffer to get them on the next loop
    ind = 0;
    for (int j = bytes - (bytes % data_size); j < bytes ; j++)
      buf[ind++] = buf[j];
  }
}


///////////////////////////////////////////////////////////////////////////////
int
urg_laser::poll_scan(urg_laser_scan_t* scan, double min_ang, double max_ang, int cluster, int timeout)
{
  int status;

  // Always set num_readings to 0 so we can return easily in case of erro
  scan->num_readings = 0;

  if (!port_open ())
    URG_RET_ERR(-1, "Port is not open\n");


  if (SCIP_version == 1)
  {

    status = urg_cmd1("G00076801");

    if (status != 0) {
      return status;
    }

    // Finish SCIP1.1 stuff later.

  }
  else if(SCIP_version == 2)
  {
    if (cluster == 0)
      cluster = 1;

    int min_i = (int)(afrt + min_ang*ares/(2.0*M_PI));
    int max_i = (int)(afrt + max_ang*ares/(2.0*M_PI));
    
    char cmdbuf[MAX_CMD_LEN];

    sprintf(cmdbuf,"GD%.4d%.4d%.2d", min_i, max_i, cluster);

    status = urg_cmd(cmdbuf, timeout);
    
    scan->system_time_stamp = time_helper() + offset;

    if (status != 0)
      URG_RET_ERR_ARGS(status, "Error Status is %d", status);
 
    // Populate configuration
    scan->config.min_angle  =  (min_i - afrt) * (2.0*M_PI)/(ares);
    scan->config.max_angle  =  (max_i - afrt) * (2.0*M_PI)/(ares);
    scan->config.ang_increment =  cluster*(2.0*M_PI)/(ares);
    scan->config.time_increment = (60.0)/(double)(rate * ares);
    scan->config.scan_time = 0.0;
    scan->config.min_range  =  dmin / 1000.0;
    scan->config.max_range  =  dmax / 1000.0;

    if (read_data(scan, false, timeout) != 0)
      URG_RET_ERR(-1, "Failed to read data.")

  long long inc = min_i * scan->config.time_increment * 1000000000;

  scan->system_time_stamp += inc;
  scan->self_time_stamp += inc;

    return 0;
  }

  return -1;
}

int
urg_laser::laser_on() {
  return urg_cmd("BM");
}

int
urg_laser::laser_off() {
  return urg_cmd("QT");
}

int
urg_laser::stop_scanning() {
  return laser_off();
}

///////////////////////////////////////////////////////////////////////////////
int
urg_laser::request_scans(bool intensity, double min_ang, double max_ang, int cluster, int skip, int count, int timeout)
{

  int status;

  // Always set num_readings to 0 so we can return easily in case of erro
  if (!port_open ())
    URG_RET_ERR(-1, "Port is not open\n");

  if (SCIP_version == 2)
  {
    if (cluster == 0)
      cluster = 1;

    int min_i = (int)(afrt + min_ang*ares/(2.0*M_PI));
    int max_i = (int)(afrt + max_ang*ares/(2.0*M_PI));
    
    char cmdbuf[MAX_CMD_LEN];

    char intensity_char = 'D';
    if (intensity)
      intensity_char = 'E';

    sprintf(cmdbuf,"M%c%.4d%.4d%.2d%.1d%.2d", intensity_char, min_i, max_i, cluster, skip, count);

    status = urg_cmd(cmdbuf, timeout);

    return status;
  }

  return -1;
}


int
urg_laser::service_scan(urg_laser_scan_t* scan, int timeout)
{

  scan->num_readings = 0;

  if (!port_open ())
    URG_RET_ERR(-1, "Port is not open\n");

  char buf[100];

  bool intensity = false;
  int min_i;
  int max_i;
  int cluster;
  int skip;
  int left;

  char* ind;

  do {
    ind = urg_readline_after(buf, 100, "M",timeout);
    scan->system_time_stamp = time_helper() + offset;

    if (ind == NULL)
      return -1;

    if (ind[0] == 'D')
      intensity = false;
    else if (ind[0] == 'E')
      intensity = true;
    else
      continue;

    ind++;

    sscanf(ind, "%4d%4d%2d%1d%2d", &min_i, &max_i, &cluster, &skip, &left);  
    urg_readline(buf,100,timeout);

  } while(strcmp(buf,"99b\n"));

  scan->config.min_angle  =  (min_i - afrt) * (2.0*M_PI)/(ares);
  scan->config.max_angle  =  (max_i - afrt) * (2.0*M_PI)/(ares);
  scan->config.ang_increment =  cluster*(2.0*M_PI)/(ares);
  scan->config.time_increment = (60.0)/(double)(rate * ares);
  scan->config.scan_time = (60.0 * (skip + 1))/((double)(rate));
  scan->config.min_range  =  dmin / 1000.0;
  scan->config.max_range  =  dmax / 1000.0;

  if (read_data(scan, intensity, timeout) != 0)
    URG_RET_ERR(-1, "Failed to read data.");

  long long inc = min_i * scan->config.time_increment * 1000000000;

  scan->system_time_stamp += inc;
  scan->self_time_stamp += inc;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
int 
urg_laser::get_ID()
{
  int id = 0;

  if (!port_open ())
    return -1;

  if (SCIP_version == 1)
    if (urg_cmd1("V") != 0)
      URG_RET_ERR(-1, "V command failed");

  if (SCIP_version == 2)
    if (urg_cmd("VV") != 0)
      URG_RET_ERR(-1, "VV command failed");
  
  char buf[100];
  char* num = urg_readline_after(buf, 100, "SERI:");

  if (num == NULL)
    URG_RET_ERR(-1, "'SERI:' field could not be found");

  sscanf(num, "%d", &id);

  return id;
}

int urg_laser::calc_latency(bool intensity, double min_ang, double max_ang, int clustering, int skip, int num, int timeout)
{
  printf("Determining scan latency.\n");
  offset = 0;

  unsigned long long comp_time = 0;
  unsigned long long urg_time = 0;
  long long diff_time = 0;
  long long drift_time = 0;
  long long tmp_offset1 = 0;
  long long tmp_offset2 = 0;

  int count = 0;
 
  // Put into timing mode.
  printf("Putting URG in timing mode.\n");
  urg_cmd("TM0");
  count = 100;
  printf("Estimating constant offset... ");
  for (int i = 0; i < count;i++)
  {
    usleep(1000);
    urg_cmd("TM1");
    comp_time = time_helper();
    urg_time = read_time();

    diff_time = comp_time - urg_time;

    tmp_offset1 += diff_time / count;
  }
  printf("%lld nanoseconds\n", tmp_offset1);
  printf("Estimating drift rate... ");
  unsigned long long start_time = time_helper();
  usleep(5000000);
  urg_cmd("TM1;a");
  urg_cmd("TM1;b");
  comp_time = time_helper();
  drift_time = comp_time - start_time;
  urg_time = read_time() + tmp_offset1;
  diff_time = comp_time - urg_time;
  double drift_rate = double(diff_time) / double(drift_time);
  printf("%g\n", drift_rate);
  printf("Leaving timing mode.\n");
  urg_cmd("TM2");
  
  printf("Putting URG in scan mode.\n");
  if (request_scans(intensity, min_ang, max_ang, clustering, skip, num, timeout) != 0)
    URG_RET_ERR(-1,"Error requesting scans during latency calculation");

  urg_laser_scan_t scan;

  printf("Estimating latency of scans... ");
  count = 200;
  for (int i = 0; i < count;i++)
  {
    service_scan(&scan);

    comp_time = scan.system_time_stamp;
    drift_time = comp_time - start_time;
    urg_time = scan.self_time_stamp + tmp_offset1 + drift_time*drift_rate;
    diff_time = urg_time - comp_time;

    tmp_offset2 += diff_time / count;
  }
  printf("%lld nanoseconds\n", tmp_offset2);

  offset = tmp_offset2;

  stop_scanning();

  return 0;
}
