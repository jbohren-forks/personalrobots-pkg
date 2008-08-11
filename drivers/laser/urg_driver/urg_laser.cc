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
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>

#include "urg_laser.h"

#include <time.h>
#if POSIX_TIMERS <= 0
#include <sys/time.h>
#endif

///////////////////////////////////////////////////////////////////////////////
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


///////////////////////////////////////////////////////////////////////////////
URG::laser::laser() : SCIP_version(1),
                      dmin(0), dmax(0), ares(0), amin(0), amax(0), afrt(0), rate(0),
                      wrapped(0), last_time(0), offset(0),
                      laser_port(NULL), laser_fd(-1)
{ }


///////////////////////////////////////////////////////////////////////////////
URG::laser::~laser ()
{
  if (port_open())
    close();
}


///////////////////////////////////////////////////////////////////////////////
void
URG::laser::open(const char * port_name, bool use_serial, int baud)
{
  if (port_open())
    close();
  
  laser_port = fopen(port_name, "r+");
  if (laser_port == NULL)
    URG_EXCEPT_ARGS(URG::exception, "Failed to open port: %s -- error = %d: %s", port_name, errno, strerror(errno));

  try
  {

    laser_fd = fileno (laser_port);
    if (laser_fd == -1)
      URG_EXCEPT_ARGS(URG::exception, "Failed to get file descriptor --  error = %d: %s", errno, strerror(errno));

    int bauds[] = {B115200, B57600, B19200};
    
    if (use_serial)
    {
      int i = 0;
      for (i = 0; i < 3; i++) {
        if (change_baud(bauds[i], baud, 100))
          break;
      }
      if (i == 3)
        URG_EXCEPT(URG::exception, "Failed to connect at any baud rate");
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
    send_cmd("TM2", 1000);
    send_cmd("QT", 1000);
    urg_flush();

    query_SCIP_version();

    query_sensor_config();

  }
  catch (URG::exception& e)
  {
    // These exceptions mean something failed on open and we should close
    //    close();
    if (laser_port != NULL)
      fclose(laser_port);
    laser_port = NULL;
    laser_fd = -1;
    throw e;
  }
}


///////////////////////////////////////////////////////////////////////////////
void
URG::laser::close ()
{
  int retval = 0;

  if (port_open()) {
    //Try to be a good citizen and turn off the laser before we shutdown communication
    try
    {
      if (SCIP_version == 2)
        send_cmd("QT",1000);
      if (SCIP_version == 1)
        send_cmd1("L0",1000);
      urg_flush();
    }
    catch (URG::exception& e) {
      //Exceptions here can be safely ignored since we are closing the port anyways
    }

    retval = fclose(laser_port);
  }

  laser_port = NULL;
  laser_fd = -1;

  if (retval != 0)
    URG_EXCEPT_ARGS(URG::exception, "Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
}


///////////////////////////////////////////////////////////////////////////////
int
URG::laser::send_cmd(const char* cmd, int timeout)
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  char buf[100]; 

  urg_write(cmd);
  urg_write("\n");

  urg_readline_after(buf, 100, cmd, timeout);
  urg_readline(buf,100,timeout);

  if (!check_sum(buf,4))
    URG_EXCEPT(URG::corrupted_data_exception, "Checksum failed on status code.");

  buf[2] = 0;
  
  if (buf[0] - '0' >= 0 && buf[0] - '0' <= 9 && buf[1] - '0' >= 0 && buf[1] - '0' <= 9)
    return (buf[0] - '0')*10 + (buf[1] - '0');
  else
    URG_EXCEPT_ARGS(URG::exception, "Hokuyo error code returned. Cmd: %s --  Error: %s", cmd, buf);
}


///////////////////////////////////////////////////////////////////////////////
//THIS NEEDS TO BE IMPLEMENTED
int
URG::laser::send_cmd1(const char* cmd, int timeout)
{
  return -1;
}


///////////////////////////////////////////////////////////////////////////////
int
URG::laser::urg_write(const char* msg)
{
  int retval = fputs(msg, laser_port);
  if (retval != EOF)
    return retval;
  else
    URG_EXCEPT(URG::exception, "fputs failed");
}


///////////////////////////////////////////////////////////////////////////////
int
URG::laser::urg_flush()
{
  int retval = tcflush(laser_fd, TCIOFLUSH);
  if (retval != 0)
    URG_EXCEPT(URG::exception, "tcflush failed");
  
  return retval;
} 


///////////////////////////////////////////////////////////////////////////////
int 
URG::laser::urg_readline(char *buf, int len, int timeout)
{
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
        URG_EXCEPT_ARGS(URG::exception, "poll failed   --  error = %d: %s", errno, strerror(errno));

      if (retval == 0)
        URG_EXCEPT(URG::timeout_exception, "timeout reached");
    }

    ret = fgets(&buf[current], len-current, laser_port);

    if (ret != &buf[current])
      URG_EXCEPT(URG::exception, "fgets failed");

    current += strlen(&buf[current]);
  }
  URG_EXCEPT(URG::exception, "buffer filled without end of line being found");
}


char*
URG::laser::urg_readline_after(char* buf, int len, const char* str, int timeout)
{
  buf[0] = 0;
  char* ind = &buf[0];

  int bytes_read = 0;
  int skipped = 0;

  while ((strncmp(buf, str, strlen(str))) != 0) {
    bytes_read = urg_readline(buf,len,timeout);

    if ((skipped += bytes_read) > MAX_SKIPPED)
      URG_EXCEPT(URG::exception, "too many bytes skipped while searching for match");
  }

  return ind += strlen(str);
}

///////////////////////////////////////////////////////////////////////////////
void
URG::laser::query_SCIP_version ()
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  /////////////////
  // According to Hokuyo Documentation, SCIP1.0 will ignore invalid
  // commands.  SCIP2.0 may or may not respond to SCIP1.0 commands.
  // For backwards compatibility, we first try the VV (SCIP2.0)
  // command with a timeout.
  /////////////////

  //Default to 1
  SCIP_version = 1;

  try
  {
    if (send_cmd("VV",1000) == 0)
    {
      SCIP_version = 2;
      return;
    }
  } catch (URG::timeout_exception& e)
  {
    // This is ok since SCIP1.1 devices wont respond to SCIP2.0
    // commands.
  }

  if (send_cmd1("V",1000) == 0)
  {
    // Device is only SCIP1.1 compliant
    char buf[100];

    char* num = urg_readline_after(buf, 100, "FIRM:");

    num[1] = 0;
    int firmware = atoi(num);
    if (firmware >= 3)
      if (send_cmd1("SCIP2.0\n") == 0)
	SCIP_version = 2;

    return;
  }
  else
  {
    URG_EXCEPT(URG::exception, "No response to Version query for SCIP2.0 or SCIP1.1");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
URG::laser::query_sensor_config()
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  //////////////
  // SCIP1.0
  //////////////
  if (SCIP_version == 1)
  {
    if (send_cmd1("V") != 0)
      URG_EXCEPT(URG::exception, "Error checking version number");

    // If someone can give me the definition of how this information
    // might is encoded in the V response, I can pull it out, but
    // didn't want to reverse engineer the parsing that was being done
    // by the previous version of this code.
    return;
  }

  //////////////
  // SCIP2.0
  //////////////
  else if(SCIP_version == 2)
  {
    if (send_cmd("PP",1000) != 0)
      URG_EXCEPT(URG::exception, "Error requesting configuration information");

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
    
    return;
  }
  else
  {
    // This should not happen
    URG_EXCEPT(URG::exception, "Tried to query sensor configuration with an unknown SCIP version\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
bool
URG::laser::change_baud (int curr_baud, int new_baud, int timeout)
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  struct termios newtio;
  int fd;
  fd = fileno (laser_port);

  if (tcgetattr (fd, &newtio) < 0)
    URG_EXCEPT_ARGS(URG::exception, "tcgetattr failed  --  error = %d: %s\n", errno, strerror(errno));

  cfmakeraw (&newtio);
  cfsetispeed (&newtio, curr_baud);
  cfsetospeed (&newtio, curr_baud);

  if (tcsetattr (fd, TCSAFLUSH, &newtio) < 0 )
    URG_EXCEPT_ARGS(URG::exception, "tcsetattr failed  --  error = %d: %s\n", errno, strerror(errno));

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
        URG_EXCEPT_ARGS(URG::exception, "unknown baud rate: %d",new_baud);
    }
    try
    {
      if (send_cmd1(buf, timeout) != 0) {
        return false;
      }
    } catch (URG::timeout_exception& e) { }
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
        URG_EXCEPT_ARGS(URG::exception, "unknown baud rate: %d",new_baud);
    }
    try
    {
      if (send_cmd(buf, timeout) != 0) {
        return false;
      }
    } catch (URG::timeout_exception& e) { }
  }

  if (tcgetattr (fd, &newtio) < 0)
    URG_EXCEPT_ARGS(URG::exception, "tcgetattr failed  --  error = %d: %s\n", errno, strerror(errno));

  cfmakeraw (&newtio);
  cfsetispeed (&newtio, new_baud);
  cfsetospeed (&newtio, new_baud);

  if (tcsetattr (fd, TCSAFLUSH, &newtio) < 0 )
    URG_EXCEPT_ARGS(URG::exception, "tcsetattr failed  --  error = %d: %s\n", errno, strerror(errno));

  usleep (200000);
  return (0);

}


bool
URG::laser::check_sum(const char* buf, int buf_len)
{
  char sum = 0;
  for (int i = 0; i < buf_len - 2; i++)
    sum += (unsigned char)(buf[i]);

  if ((sum & 63) + 0x30 == buf[buf_len - 2])
    return true;
  else
    return false;
}


unsigned long long
URG::laser::read_time(int timeout)
{
  char buf[100];

  urg_readline(buf, 100, timeout);
  if (!check_sum(buf, 6))
    URG_EXCEPT(URG::corrupted_data_exception, "Checksum failed on time stamp.");

  unsigned int urg_time = ((buf[0]-0x30) << 18) | ((buf[1]-0x30) << 12) | ((buf[2]-0x30) << 6) | (buf[3] - 0x30);

  if (urg_time == last_time)
    fprintf(stderr, "This timestamp is same as the last timestamp.\nSomething is probably going wrong. Try decreasing data rate.");
  else if (urg_time < last_time)
    wrapped++;
  
  last_time = urg_time;
  
  return (unsigned long long)((wrapped << 24) | urg_time)*(unsigned long long)(1000000);
}

void
URG::laser::read_data(URG::laser_scan_t* scan, bool has_intensity, int timeout)
{
  scan->num_readings = 0;

  int data_size = 3;
  if (has_intensity)
    data_size = 6;

  char buf[100];

  int ind = 0;

  scan->self_time_stamp = read_time(timeout);

  int bytes;

  for (;;)
  {
    bytes = urg_readline(&buf[ind], 100 - ind, timeout);
    
    if (bytes == 1)          // This is \n\n so we should be done
      return;
    
    if (!check_sum(&buf[ind], bytes))
      URG_EXCEPT(URG::corrupted_data_exception, "Checksum failed on data read.");
    
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
        URG_EXCEPT(URG::corrupted_data_exception, "Got more readings than expected");
    }
    // Shuffle remaining bytes to front of buffer to get them on the next loop
    ind = 0;
    for (int j = bytes - (bytes % data_size); j < bytes ; j++)
      buf[ind++] = buf[j];
  }
}


///////////////////////////////////////////////////////////////////////////////
int
URG::laser::poll_scan(URG::laser_scan_t* scan, double min_ang, double max_ang, int cluster, int timeout)
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  int status;

  // Always set num_readings to 0 so we can return easily in case of erro
  scan->num_readings = 0;

  if (SCIP_version == 1)
  {
    status = send_cmd1("G00076801");

    if (status != 0) {
      return status;
    }

    URG_EXCEPT(URG::exception, "SCIP version 1.1 not implemented yet\n");

  }
  else if(SCIP_version == 2)
  {
    if (cluster == 0)
      cluster = 1;

    int min_i = (int)(afrt + min_ang*ares/(2.0*M_PI));
    int max_i = (int)(afrt + max_ang*ares/(2.0*M_PI));
    
    char cmdbuf[MAX_CMD_LEN];

    sprintf(cmdbuf,"GD%.4d%.4d%.2d", min_i, max_i, cluster);

    status = send_cmd(cmdbuf, timeout);

    scan->system_time_stamp = time_helper() + offset;

    if (status != 0)
      return status;
 
    // Populate configuration
    scan->config.min_angle  =  (min_i - afrt) * (2.0*M_PI)/(ares);
    scan->config.max_angle  =  (max_i - afrt) * (2.0*M_PI)/(ares);
    scan->config.ang_increment =  cluster*(2.0*M_PI)/(ares);
    scan->config.time_increment = (60.0)/(double)(rate * ares);
    scan->config.scan_time = 0.0;
    scan->config.min_range  =  dmin / 1000.0;
    scan->config.max_range  =  dmax / 1000.0;

    read_data(scan, false, timeout);

    long long inc = (long long)(min_i * scan->config.time_increment * 1000000000);
    
    scan->system_time_stamp += inc;
    scan->self_time_stamp += inc;

    return 0;
  }
  else
  {
    // This should not happen
    URG_EXCEPT(URG::exception, "Tried to poll for scan with an unknown SCIP version\n");
  }
}

int
URG::laser::laser_on() {
  int res = send_cmd("BM",1000);
  if (res == 1)
    URG_EXCEPT(URG::exception, "Unable to control laser due to malfunction");
  return res;
}

int
URG::laser::laser_off() {
  return send_cmd("QT",1000);
}

int
URG::laser::stop_scanning() {
  return laser_off();
}

///////////////////////////////////////////////////////////////////////////////
int
URG::laser::request_scans(bool intensity, double min_ang, double max_ang, int cluster, int skip, int count, int timeout)
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  int status;

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

    status = send_cmd(cmdbuf, timeout);

    return status;
  } else {
    URG_EXCEPT(URG::exception, "Tried to poll for scan with an unknown SCIP version");
  }
}


int
URG::laser::service_scan(URG::laser_scan_t* scan, int timeout)
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  // Always set num_readings to 0 so we can return easily in case of error
  scan->num_readings = 0;

  char buf[100];

  bool intensity = false;
  int min_i;
  int max_i;
  int cluster;
  int skip;
  int left;

  char* ind;

  int status = -1;

  do {
    ind = urg_readline_after(buf, 100, "M",timeout);
    scan->system_time_stamp = time_helper() + offset;

    if (ind[0] == 'D')
      intensity = false;
    else if (ind[0] == 'E')
      intensity = true;
    else
      continue;

    ind++;

    sscanf(ind, "%4d%4d%2d%1d%2d", &min_i, &max_i, &cluster, &skip, &left);  
    urg_readline(buf,100,timeout);

    buf[4] = 0;

    if (!check_sum(buf, 4))
      URG_EXCEPT_ARGS(URG::corrupted_data_exception, "Checksum failed on status code: %s", buf);

    sscanf(buf, "%2d", &status);

    if (status != 99)
      return status;
    
  } while(status != 99);

  scan->config.min_angle  =  (min_i - afrt) * (2.0*M_PI)/(ares);
  scan->config.max_angle  =  (max_i - afrt) * (2.0*M_PI)/(ares);
  scan->config.ang_increment =  cluster*(2.0*M_PI)/(ares);
  scan->config.time_increment = (60.0)/(double)(rate * ares);
  scan->config.scan_time = (60.0 * (skip + 1))/((double)(rate));
  scan->config.min_range  =  dmin / 1000.0;
  scan->config.max_range  =  dmax / 1000.0;

  read_data(scan, intensity, timeout);

  long long inc = (long long)(min_i * scan->config.time_increment * 1000000000);

  scan->system_time_stamp += inc;
  scan->self_time_stamp += inc;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
std::string 
URG::laser::get_ID()
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  if (SCIP_version == 1)
    if (send_cmd1("V") != 0)
      URG_EXCEPT(URG::exception, "Error requesting version information");

  if (SCIP_version == 2)
    if (send_cmd("VV",1000) != 0)
      URG_EXCEPT(URG::exception, "Error requesting version information");
  
  char buf[100];
  char* serial = urg_readline_after(buf, 100, "SERI:");

  std::string seristring(serial);
  seristring = std::string("H") + seristring.substr(1,seristring.length() - 4);

  return seristring;
}


//////////////////////////////////////////////////////////////////////////////
std::string
URG::laser::get_status()
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  if (SCIP_version == 2)
    if (send_cmd("II",1000) != 0)
      URG_EXCEPT(URG::exception, "Error requesting device information information");
  
  char buf[100];
  char* stat = urg_readline_after(buf, 100, "STAT:");

  std::string statstr(stat);
  statstr = statstr.substr(0,statstr.length() - 3);

  return statstr;
}


//////////////////////////////////////////////////////////////////////////////
long long
URG::laser::calc_latency(bool intensity, double min_ang, double max_ang, int clustering, int skip, int num, int timeout)
{
  if (!port_open())
    URG_EXCEPT(URG::exception, "Port not open.");

  offset = 0;

  unsigned long long comp_time = 0;
  unsigned long long urg_time = 0;
  long long diff_time = 0;
  long long drift_time = 0;
  long long tmp_offset1 = 0;
  long long tmp_offset2 = 0;

  int count = 0;
 
  // Put into timing mode.
  //  printf("Putting URG in timing mode.\n");
  send_cmd("TM0",timeout);
  count = 100;
  //  printf("Estimating constant offset... ");
  for (int i = 0; i < count;i++)
  {
    usleep(1000);
    send_cmd("TM1",timeout);
    comp_time = time_helper();
    urg_time = read_time();

    diff_time = comp_time - urg_time;

    tmp_offset1 += diff_time / count;
  }
  //  printf("%lld nanoseconds\n", tmp_offset1);
  //  printf("Estimating drift rate... ");
  unsigned long long start_time = time_helper();
  usleep(5000000);
  send_cmd("TM1;a",timeout);
  send_cmd("TM1;b",timeout);
  comp_time = time_helper();
  drift_time = comp_time - start_time;
  urg_time = read_time() + tmp_offset1;
  diff_time = comp_time - urg_time;
  double drift_rate = double(diff_time) / double(drift_time);
  //  printf("%g\n", drift_rate);
  //  printf("Leaving timing mode.\n");
  send_cmd("TM2",timeout);
  
  //    printf("Putting URG in scan mode.\n");
  if (request_scans(intensity, min_ang, max_ang, clustering, skip, num, timeout) != 0)
    URG_EXCEPT(URG::exception, "Error requesting scans during latency calculation");

  URG::laser_scan_t scan;

  //    printf("Estimating latency of scans... ");
  count = 200;
  for (int i = 0; i < count;i++)
  {
    try
    {
      service_scan(&scan, 1000);
    } catch (URG::corrupted_data_exception &e) {
      //      printf("Skipping corrupted data...\n");
      continue;
    }

    comp_time = scan.system_time_stamp;
    drift_time = comp_time - start_time;
    urg_time = scan.self_time_stamp + tmp_offset1 + (long long)(drift_time*drift_rate);
    diff_time = urg_time - comp_time;

    tmp_offset2 += diff_time / count;
  }
  //  printf("%lld nanoseconds\n", tmp_offset2);

  offset = tmp_offset2;

  stop_scanning();

  return offset;
}
