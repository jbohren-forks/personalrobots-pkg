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

#include "hokuyo_utm.h"

#include <time.h>

#if POSIX_TIMERS <= 0
#include <sys/time.h>
#endif


//! Macro for throwing an exception with a message
#define UTM_EXCEPT(except, msg) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "hokuyo_utm::Laser::%s: " msg, __FUNCTION__); \
    throw except(buf); \
  }

//! Macro for throwing an exception with a message, passing args
#define UTM_EXCEPT_ARGS(except, msg, ...) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "hokuyo_utm::laser::%s: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf); \
  }


//! Helper function for querying the system time
static unsigned long long timeHelper()
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
hokuyo_utm::Laser::Laser() :
                      dmin_(0), dmax_(0), ares_(0), amin_(0), amax_(0), afrt_(0), rate_(0),
                      wrapped_(0), last_time_(0), offset_(0),
                      laser_port_(NULL), laser_fd_(-1)
{ }


///////////////////////////////////////////////////////////////////////////////
hokuyo_utm::Laser::~Laser ()
{
  if (portOpen())
    close();
}


///////////////////////////////////////////////////////////////////////////////
void
hokuyo_utm::Laser::open(const char * port_name, bool use_serial, int baud)
{
  if (portOpen())
    close();
  
  laser_port_ = fopen(port_name, "r+");
  if (laser_port_ == NULL)
    UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "Failed to open port: %s -- error = %d: %s", port_name, errno, strerror(errno));

  try
  {

    laser_fd_ = fileno (laser_port_);
    if (laser_fd_ == -1)
      UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "Failed to get file descriptor --  error = %d: %s", errno, strerror(errno));

    int bauds[] = {B115200, B57600, B19200};
    
    if (use_serial)
    {
      int i = 0;
      for (i = 0; i < 3; i++) {
        if (changeBaud(bauds[i], baud, 100))
          break;
      }
      if (i == 3)
        UTM_EXCEPT(hokuyo_utm::Exception, "Failed to connect at any baud rate");
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
      tcflush (laser_fd_, TCIFLUSH);
      tcsetattr (laser_fd_, TCSANOW, &newtio);
      usleep (200000);
    }

    // Just in case a previous failure mode has left our Hokuyo
    // spewing data, we send the TM2 and QT commands to be safe.
    utmFlush();
    sendCmd("TM2", 1000);
    sendCmd("QT", 1000);
    utmFlush();

    querySensorConfig();

  }
  catch (hokuyo_utm::Exception& e)
  {
    // These exceptions mean something failed on open and we should close
    if (laser_port_ != NULL)
      fclose(laser_port_);
    laser_port_ = NULL;
    laser_fd_ = -1;
    throw e;
  }
}


///////////////////////////////////////////////////////////////////////////////
void
hokuyo_utm::Laser::close ()
{
  int retval = 0;

  if (portOpen()) {
    //Try to be a good citizen and turn off the laser before we shutdown communication
    try
    {
      sendCmd("QT",1000);
      utmFlush();
    }
    catch (hokuyo_utm::Exception& e) {
      //Exceptions here can be safely ignored since we are closing the port anyways
    }

    retval = fclose(laser_port_);
  }

  laser_port_ = NULL;
  laser_fd_ = -1;

  if (retval != 0)
    UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
}


///////////////////////////////////////////////////////////////////////////////
int
hokuyo_utm::Laser::sendCmd(const char* cmd, int timeout)
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

  char buf[100]; 

  utmWrite(cmd);
  utmWrite("\n");

  utmReadlineAfter(buf, 100, cmd, timeout);
  utmReadline(buf,100,timeout);

  if (!checkSum(buf,4))
    UTM_EXCEPT(hokuyo_utm::CorruptedDataException, "Checksum failed on status code.");

  buf[2] = 0;
  
  if (buf[0] - '0' >= 0 && buf[0] - '0' <= 9 && buf[1] - '0' >= 0 && buf[1] - '0' <= 9)
    return (buf[0] - '0')*10 + (buf[1] - '0');
  else
    UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "Hokuyo error code returned. Cmd: %s --  Error: %s", cmd, buf);
}


///////////////////////////////////////////////////////////////////////////////
int
hokuyo_utm::Laser::utmWrite(const char* msg)
{
  int retval = fputs(msg, laser_port_);
  if (retval != EOF)
    return retval;
  else
    UTM_EXCEPT(hokuyo_utm::Exception, "fputs failed");
}


///////////////////////////////////////////////////////////////////////////////
int
hokuyo_utm::Laser::utmFlush()
{
  int retval = tcflush(laser_fd_, TCIOFLUSH);
  if (retval != 0)
    UTM_EXCEPT(hokuyo_utm::Exception, "tcflush failed");
  
  return retval;
} 


///////////////////////////////////////////////////////////////////////////////
int 
hokuyo_utm::Laser::utmReadline(char *buf, int len, int timeout)
{
  char* ret;
  int current=0;

  struct pollfd ufd[1];
  int retval;
  ufd[0].fd = laser_fd_;
  ufd[0].events = POLLIN;

  while (current < len - 1)
  {
    if (current > 0)
      if (buf[current-1] == '\n')
	return current;

    if(timeout >= 0)
    {
      if ((retval = poll(ufd, 1, timeout)) < 0)
        UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "poll failed   --  error = %d: %s", errno, strerror(errno));

      if (retval == 0)
        UTM_EXCEPT(hokuyo_utm::TimeoutException, "timeout reached");
    }

    ret = fgets(&buf[current], len-current, laser_port_);

    if (ret != &buf[current])
      UTM_EXCEPT(hokuyo_utm::Exception, "fgets failed");

    current += strlen(&buf[current]);
  }
  UTM_EXCEPT(hokuyo_utm::Exception, "buffer filled without end of line being found");
}


char*
hokuyo_utm::Laser::utmReadlineAfter(char* buf, int len, const char* str, int timeout)
{
  buf[0] = 0;
  char* ind = &buf[0];

  int bytes_read = 0;
  int skipped = 0;

  while ((strncmp(buf, str, strlen(str))) != 0) {
    bytes_read = utmReadline(buf,len,timeout);

    if ((skipped += bytes_read) > MAX_SKIPPED)
      UTM_EXCEPT(hokuyo_utm::Exception, "too many bytes skipped while searching for match");
  }

  return ind += strlen(str);
}


///////////////////////////////////////////////////////////////////////////////
void
hokuyo_utm::Laser::querySensorConfig()
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

  if (sendCmd("PP",1000) != 0)
    UTM_EXCEPT(hokuyo_utm::Exception, "Error requesting configuration information");

  char buf[100];
  char* ind;
    
  ind = utmReadlineAfter(buf,100,"DMIN:",-1);
  sscanf(ind, "%d", &dmin_);
    
  ind = utmReadlineAfter(buf,100,"DMAX:",-1);
  sscanf(ind, "%d", &dmax_);
    
  ind = utmReadlineAfter(buf,100,"ARES:",-1);
  sscanf(ind, "%d", &ares_);
    
  ind = utmReadlineAfter(buf,100,"AMIN:",-1);
  sscanf(ind, "%d", &amin_);
    
  ind = utmReadlineAfter(buf,100,"AMAX:",-1);
  sscanf(ind, "%d", &amax_);
    
  ind = utmReadlineAfter(buf,100,"AFRT:",-1);
  sscanf(ind, "%d", &afrt_);
    
  ind = utmReadlineAfter(buf,100,"SCAN:",-1);
  sscanf(ind, "%d", &rate_);
    
  return;
}

///////////////////////////////////////////////////////////////////////////////
bool
hokuyo_utm::Laser::changeBaud (int curr_baud, int new_baud, int timeout)
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

  struct termios newtio;
  int fd;
  fd = fileno (laser_port_);

  if (tcgetattr (fd, &newtio) < 0)
    UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "tcgetattr failed  --  error = %d: %s\n", errno, strerror(errno));

  cfmakeraw (&newtio);
  cfsetispeed (&newtio, curr_baud);
  cfsetospeed (&newtio, curr_baud);

  if (tcsetattr (fd, TCSAFLUSH, &newtio) < 0 )
    UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "tcsetattr failed  --  error = %d: %s\n", errno, strerror(errno));

  char buf[100];
  memset (buf,0,sizeof (buf));
  
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
    UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "unknown baud rate: %d",new_baud);
  }
  
  try
  {
    if (sendCmd(buf, timeout) != 0) {
      return false;
    }
  } catch (hokuyo_utm::TimeoutException& e) { }

  if (tcgetattr (fd, &newtio) < 0)
    UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "tcgetattr failed  --  error = %d: %s\n", errno, strerror(errno));

  cfmakeraw (&newtio);
  cfsetispeed (&newtio, new_baud);
  cfsetospeed (&newtio, new_baud);

  if (tcsetattr (fd, TCSAFLUSH, &newtio) < 0 )
    UTM_EXCEPT_ARGS(hokuyo_utm::Exception, "tcsetattr failed  --  error = %d: %s\n", errno, strerror(errno));

  usleep (200000);
  return (0);

}


bool
hokuyo_utm::Laser::checkSum(const char* buf, int buf_len)
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
hokuyo_utm::Laser::readTime(int timeout)
{
  char buf[100];

  utmReadline(buf, 100, timeout);
  if (!checkSum(buf, 6))
    UTM_EXCEPT(hokuyo_utm::CorruptedDataException, "Checksum failed on time stamp.");

  unsigned int urg_time = ((buf[0]-0x30) << 18) | ((buf[1]-0x30) << 12) | ((buf[2]-0x30) << 6) | (buf[3] - 0x30);

  if (urg_time == last_time_)
    fprintf(stderr, "This timestamp is same as the last timestamp.\nSomething is probably going wrong. Try decreasing data rate.");
  else if (urg_time < last_time_)
    wrapped_++;
  
  last_time_ = urg_time;
  
  return (unsigned long long)((wrapped_ << 24) | urg_time)*(unsigned long long)(1000000);
}

void
hokuyo_utm::Laser::readData(hokuyo_utm::LaserScan* scan, bool has_intensity, int timeout)
{
  scan->num_readings = 0;

  int data_size = 3;
  if (has_intensity)
    data_size = 6;

  char buf[100];

  int ind = 0;

  scan->self_time_stamp = readTime(timeout);

  int bytes;

  for (;;)
  {
    bytes = utmReadline(&buf[ind], 100 - ind, timeout);
    
    if (bytes == 1)          // This is \n\n so we should be done
      return;
    
    if (!checkSum(&buf[ind], bytes))
      UTM_EXCEPT(hokuyo_utm::CorruptedDataException, "Checksum failed on data read.");
    
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
        UTM_EXCEPT(hokuyo_utm::CorruptedDataException, "Got more readings than expected");
    }
    // Shuffle remaining bytes to front of buffer to get them on the next loop
    ind = 0;
    for (int j = bytes - (bytes % data_size); j < bytes ; j++)
      buf[ind++] = buf[j];
  }
}


///////////////////////////////////////////////////////////////////////////////
int
hokuyo_utm::Laser::pollScan(hokuyo_utm::LaserScan* scan, double min_ang, double max_ang, int cluster, int timeout)
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

  int status;

  // Always set num_readings to 0 so we can return easily in case of erro
  scan->num_readings = 0;

  if (cluster == 0)
    cluster = 1;
  
  int min_i = (int)(afrt_ + min_ang*ares_/(2.0*M_PI));
  int max_i = (int)(afrt_ + max_ang*ares_/(2.0*M_PI));
  
  char cmdbuf[MAX_CMD_LEN];
  
  sprintf(cmdbuf,"GD%.4d%.4d%.2d", min_i, max_i, cluster);
  
  status = sendCmd(cmdbuf, timeout);
  
  scan->system_time_stamp = timeHelper() + offset_;
  
  if (status != 0)
    return status;
  
  // Populate configuration
  scan->config.min_angle  =  (min_i - afrt_) * (2.0*M_PI)/(ares_);
  scan->config.max_angle  =  (max_i - afrt_) * (2.0*M_PI)/(ares_);
  scan->config.ang_increment =  cluster*(2.0*M_PI)/(ares_);
  scan->config.time_increment = (60.0)/(double)(rate_ * ares_);
  scan->config.scan_time = 0.0;
  scan->config.min_range  =  dmin_ / 1000.0;
  scan->config.max_range  =  dmax_ / 1000.0;
  
  readData(scan, false, timeout);
  
  long long inc = (long long)(min_i * scan->config.time_increment * 1000000000);
  
  scan->system_time_stamp += inc;
  scan->self_time_stamp += inc;
  
  return 0;
}

int
hokuyo_utm::Laser::laserOn() {
  int res = sendCmd("BM",1000);
  if (res == 1)
    UTM_EXCEPT(hokuyo_utm::Exception, "Unable to control laser due to malfunction");
  return res;
}

int
hokuyo_utm::Laser::laserOff() {
  return sendCmd("QT",1000);
}

int
hokuyo_utm::Laser::stopScanning() {
  return laserOff();
}

///////////////////////////////////////////////////////////////////////////////
int
hokuyo_utm::Laser::requestScans(bool intensity, double min_ang, double max_ang, int cluster, int skip, int count, int timeout)
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

  int status;

  if (cluster == 0)
    cluster = 1;
  
  int min_i = (int)(afrt_ + min_ang*ares_/(2.0*M_PI));
  int max_i = (int)(afrt_ + max_ang*ares_/(2.0*M_PI));
  
  char cmdbuf[MAX_CMD_LEN];
  
  char intensity_char = 'D';
  if (intensity)
    intensity_char = 'E';
  
  sprintf(cmdbuf,"M%c%.4d%.4d%.2d%.1d%.2d", intensity_char, min_i, max_i, cluster, skip, count);
  
  status = sendCmd(cmdbuf, timeout);
  
  return status;
}


int
hokuyo_utm::Laser::serviceScan(hokuyo_utm::LaserScan* scan, int timeout)
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

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
    ind = utmReadlineAfter(buf, 100, "M",timeout);
    scan->system_time_stamp = timeHelper() + offset_;

    if (ind[0] == 'D')
      intensity = false;
    else if (ind[0] == 'E')
      intensity = true;
    else
      continue;

    ind++;

    sscanf(ind, "%4d%4d%2d%1d%2d", &min_i, &max_i, &cluster, &skip, &left);  
    utmReadline(buf,100,timeout);

    buf[4] = 0;

    if (!checkSum(buf, 4))
      UTM_EXCEPT_ARGS(hokuyo_utm::CorruptedDataException, "Checksum failed on status code: %s", buf);

    sscanf(buf, "%2d", &status);

    if (status != 99)
      return status;
    
  } while(status != 99);

  scan->config.min_angle  =  (min_i - afrt_) * (2.0*M_PI)/(ares_);
  scan->config.max_angle  =  (max_i - afrt_) * (2.0*M_PI)/(ares_);
  scan->config.ang_increment =  cluster*(2.0*M_PI)/(ares_);
  scan->config.time_increment = (60.0)/(double)(rate_ * ares_);
  scan->config.scan_time = (60.0 * (skip + 1))/((double)(rate_));
  scan->config.min_range  =  dmin_ / 1000.0;
  scan->config.max_range  =  dmax_ / 1000.0;

  readData(scan, intensity, timeout);

  long long inc = (long long)(min_i * scan->config.time_increment * 1000000000);

  scan->system_time_stamp += inc;
  scan->self_time_stamp += inc;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
std::string 
hokuyo_utm::Laser::getID()
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

  if (sendCmd("VV",1000) != 0)
    UTM_EXCEPT(hokuyo_utm::Exception, "Error requesting version information");
  
  char buf[100];
  char* serial = utmReadlineAfter(buf, 100, "SERI:");

  std::string seristring(serial);
  seristring = std::string("H") + seristring.substr(1,seristring.length() - 4);

  return seristring;
}


//////////////////////////////////////////////////////////////////////////////
std::string
hokuyo_utm::Laser::getStatus()
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

  if (sendCmd("II",1000) != 0)
    UTM_EXCEPT(hokuyo_utm::Exception, "Error requesting device information information");
  
  char buf[100];
  char* stat = utmReadlineAfter(buf, 100, "STAT:");

  std::string statstr(stat);
  statstr = statstr.substr(0,statstr.length() - 3);

  return statstr;
}


//////////////////////////////////////////////////////////////////////////////
long long
hokuyo_utm::Laser::calcLatency(bool intensity, double min_ang, double max_ang, int clustering, int skip, int num, int timeout)
{
  if (!portOpen())
    UTM_EXCEPT(hokuyo_utm::Exception, "Port not open.");

  offset_ = 0;

  unsigned long long comp_time = 0;
  unsigned long long urg_time = 0;
  long long diff_time = 0;
  long long drift_time = 0;
  long long tmp_offset1 = 0;
  long long tmp_offset2 = 0;

  int count = 0;
 
  sendCmd("TM0",timeout);
  count = 100;

  for (int i = 0; i < count;i++)
  {
    usleep(1000);
    sendCmd("TM1",timeout);
    comp_time = timeHelper();
    urg_time = readTime();

    diff_time = comp_time - urg_time;

    tmp_offset1 += diff_time / count;
  }

  unsigned long long start_time = timeHelper();
  usleep(5000000);
  sendCmd("TM1;a",timeout);
  sendCmd("TM1;b",timeout);
  comp_time = timeHelper();
  drift_time = comp_time - start_time;
  urg_time = readTime() + tmp_offset1;
  diff_time = comp_time - urg_time;
  double drift_rate = double(diff_time) / double(drift_time);

  sendCmd("TM2",timeout);
  
  if (requestScans(intensity, min_ang, max_ang, clustering, skip, num, timeout) != 0)
    UTM_EXCEPT(hokuyo_utm::Exception, "Error requesting scans during latency calculation");

  hokuyo_utm::LaserScan scan;

  count = 200;
  for (int i = 0; i < count;i++)
  {
    try
    {
      serviceScan(&scan, 1000);
    } catch (hokuyo_utm::CorruptedDataException &e) {
      continue;
    }

    comp_time = scan.system_time_stamp;
    drift_time = comp_time - start_time;
    urg_time = scan.self_time_stamp + tmp_offset1 + (long long)(drift_time*drift_rate);
    diff_time = urg_time - comp_time;

    tmp_offset2 += diff_time / count;
  }

  offset_ = tmp_offset2;

  stopScanning();

  return offset_;
}
