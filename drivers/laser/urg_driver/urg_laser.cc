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

#define URG_ERR(err_code, msg) {printf("urg_laser::%s: " msg "\n", __FUNCTION__); return err_code;}
#define URG_ERR_ARGS(err_code, msg, ...) {printf("urg_laser::%s: " msg "\n", __FUNCTION__, __VA_ARGS__); return err_code;}

///////////////////////////////////////////////////////////////////////////////
urg_laser::urg_laser ()
{
  // Defaults to SCIP version 1
  SCIP_version = 1;
  laser_port   = NULL;
  laser_fd     = -1;
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
    URG_ERR_ARGS(-1, "Failed to open Port: %s error = %d: %s\n", port_name, errno, strerror(errno));

  laser_fd = fileno (laser_port);
  if (laser_fd == -1)
    URG_ERR_ARGS(-1, "Failed to get file descriptor: error = %d:%s\n", errno, strerror(errno));

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
  urg_cmd("QT", 1000);
  usleep(200000);
  urg_flush();

  if (query_SCIP_version() < 0) {
    close();
    URG_ERR(-1,"Failed to find SCIP information");
  }

  if (query_sensor_config() < 0) {
    close();
    URG_ERR(-1,"Failed to Query sensor configuration");
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
  char buf[100]; 

  if (urg_write(cmd) < 0)
    URG_ERR(-1, " write error");

  if (urg_write("\n") < 0)
    URG_ERR(-1, " write error");

  if (urg_readline_after(buf, 100, cmd, timeout) == NULL)
    URG_ERR_ARGS(-1, "CMD: '%s' readline failed while looking for command echo", cmd);

  if (urg_readline(buf,100,timeout) < 0)
    URG_ERR_ARGS(-1, "CMD: '%s' readline failed while looking for status", cmd);

  if (check_sum(buf,4) != 0)
    URG_ERR_ARGS(-1, "CMD: '%s' checksum of status failed.  %s", cmd, buf);

  if (buf[0] - '0' >= 0 && buf[0] - '0' <= 9 && buf[1] - '0' >= 0 && buf[1] - '0' <= 9)
    return (buf[0] - '0')*10 + (buf[1] - '0');
  else
    URG_ERR_ARGS(100 + buf[0] - 'A', "CMD: '%s' invalid command.  Error code: %s", cmd, buf);

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
    URG_ERR(-1," port not open");

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
	URG_ERR_ARGS(-1, " poll error %d %s", errno, strerror(errno));

      if (retval == 0)
	URG_ERR(-1, " timed out on readline");
    }

    ret = fgets(&buf[current], len-current, laser_port);

    if (ret != &buf[current])
      URG_ERR(-1, " fgets returned error");

    current += strlen(&buf[current]);
  }
  URG_ERR_ARGS(-1, " buffer size reached without finding line termination: |%s|", buf);
}


char*
urg_laser::urg_readline_after(char* buf, int len, const char* str, int timeout)
{
  buf[0] = 0;
  char* ind = NULL;

  int bytes_read = 0;
  int skipped = 0;

  while ((ind = strstr(buf, str)) == NULL) {
    if ((bytes_read = urg_readline(buf,len,timeout)) < 0)
      URG_ERR_ARGS(NULL, "failed on readline while searching for: '%s'", str);

    if ((skipped += bytes_read) > MAX_SKIPPED)
      URG_ERR_ARGS(NULL, "skipped too many bytes while searching for: '%s'", str);
  }

  return ind += strlen(str);
}



///////////////////////////////////////////////////////////////////////////////
int
urg_laser::urg_scanf(const char* fmt, ...) {
  if (!port_open())
    URG_ERR(-1, " port not open");

  va_list argList;
  va_start(argList,fmt);
  
  int retval = vfscanf(laser_port, fmt, argList);

  va_end(argList);
  
  return retval;
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
    URG_ERR(-1, "No response to Version query for SCIP2.0 or SCIP1.1");
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
      URG_ERR(-1, "Error checking version number\n");

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
      URG_ERR(-1, "Error requesting configuration information\n");


    char buf[100];

    // Skip over model.
    urg_readline(buf,100,-1);

    urg_scanf("DMIN:%d;",&dmin);
    urg_readline(buf,100,-1);

    urg_scanf("DMAX:%d;",&dmax);
    urg_readline(buf,100,-1);

    urg_scanf("ARES:%d;",&ares);
    urg_readline(buf,100,-1);

    urg_scanf("AMIN:%d;",&amin);
    urg_readline(buf,100,-1);

    urg_scanf("AMAX:%d;",&amax);
    urg_readline(buf,100,-1);

    urg_scanf("AFRT:%d;",&afrt);
    urg_readline(buf,100,-1);

    urg_scanf("SCAN:%d;",&scan);
    urg_readline(buf,100,-1);

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
      URG_ERR(-1, "urg_laser::change_baud: failed to change baud rate");

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

///////////////////////////////////////////////////////////////////////////////
int
urg_laser::get_readings(urg_laser_readings_t* res, double min_ang, double max_ang, int cluster, int timeout)
{
  int status;

  // Always set num_readings to 0 so we can return easily in case of erro
  res->num_readings = 0;

  if (!port_open ())
    URG_ERR(-1, "Port is not open\n");


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

    if (status != 0)
      URG_ERR(status, "Status not 0");
 
    //Data should never be more than 64 bytes before hitting a \n
    char buf[100];

    // skip over the timestamp
    if (urg_readline(buf, 100, timeout) < 0)
      return -1;

    // we use ind to cope with data spanning data boundaries
    int ind = 0;
    int i = 0;

    // Populate configuration
    res->config.min_angle  =  (min_i - afrt) * (2.0*M_PI)/(ares);
    res->config.max_angle  =  (max_i - afrt) * (2.0*M_PI)/(ares);
    res->config.resolution =  cluster*(2.0*M_PI)/(ares);
    res->config.max_range  =  dmax / 1000.0;

    for (;;)
    {
      int bytes = urg_readline(&buf[ind], 100 - ind, timeout);
      
      if (bytes < 0)
	return -1;

      if (bytes == 1)          // This is \n\n so we should be done
        return status;

      if (check_sum(&buf[ind], bytes) != 0)
	return -1;

      bytes += ind - 2;
      
      // Read as many ranges as we can get
      for (int j = 0; j < bytes - (bytes % 3); j+=3)
      {
        if (i < MAX_READINGS)
        {
          res->ranges[i++] = ((buf[j]-0x30) << 12) | ((buf[j+1]-0x30) << 6) | (buf[j+2]-0x30);
          res->num_readings++;
        }
        else
          URG_ERR(-1,"Got too many readings.");
      }
	// Shuffle remaining bytes to front of buffer to get them on the next loop
      ind = 0;
      for (int j = bytes - (bytes % 3); j < bytes ; j++)
	buf[ind++] = buf[j];
    }
  }

  return -1;
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
      URG_ERR(-1, "V command failed");

  if (SCIP_version == 2)
    if (urg_cmd("VV") != 0)
      URG_ERR(-1, "VV command failed");
  
  char buf[100];
  char* num = urg_readline_after(buf, 100, "SERI:");

  if (num == NULL)
    URG_ERR(-1, "'SERI:' field could not be found");

  sscanf(num, "%d", &id);

  return id;
}
