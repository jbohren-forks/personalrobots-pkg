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


#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <poll.h>

//#include <replace/replace.h>

#include "urg_laser.h"

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
  {
    printf("urg_laser::open: Failed to open Port: %s error = %d: %s\n",
            port_name, errno, strerror (errno));
    return -1;
  }

  laser_fd = fileno (laser_port);
  if (laser_fd == -1) {
    printf("urg_laser::open: Failed to get file descriptor: error = %d:%s\n",
	    errno, strerror(errno));
    return -1;
  }

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
  // spewing data, we send the QT command.
  urg_flush();
  urg_cmd("QT");
  usleep(200000);
  urg_flush();

  if (query_SCIP_version() < 0) {
    printf ("urg_laser::open: Failed to find SCIP information\n");
    close();
    return -1;
  }

  if (query_sensor_config() < 0) {
    printf ("urg_laser::open: Failed to Query sensor configuration\n");
    close();
    return -1;
  }

  return 0;
}


///////////////////////////////////////////////////////////////////////////////
int
urg_laser::close ()
{
  int retval = 0;

  if (port_open()) {
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
  char* buf = new char[strlen(cmd)+1];
  sprintf(buf, "%s\n", cmd);

  urg_write(buf);

  if (skip_until(buf, strlen(buf), timeout) < 0) {
    printf ("urg_laser::urg_cmd: skippping until command echo failed\n");
    delete[] buf;
    return -1;
  }
  delete[] buf;

  char buf2[4];
  if (read_until(buf2, 4, "\n", 1, timeout) == 4) {
    int status =  (buf2[0] - '0')*10 + (buf2[1] - '0');
    return status;
  }
  else
    return -1;
}


///////////////////////////////////////////////////////////////////////////////
int urg_laser::urg_cmd1(const char* cmd, int timeout) {

  return -1;

}

///////////////////////////////////////////////////////////////////////////////
int urg_laser::urg_write(const char* msg)
{
  return write(laser_fd, msg, strlen(msg));
}


///////////////////////////////////////////////////////////////////////////////
int urg_laser::urg_flush()
{
  return tcflush(laser_fd, TCIOFLUSH);
} 


///////////////////////////////////////////////////////////////////////////////
int 
urg_laser::urg_read(char *buf, int len, int timeout)
{
  int ret;
  int current=0;
  struct pollfd ufd[1];
  int retval;

  ufd[0].fd = laser_fd;
  ufd[0].events = POLLIN;

  while (current < len)
  {
    if(timeout >= 0)
    {
      if ((retval = poll(ufd, 1, timeout)) < 0)
      {
        perror ("urg_laser::read:poll:");
        return -1;
      }
      else if (retval == 0)
      {
        printf("urg_laser::read: timed out on read\n");
        return (-1);
      }
    }

    ret = read (laser_fd, &buf[current], len-current);
    if (ret < 0)
      return ret;
    
    current += ret;
  }
  return len;
}


///////////////////////////////////////////////////////////////////////////////
int 
urg_laser::skip_until(const char* search, int search_len, int timeout)
{
  int ret;
  struct pollfd ufd[1];
  int retval;

  int ind = 0;
  int skipped = 0;

  ufd[0].fd = laser_fd;
  ufd[0].events = POLLIN;

  char* buf = new char[strlen(search)];

  while (ind < search_len)
  {

    if(timeout >= 0)
    {
      if ((retval = poll(ufd, 1, timeout)) < 0)
      {
        perror ("urg_laser::skip_until:poll:");
        delete[] buf;
        return -1;
      }
      else if (retval == 0)
      {
        printf("urg_laser::skip_until: timed out on read\n");
        delete[] buf;
        return (-1);
      }
    }

    int check_ind = 0;
    int read_len = search_len - ind;

    if (skipped + read_len > MAX_SKIPPED) {
        printf("urg_laser::skip_until: MAX_SKIPPED exceeded\n");
        delete[] buf;
        return -1;
    }

    ret = read (laser_fd, buf, read_len);

    if (ret < 0) {
      delete[] buf;
      return ret;
    }

    skipped += ret;
    
    while (check_ind < read_len) {
      if (buf[check_ind++] == search[ind]) {
	ind++;
      } else {
	ind = 0;
      }
    }
  }
  delete[] buf;
  return skipped;
}

///////////////////////////////////////////////////////////////////////////////
int 
urg_laser::read_until(char* buf, int buf_len, const char* search, int search_len, int timeout)
{
  int ret;
  int current=0;
  struct pollfd ufd[1];
  int retval;

  int ind = 0;

  ufd[0].fd = laser_fd;
  ufd[0].events = POLLIN;

  while (ind < search_len)
  {
    if(timeout >= 0)
    {
      if ((retval = poll (ufd, 1, timeout)) < 0)
      {
        perror ("urg_laser::read_until:poll:");
        return (-1);
      }
      else if (retval == 0)
      {
        printf("urg_laser::read_until: timed out on read\n");
        return (-1);
      }
    }

    int check_ind = current;
    int read_len = search_len - ind;

    if (current + read_len > buf_len) {
      printf("urg_laser::read_until: buffer length exceeded\n");
      return -1;
    }

    ret = read (laser_fd, &buf[current], read_len);
    if (ret < 0) {
      printf("urg_laser::read_until: read failure\n");
      return ret;
    }

    current += ret;
    
    while (check_ind < current) {
      if (buf[check_ind++] == search[ind]) {
	ind++;
      } else {
	ind = 0;
      }
    }
  }

  return current;
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
    char buf[6];
    if (skip_until("FIRM:", 5, 1000) > 0) {    
      // Read the firmware version major value 
      urg_read(buf, 1);
      buf[1] = 0;
      int firmware = atoi(buf);
      if (firmware >= 3)
	if (urg_cmd1("SCIP2.0\n") == 0)
	  SCIP_version = 2;
    } 
    else
    {
      printf("urg_laser:query_SCIP_version: Warning, 'FIRM' field could not be found\n");
    } 
  }
  else
  {
    printf("urg_laser:query_SCIP_version: No response to Version query for SCIP2.0 or SCIP1.1\n");
    urg_flush();
    return -1;
  }
  urg_flush();
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
int
urg_laser::query_sensor_config()
{
    printf("Query sensor config.\n");

  //////////////
  // SCIP1.0
  //////////////
  if (SCIP_version == 1)
  {
    if (urg_cmd1("V") != 0) {
      printf("urg_laser> E: GetSensorConfig: Error checking version number\n");
      urg_flush();
      return -1;
    }

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

    printf("Sending command: PP\n");

    if (urg_cmd("PP") != 0) {
      printf ("urg_laser> E: GetSensorConfig: Error requesting information\n");
      urg_flush();
      return -1;
    }

    char buf[100];
    int len;
    
    // read min range
    skip_until("DMIN:", 5, -1);
    len = read_until(buf, 100, ";", 1, -1);
    buf[len-1] = 0;
    dmin = atoi(buf);

    // read max range
    skip_until("DMAX:", 5, -1);
    len = read_until(buf, 100, ";", 1, -1);
    buf[len-1] = 0;
    dmax = atoi(buf);

    // read angular resolution
    skip_until("ARES:", 5, -1);
    len = read_until(buf, 100, ";", 1, -1);
    buf[len-1] = 0;
    ares = atoi(buf);

    // read min angle
    skip_until("AMIN:", 5, -1);
    len = read_until(buf, 100, ";", 1, -1);
    buf[len-1] = 0;
    amin = atoi(buf);

    // read min angle
    skip_until("AMAX:", 5, -1);
    len = read_until(buf, 100, ";", 1, -1);
    buf[len-1] = 0;
    amax = atoi(buf);

    // read front angle
    skip_until("AFRT:", 5, -1);
    len = read_until(buf, 100, ";", 1, -1);
    buf[len-1] = 0;
    afrt = atoi(buf);

    // read scan rage
    skip_until("SCAN:", 5, -1);
    len = read_until(buf, 100, ";", 1, -1);
    buf[len-1] = 0;
    scan = atoi(buf);
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
    if (urg_cmd(buf) != 0) {
      printf("urg_laser::change_baud: failed to change baud rate");
      return -1;
    }
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


///////////////////////////////////////////////////////////////////////////////
int
urg_laser::get_readings(urg_laser_readings_t* res, double min_ang, double max_ang, int cluster)
{
  int status;

  // Always set num_readings to 0 so we can return easily in case of erro
  res->num_readings = 0;

  if (!port_open ()) {
    status = -1;
    return status;
  }

  if (SCIP_version == 1)
  {

    status = urg_cmd1("G00076801");

    if (status != 0) {
      urg_flush();
      return status;
    }

    // Finish SCIP1.1 stuff later.

  }
  else if(SCIP_version == 2)
  {
    
    int min_i = afrt + min_ang*ares/(2.0*M_PI);
    int max_i = afrt + max_ang*ares/(2.0*M_PI);
    
    char cmdbuf[MAX_CMD_LEN];

    sprintf(cmdbuf,"GD%.4d%.4d%.2d", min_i, max_i, cluster);

    status = urg_cmd(cmdbuf);

    printf("Sending command; %d %d %d -- %s\n",min_i, max_i, cluster, cmdbuf);
     
    if (status != 0) {
      urg_flush();
      return status;
    }
 
    //Data should never be more than 64 bytes before hitting a \n
    char buf[100];

    // skip over the timestamp
    skip_until("\n", 1, -1);

    // we use ind to cope with data spanning data boundaries
    int ind = 0;
    int i = 0;


    // Populate configuration
    res->config.min_angle  =  (min_i - afrt) * (2.0*M_PI)/(ares);
    res->config.max_angle  =  (max_i - afrt) * (2.0*M_PI)/(ares);
    res->config.resolution =  (2.0*M_PI)/(ares);
    res->config.max_range  =  dmax / 1000.0;

    for (;;)
    {
      int bytes = read_until(&buf[ind], 100 - ind, "\n", 1, -1);

      if (bytes == 1) {
        // This is \n\n so we should be done
        return status;
      }

      bytes += ind - 2;
      

      // Read as many ranges as we can get
      for (int j = 0; j < bytes - (bytes % 3); j+=3)
      {
        if (i < MAX_READINGS)
        {
          res->ranges[i++] = ((buf[j]-0x30) << 12) | ((buf[j+1]-0x30) << 6) | (buf[j+2]-0x30);
          res->num_readings++;
          //          printf("%d ", res->ranges[i]);
        }
        else
        {
          printf("urg_laser::get_readings: Got too many readings.\n");
        }
      }

      // Shuffle remaining bytes to front of buffer to get them on the next loop
      ind = 0;
      for (int j = bytes - (bytes % 3); j < bytes ; j++) {
        buf[ind++] = buf[j];
      }
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
  {

    if (urg_cmd1("V") != 0) {
      return -1;
    }
  
    char buf[100];
    skip_until("SERI:", 5, -1);
    int len = read_until(buf, 100, "\n", 1, -1);
    buf[len-1] = 0;
    id = atoi(buf);

  }
  else // SCIP_version == 2
  {
    if (urg_cmd1("VV") != 0) {
      return -1;
    }
  
    char buf[100];
    skip_until("SERI:", 5, -1);
    int len = read_until(buf, 100, ";", 1, -1);
    buf[len-1] = 0;
    id = atoi(buf);

  }
  return id;
}
