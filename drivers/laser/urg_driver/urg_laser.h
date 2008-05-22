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
#include <termios.h>

//! The maximum number of readings that can be returned from a urg
#define MAX_READINGS 1128

//! The maximum length a command will ever be
#define MAX_CMD_LEN 100

//! The maximum number of bytes that should be skipped when looking for a response
#define MAX_SKIPPED 1000000  


//! A struct for returning configuration from the urg
typedef struct urg_laser_config
{
  //! Start angle for the laser scan [rad].
  float min_angle;
  //! Stop angle for the laser scan [rad].
  float max_angle;
  //! Scan resolution [rad].
  float resolution;
  //! Minimum range [m]
  float min_range;
  //! Maximum range [m]
  float max_range;
  //! Range Resolution [m]
  float range_res;
  //! Scanning frequency [Hz]
  float scanning_frequency;
} urg_laser_config_t;


//! A struct for returning laser readings from the urg
typedef struct urg_laser_scan
{
  //! Array of ranges
  float ranges[MAX_READINGS];
  //! Array of intensities
  float intensities[MAX_READINGS];
  //! Number of readings
  int num_readings;
  //! Self reported time
  int time_stamp;
  //! Configuration of scan
  urg_laser_config_t config;
} urg_laser_scan_t;


//! A class for interfacing to the various Hokuyo URG devices
/*!
 *  Except where specified otherwise, methods of this class return 0
 *  or positive values on success and negative values on failure.
 */
class urg_laser
{
public:
  //! Constructor
  urg_laser();

  //! Destructor
  ~urg_laser();
  
  //! Open the port
  /*! 
   * This must be done before the URG can be used. This call essentially
   * wraps fopen, with some additional calls to tcsetattr.
   * 
   * \param port_name   A character array containing the name of the port
   * \param use_serial  Set to 1 if using serial instead of USB
   * \param baud        Baud rate to use when working over serial
   *
   * \return            Returns 0 on success, -1 on failure
   */
  int open(const char * port_name, bool use_serial = 0, int baud = B115200);

  //! Close the port
  /*!
   * This call essentiall wraps fclose.
   *
   * \return            Returns 0 on success, -1 on failure
   */
  int close();
  
  //! Check whether the port is open
  bool port_open() {  return laser_port != NULL; }

  //! Sends an SCIP2.0 command to the urg
  /*!
   *  This function allows commands to be sent directly to the URG.
   *  Possible commands can be found in the URG documentation.
   *
   *  \param Command and arguments in a character array
   * 
   *  \return Status code returned from urg device.
   */
  int urg_cmd(const char* cmd, int timeout = -1);

  //! Sends an SCIP1.1 command to the urg
  /*!
   *  This function allows commands to be sent directly to the URG.
   *  Possible commands can be found in the URG documentation.
   *
   *  \param Command and arguments in a character array
   * 
   *  \return Status code returned from urg device.
   */
  int urg_cmd1(const char* cmd, int timeout = -1);
  
  //! Change the baud rate
  /*!
   * Baud rates are specified using defines in termios:
   * 
   * B19200 - B250000
   *
   * \param curr_baud The current baud rate
   * \param new_baud  The desired baud rate
   * \param timeout   A timeout in milliseconds
   *
   * \return          Returns 0 on success, -1 on failure
   */
  int change_baud(int curr_baud, int new_baud, int timeout = -1);

  //! Get a single scan from the urg
  int poll_scan(urg_laser_scan_t * scan, double min_ang, double max_ang, int clustering = 0, int timeout = -1);

  int request_scans(bool intensity, double min_ang, double max_ang, int cluster = 0, int skip = 0, int num = 0, int timeout = -1);

  int service_scan(urg_laser_scan_t * scan, int timeout = -1);

  int stop_scanning();

  //! Get serial number from the URG
  int get_ID();

  //! Get the SCIP version in use by the urg
  int get_SCIP_version() { return SCIP_version; }
  
private:
  //! Query which version of SCIP the URG is using
  int query_SCIP_version();

  //! Query the sensor configuration of the urg
  int query_sensor_config();

  //! Wrapper around tcflush
  int urg_flush();

  //! Wrapper around write
  int urg_write(const char* msg);

  int urg_readline(char *buf, int len, int timeout = -1);

  char* urg_readline_after(char *buf, int len, const char *str, int timeout = -1);

  int check_sum(const char* buf, int buf_len);

  int read_data(urg_laser_scan_t* scan, bool has_intensity, int timout = -1);

  int SCIP_version;  

  int dmin;
  int dmax;
  int ares;
  int amin;
  int amax;
  int afrt;
  int scan;

  FILE * laser_port;
  int laser_fd;
};
