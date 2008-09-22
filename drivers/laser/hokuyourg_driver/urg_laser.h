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

/*! \mainpage
 *  \htmlinclude manifest.html
 */

#ifndef URG_LASER_HH
#define URG_LASER_HH

#include <stdexcept>
#include <termios.h>
#include <string>

//! A namespace containing all URG related things
namespace URG {

  //! The maximum number of readings that can be returned from an urg
  const int MAX_READINGS = 1128;

  //! The maximum length a command will ever be
  const int MAX_CMD_LEN = 100;

  //! The maximum number of bytes that should be skipped when looking for a response
  const int MAX_SKIPPED = 1000000;

  //! Macro for throwing an exception with a message
#define URG_EXCEPT(except, msg) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "URG::laser::%s: " msg, __FUNCTION__); \
    throw except(buf); \
  }

  //! Macro for throwing an exception with a message, passing args
#define URG_EXCEPT_ARGS(except, msg, ...) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "URG::laser::%s: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf); \
  }
  
  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
#define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }
  
  //! A standard URG exception
  DEF_EXCEPTION(exception, std::runtime_error);

  //! An exception for use when a timeout is exceeded
  DEF_EXCEPTION(timeout_exception, exception);

  //! An exception for use when data is corrupted
  DEF_EXCEPTION(corrupted_data_exception, exception);


  //! A struct for returning configuration from the urg
  typedef struct laser_config
  {
    //! Start angle for the laser scan [rad].
    float min_angle;
    //! Stop angle for the laser scan [rad].
    float max_angle;
    //! Scan resolution [rad].
    float ang_increment;
    //! Scan resoltuion [s]
    float time_increment;
    //! Time between scans
    float scan_time;
    //! Minimum range [m]
    float min_range;
    //! Maximum range [m]
    float max_range;
    //! Range Resolution [m]
    float range_res;
  } laser_config_t;


  //! A struct for returning laser readings from the urg
  typedef struct laser_scan
  {
    //! Array of ranges
    float ranges[MAX_READINGS];
    //! Array of intensities
    float intensities[MAX_READINGS];
    //! Number of readings
    int num_readings;
    //! Self reported time stamp in nanoseconds
    unsigned long long self_time_stamp;
    //! System time when first range was measured in nanoseconds
    unsigned long long system_time_stamp;
    //! Configuration of scan
    laser_config_t config;
  } laser_scan_t;


  //! A class for interfacing to the various Hokuyo URG laser devices
  /*! 
   * NOTE: Just about all methods of this class may throw an exception
   * of type URG::exception in the event of an error when
   * communicating with the device.  However, many methods which wrap
   * commands that are sent to the URG will return a URG-supplied
   * status code.  This code may indicate some form of failure on the
   * part of the device itself.  For more information, consult the URG
   * manual.
   */
  class laser
  {
  public:
    //! Constructor
    laser();

    //! Destructor
    ~laser();
  
    //! Open the port
    /*! 
     * This must be done before the URG can be used. This call essentially
     * wraps fopen, with some additional calls to tcsetattr.
     * 
     * \param port_name   A character array containing the name of the port
     * \param use_serial  Set to 1 if using serial instead of USB
     * \param baud        Baud rate to use when working over serial
     *
     */
    void open(const char * port_name, bool use_serial = 0, int baud = B115200);

    //! Close the port
    /*!
     * This call essentiall wraps fclose.
     */
    void close();
  
    //! Check whether the port is open
    bool port_open() {  return laser_port != NULL; }

    //! Sends an SCIP2.0 command to the urg
    /*!
     *  This function allows commands to be sent directly to the URG.
     *  Possible commands can be found in the URG documentation.  It *
     *  will only read up until the end of the status code.  Any
     *  additional bytes will be left on the line for parsing.
     *
     *  \param cmd    Command and arguments in a character array
     *  \param timeout Timeout in milliseconds.
     * 
     *  \return Status code returned from urg device.
     */
    int send_cmd(const char* cmd, int timeout = -1);

    //! Sends an SCIP1.1 command to the urg
    /*!
     *  This function allows commands to be sent directly to the URG.
     *  Possible commands can be found in the URG documentation.
     *  THIS IS NOT YET IMPLEMENTED.
     *
     *  \param cmd    Command and arguments in a character array
     *  \param timeout Timeout in milliseconds.
     * 
     *  \return Status code returned from urg device.
     */
    int send_cmd1(const char* cmd, int timeout = -1);
  
    //! Change the baud rate
    /*!
     * Baud rates are specified using defines in termios:
     * 
     * B19200 - B250000
     *
     * \param curr_baud The current baud rate
     * \param new_baud  The desired baud rate
     * \param timeout Timeout in milliseconds.
     *
     * \return          Returns true on success, false on failure
     */
    bool change_baud(int curr_baud, int new_baud, int timeout = -1);

    //! Retrieve a single scan from the urg
    /*!
     * \param scan       A pointer to an urg_laser_scan_t which will be populated
     * \param min_ang    Minimal angle of the scan
     * \param max_ang    Maximum angle of the scan
     * \param clustering Number of adjascent ranges to be clustered into a single measurement.
     * \param timeout    Timeout in milliseconds.
     *
     * \return Status code returned from urg device.
     */
    int poll_scan(laser_scan_t * scan, double min_ang, double max_ang, int clustering = 0, int timeout = -1);

    //! Request a sequence of scans from the urg
    /*!
     *  This function will request a sequence of scans from the URG.  If
     *  indefinite scans are requested, stop_scanning() must be called
     *  to terminate scanning.  Service_scan() must be called repeatedly
     *  to receive scans as they come in.
     *
     * \param intensity  Whether or not intensity data should be provided
     * \param min_ang    Minimal angle of the scan (radians)
     * \param max_ang    Maximum angle of the scan (radians)
     * \param clustering Number of adjascent ranges to be clustered into a single measurement
     * \param skip       Number of scans to skip between returning measurements
     * \param num        Number of scans to return (0 for indefinite)
     * \param timeout    Timeout in milliseconds.
     *
     * \return Status code returned from urg device.
     */
    int request_scans(bool intensity, double min_ang, double max_ang, int clustering = 0, int skip = 0, int num = 0, int timeout = -1);

    //! Retrieve a scan if they have been requested
    /*!
     * \param scan       A pointer to an urg_laser_scan_t which will be populated.
     * \param timeout    Timeout in milliseconds.
     *
     * \return 0 on succes, Status code returned from urg device on failure.  (Normally 99 is used for success)
     */
    int service_scan(laser_scan_t * scan, int timeout = -1);

    //! Turn the laser off
    /*!
     * \return Status code returned from urg device.
     */
    int laser_off();

    //! Turn the laser on
    /*!
     * \return Status code returned from urg device.
     */
    int laser_on();

    //! Stop retrieving scans
    /*!
     * \return Status code returned from urg device.
     */
    int stop_scanning();

    //! Get serial number from the URG
    /*!
     * \return  Serial number of URG.
     */
    std::string get_ID();

    //! Get status message from the URG
    /*!
     * \return  Status message
     */
    std::string get_status();

    //! Get the SCIP version in use by the urg
    int get_SCIP_version() { return SCIP_version; }
    /*!
     * \return The SCIP version in use by the URG.
     */

    //! Compute latency between actual urg readings and system time
    /*!
     *  This function will estimate the latency between when the data
     *  is actually acquired by the urg and a read call returns from
     *  the urg with the data.  This latency is stored in an offset
     *  variable and applied to the measurement of system time which
     *  occurs immediately after a read.  This means that the
     *  system_time_stamp variable on laser_scan struct is corrected
     *  and accurately reflects the exact time of the hokuyo data in
     *  computer system time.  This function takes the same arguments
     *  as a call to request scans, since this latency may be
     *  parameter dependent.  NOTE: This method will take
     *  approximately 10 seconds to return.
     *
     * \param intensity  Whether or not intensity data should be provided
     * \param min_ang    Minimal angle of the scan (radians)
     * \param max_ang    Maximum angle of the scan (radians)
     * \param clustering Number of adjascent ranges to be clustered into a single measurement
     * \param skip       Number of scans to skip between returning measurements
     * \param num        Number of scans to return (0 for indefinite)
     * \param timeout    Timeout in milliseconds.
     *
     * \return Latency in nanoseconds
     */
    long long calc_latency(bool intensity, double min_ang, double max_ang, int clustering = 0, int skip = 0, int num = 0, int timeout = -1);

  private:
    //! Query which version of SCIP the URG is using
    void query_SCIP_version();

    //! Query the sensor configuration of the urg
    void query_sensor_config();

    //! Wrapper around tcflush
    int urg_flush();

    //! Wrapper around fputs
    int urg_write(const char* msg);

    //! Read a full line from the urg using fgets
    int urg_readline(char *buf, int len, int timeout = -1);

    //! Search for a particular sequence and then read the rest of the line
    char* urg_readline_after(char *buf, int len, const char *str, int timeout = -1);

    //! Compute the checksum of a given buffer
    bool check_sum(const char* buf, int buf_len);

    //! Read in a time value
    unsigned long long read_time(int timeout = -1);

    //! Read in a scan
    void read_data(laser_scan_t* scan, bool has_intensity, int timout = -1);

    int SCIP_version;  

    int dmin;
    int dmax;
    int ares;
    int amin;
    int amax;
    int afrt;
    int rate;

    int wrapped;

    unsigned int last_time;

    long long offset;

    FILE* laser_port;
    int laser_fd;
  };

}

#endif
