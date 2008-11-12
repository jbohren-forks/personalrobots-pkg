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


#ifndef MS_3DMGX2_HH
#define MS_3DMGX2_HH

#include <fstream>
#include <stdexcept>

namespace MS_3DMGX2
{

  const double G               = 9.80665; // m/sec^2
  const int TICKS_PER_SEC      = 19660800;
  const int MAX_BYTES_SKIPPED  = 1000;
  const int KF_NUM_SUM         = 100;
  const double KF_K_1          = 0.00995031;
  const double KF_K_2          = 0.0000497506;


  #define IMU_EXCEPT(except, msg) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "MS_3DMGX2::IMU::%s: " msg, __FUNCTION__); \
    throw except(buf); \
  }

  //! Macro for throwing an exception with a message, passing args
  #define IMU_EXCEPT_ARGS(except, msg, ...) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "MS_3DMGX2::IMU::%s: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf); \
  }
  
  //! Macro for defining exception (std::runtime_error should be top parent)
  #define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }

  DEF_EXCEPTION(exception, std::runtime_error);
  DEF_EXCEPTION(timeout_exception, exception);
  DEF_EXCEPTION(corrupted_data_exception, exception);

  //! A class for interfacing to the microstrain 3dmgx2 and inertialink IMUs
  class IMU
  {

  public: 

    enum cmd {
      CMD_RAW                      =  0xC1,
      CMD_ACCEL_ANGRATE            =  0xC2,
      CMD_DELVEL_DELANG            =  0xC3,
      CMD_CONTINUOUS               =  0xC4,
      CMD_ORIENT                   =  0xC5,
      CMD_ATT_UPDATE               =  0xC6,
      CMD_MAG_VEC                  =  0xC7,
      CMD_ACCEL_ANGRATE_ORIENT     =  0xC8,
      CMD_WRITE_ACCEL_BIAS         =  0xC9,
      CMD_WRITE_GYRO_BIAS          =  0xCA,
      CMD_ACCEL_ANGRATE_MAG        =  0xCB,
      CMD_ACCEL_ANGRATE_MAG_ORIENT =  0xCC,
      CMD_CAPTURE_GYRO_BIAS        =  0xCD,
      CMD_EULER                    =  0xCE,
      CMD_EULER_ANGRATE            =  0xCF,
      CMD_TEMPERATURES             =  0xD1,
      CMD_GYROSTAB_ANGRATE_MAG     =  0xD2,
      CMD_DELVEL_DELANG_MAG        =  0xD3,
      CMD_STOP_CONTINUOUS          =  0xFA
    };


    // Constructor
    IMU();

    // Destructor
    ~IMU();

    // Open the serial port
    // Returns 0 on success
    void open_port(const char *port_name);

    // Close the serial port
    // Returns 0 on success
    void close_port();


    void init_time();

    void init_gyros(double* bias_x = 0, double* bias_y = 0, double* bias_z = 0);

    // Put into continuous mode:
    bool set_continuous(cmd command);

    // Take out of continuous mode:
    void stop_continuous();

    // Read the ...
    void receive_accel_angrate(uint64_t *time, double accel[3], double angrate[3]);

    // Read the stabilized acceleration vectors
    void receive_accel_angrate_mag(uint64_t *time, double accel[3], double angrate[3], double mag[3]);

    void receive_euler(uint64_t *time, double *roll, double *pitch, double *yaw);

    void receive_accel_angrate_orientation(uint64_t *time, double accel[3], double angrate[3], double orientation[9]);


    // Send a packet and wait for a reply from the IMU.
    // Returns the number of bytes read.
    int transact(void *cmd, int cmd_len, void *rep, int rep_len, int timeout = 0);

    // Send a packet to the IMU
    int send(void *cmd, int cmd_len);

    // Receive a packet from the IMU
    int receive(uint8_t command, void *rep, int rep_len, int timeout = 0, uint64_t* sys_time = NULL);

    uint64_t extract_time(uint8_t* addr);

    // Kalman filter for time
    uint64_t filter_time(uint64_t imu_time, uint64_t sys_time);

    // convert uint64_t time to double time
    double to_double(uint64_t time);

    // convert double time to uint64_t time
    uint64_t to_uint64_t(double time);

    // Port file descriptor
    int fd;

    uint32_t wraps;

    uint32_t offset_ticks;

    uint32_t last_ticks;

    uint32_t diff_ticks;

    unsigned long long start_time;

    double time_est[2];
    double P_time_est[2][2];

    bool continuous;

    unsigned int counter;
    double offset, d_offset, sum_meas;
  };

}
#endif
