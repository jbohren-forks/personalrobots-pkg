///////////////////////////////////////////////////////////////////////////////
// The ipdcmot package provides a library that talks to the FMOD IP-based 
// motor controller. I just have their single-channel 1.5A box, but perhaps
// some of this code will be useful on their other boxes as well.
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#ifndef IPDCMOT_IPDCMOT_H
#define IPDCMOT_IPDCMOT_H

#include <string>
#include <arpa/inet.h>
#include <time.h>
#include <pthread.h>
#include "boost/thread/mutex.hpp"
using namespace std;

class IPDCMOT
{
public:
  IPDCMOT(string host, double mount_bias_deg = 0, bool home_myself = true);
  ~IPDCMOT();

  void home();
  void stop();
  bool get_pos_blocking(double *position, int *position_enc, int max_wait_secs = 20);
  bool set_pos_deg_blocking(double deg, int tick_tol = 1, int max_wait_secs = 20);
  void set_pos_deg_nonblocking(double deg);
  void set_patrol(double stop1, double stop2, double speed, int32_t init_dir);
  inline int get_patrol_dir() { return (patrol.dir == INCREASING ? 1 : -1); }
  
private:
  boost::mutex net_mutex;
  double max_ang_vel, mount_bias_deg;
  bool ok, homing_in_progress, awaiting_response, awaiting_position;
  enum patrol_dir_t { INCREASING = 0, DECREASING };
  struct
  {
    double stop1, stop2, speed;
    patrol_dir_t dir;
  } patrol;
  enum
  {
    IDLE = 0,
    PATROL,
  } servo_mode;
  string host;
  enum reg_mode_t
  {
    UNKNOWN  = 0x00,
    VELOCITY = 0x04,
    POSITION = 0x05
  } reg_mode;
  double last_pos_deg;
  int last_pos_enc;
  static int calc_checksum(uint8_t *pkt, unsigned len);
  static void place_checksum(uint8_t *pkt, unsigned len);
  void set_regulation_mode(reg_mode_t mode);
  void set_input(int input);

  void set_pos_enc(int pan_request);
  void set_vel(int vel_request);
  
  void request_pos();
  //pthread_cond_t pos_cond;
  //pthread_mutex_t pos_mutex;

  const static double DEG_TO_ENC, RAD_TO_ENC;
  const static double ENC_TO_DEG, ENC_TO_RAD;
  const static double RAD_TO_DEG, DEG_TO_RAD;

  int sock;
  sockaddr_in server;
  
  static void *s_recv_thread(void *parent);
  void recv_thread();
  void send_packet(uint8_t *pkt, unsigned len);

  static void *s_patrol_thread(void *parent);
  void patrol_thread();
  
  struct timespec init_time;
  double get_runtime();
};

#endif

