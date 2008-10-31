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

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <ipdcmot/ipdcmot.h>
#include <cstdio>
#include <cstdlib>
#include <pthread.h>
#include <errno.h>
#include <math.h>

const double IPDCMOT::ENC_TO_DEG = 0.00453032;
const double IPDCMOT::ENC_TO_RAD = ENC_TO_DEG * M_PI / 180;
const double IPDCMOT::DEG_TO_ENC = 1.0 / ENC_TO_DEG;
const double IPDCMOT::RAD_TO_ENC = 1.0 / ENC_TO_RAD;
const double IPDCMOT::RAD_TO_DEG = 180.0 / M_PI;
const double IPDCMOT::DEG_TO_RAD = M_PI / 180.0;

IPDCMOT::IPDCMOT(string host, double mount_bias_deg, bool home_myself) : 
  max_ang_vel(500), mount_bias_deg(mount_bias_deg), ok(true),
  homing_in_progress(false), awaiting_response(false), awaiting_position(false),
  servo_mode(IDLE), host(host), reg_mode(UNKNOWN), last_pos_deg(0), last_pos_enc(0), sock(0)
{
  if (clock_gettime(CLOCK_REALTIME, &init_time) == -1)
  {
    printf("woah! couldn't get the system time. BYE.\n");
    exit(100);
  }
  
	printf("opening connection to host [%s]...\n", host.c_str());
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
	{
		printf("couldn't create socket. ur done.\n");
		return;
	}
	hostent *hp;
	server.sin_family = AF_INET;
	server.sin_port = htons(7010);
	if (inet_addr(host.c_str()) == INADDR_NONE)
	{
		hp = gethostbyname(host.c_str());
		if (!hp)
		{
			printf("couldn't resolve host name: [%s]\n", host.c_str());
			close(sock);
			return;
		}
		server.sin_addr.s_addr = *((unsigned long *)hp->h_addr);
		printf("ip address for %s is %s\n", host.c_str(), inet_ntoa(server.sin_addr));
	}
	else
		server.sin_addr.s_addr = inet_addr(host.c_str());
/*
  if (pthread_cond_init(&pos_cond, NULL))
  {
    printf("woah! couldn't create the position pthread condition variable\n");
    exit(123);
  }
  if (pthread_mutex_init(&pos_mutex, NULL))
  {
    printf("woah! couldn't create the position mutex\n");
    exit(124);
  }
*/		

  // do something non-insane 
  patrol.stop1 = 1000;
  patrol.stop2 = 3000;
  patrol.speed = 100;  
  patrol.dir = INCREASING;
  
  printf("spinning out receive thread\n");
	pthread_t recv_thread_handle;
	pthread_create(&recv_thread_handle, NULL, s_recv_thread, this);

  printf("spinning out patrol thread\n");
  pthread_t patrol_thread_handle;
  pthread_create(&patrol_thread_handle, NULL, s_patrol_thread, this);

  if (home_myself)
  {
    home(); // no need, because the hardware homes itself at boot
    set_pos_deg_blocking(1, 5);
  }
}

IPDCMOT::~IPDCMOT()
{
  ok = false;
  usleep(200000);
  stop();
  //pthread_cond_destroy(&pos_cond);
}

void IPDCMOT::home()
{
  net_mutex.lock();
  servo_mode = IDLE;
	unsigned char pkt[10];
	pkt[0] = 0x00;
	pkt[1] = 0x22;
	pkt[2] = 0x12;
	pkt[3] = 0x34;
	pkt[4] = 0x00;
	pkt[5] = 0x01;
	pkt[6] = 0x49;
	place_checksum(pkt, 7);
	homing_in_progress = true;
	send_packet(pkt, 9);
	printf("sent home packet\n");
	double start_time = get_runtime(); //clock.runtime();
	while (homing_in_progress && get_runtime() < start_time + 15.0)
	{
		usleep(100000);
		// query the warning register to see if we've arrived at home yet
		uint8_t pkt[10];
		pkt[0] = 0x00;
		pkt[1] = 0x21;
		pkt[2] = 0x12;
		pkt[3] = 0x34;
		pkt[4] = 0x00;
		pkt[5] = 0x01;
		pkt[6] = 0x08; // warning register
		place_checksum(pkt, 7);
		send_packet(pkt, 9);
	}
	if (homing_in_progress)
	{
		printf("never got to home. I should abort...\n");
		// TODO: send the HOMINGSTOP command
	}
	else
	{
		printf("homed OK\n");
	}
  net_mutex.unlock();
}

bool IPDCMOT::get_pos_blocking(double *position, int *position_enc, int max_wait_secs)
{
  //struct timespec wait_ts;
  //int wait_result;

  if ((!position && !position_enc) || max_wait_secs <= 0)
    return false;
  
  net_mutex.lock();

  awaiting_position = true;
  request_pos();
/*
  if (clock_gettime(CLOCK_REALTIME, &wait_ts) == -1)
  {
    printf("woah! couldn't get the system time. BYE.\n");
    exit(100);
  }
  wait_ts.tv_sec += max_wait_secs;
  */
  //pthread_mutex_lock(&pos_mutex);
  //printf("going into wait loop\n");
  for (int req_sent = 0; req_sent < 20 && awaiting_position; req_sent++) // (awaiting_position)
  {
//    wait_result = pthread_cond_timedwait(&pos_cond, &pos_mutex, &wait_ts);
    for (int i = 0; i < 10 && awaiting_position; i++)
      usleep(10000);
    if (awaiting_position)
      request_pos();
  }
  volatile double local_pos = last_pos_deg;
  volatile int local_pos_enc = last_pos_enc;
  //pthread_mutex_unlock(&pos_mutex);
  //if (wait_result == ETIMEDOUT)
  //  return false;

  if (awaiting_position)
  {
    printf("still awaiting position. BAILING\n");
    net_mutex.unlock();
    return false;
  }

  if (position)
    *position = local_pos;
  if (position_enc)
    *position_enc = local_pos_enc;
  net_mutex.unlock();
  return true;
}

bool IPDCMOT::set_pos_deg_blocking(double deg, int tick_tol, int max_wait_secs)
{
  servo_mode = IDLE;
  int encoder_reading, encoder_target = (int)((deg + mount_bias_deg) * DEG_TO_ENC);
  //printf("blocking position call: target = %d\n", encoder_target);
  set_pos_enc(encoder_target);
  int start_time = (int)time(NULL), cur_time;
  while ((cur_time = (int)time(NULL)) < start_time + max_wait_secs)
  {
    //printf("getting position...\n");
    if (!get_pos_blocking(NULL, &encoder_reading))
    {
      printf("error: couldn't get an encoder reading in IPDCMOT::set_pos_deg_blocking()\n");
      return false;
    }
    //printf("encoder = %d\ttarget = %d\n", encoder_reading, encoder_target);
    if (abs(encoder_reading - encoder_target) <= tick_tol)
    {
      //printf("GOT THERE\n");
      break;
    }
  }
  return (cur_time < start_time + max_wait_secs);
}

void IPDCMOT::request_pos()
{
	uint8_t pkt[10];
	pkt[0] = 0x00;
	pkt[1] = 0x21;
	pkt[2] = 0x12;
	pkt[3] = 0x34;
	pkt[4] = 0x00;
	pkt[5] = 0x01;
	pkt[6] = 0x26; // position register
	place_checksum(pkt, 7);
	send_packet(pkt, 9);
}

void IPDCMOT::set_pos_deg_nonblocking(double deg)
{
  servo_mode = IDLE;
	set_pos_enc((int)((deg + mount_bias_deg) * DEG_TO_ENC));
}

int IPDCMOT::calc_checksum(uint8_t *pkt, unsigned len)
{
	unsigned sum = 0;
	bool add_high_byte = true;
	for (unsigned i = 0; i < len; i++)
	{
		sum += (pkt[i] << (add_high_byte ? 8 : 0)) ^ (add_high_byte ? 0xff00 : 0x00ff);
		add_high_byte = !add_high_byte;
	}
	if (!add_high_byte)
		sum += 0xff;
  unsigned checksum = ((sum >> 16) & 0xffff) + (sum & 0xffff);
	checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	return checksum;
}

void IPDCMOT::place_checksum(uint8_t *pkt, unsigned len)
{
	int csum = calc_checksum(pkt, len);
	pkt[len] = (uint8_t)((csum >> 8) & 0xff);
	pkt[len+1] = (uint8_t)(csum & 0xff);
}

void IPDCMOT::set_regulation_mode(reg_mode_t mode)
{
  net_mutex.lock();
  uint8_t pkt[20];
  pkt[0] = 0x00;
  pkt[1] = 0x22;
  pkt[2] = 0x12;
  pkt[3] = 0x34;
  pkt[4] = 0x00;
  pkt[5] = 0x02;
  pkt[6] = 0x20;
  pkt[7] = mode;
  place_checksum(pkt, 8);
  send_packet(pkt, 10);
  double send_time = get_runtime();
  awaiting_response = true;
  while (awaiting_response && get_runtime() < send_time + 1.0)
    usleep(10000);
  if (awaiting_response)
    printf("no acknowledgement of new regulation mode.\n");
  else
  {
    printf("regulation mode set to %d\n", mode);
    reg_mode = mode;
  }
  net_mutex.unlock();
}

void IPDCMOT::set_input(int input)
{
	uint8_t pkt[20];
	pkt[0] = 0x00;
	pkt[1] = 0x22;
	pkt[2] = 0x13;
	pkt[3] = 0x57;
	pkt[4] = 0x00;
	pkt[5] = 0x05;
	pkt[6] = 0x21;
	pkt[7] = ((unsigned)input >> 24) & 0xff;
	pkt[8] = ((unsigned)input >> 16) & 0xff;
	pkt[9] = ((unsigned)input >>  8) & 0xff;
	pkt[10] = ((unsigned)input >> 0) & 0xff;
	place_checksum(pkt, 11);
	send_packet(pkt, 13);
}

void IPDCMOT::set_pos_enc(int pos_request)
{
  servo_mode = IDLE;
	if (pos_request < 100)
		pos_request = 100; // NEVER let it go below zero! otherwise, it may 
                       // power up past the limit and have to spin around...
  if (pos_request > 60000)
    pos_request = 60000; // ditto
	if (reg_mode != POSITION)
		set_regulation_mode(POSITION);
	if (reg_mode == POSITION)
  {
		set_input(pos_request);
    //printf("set position input to %d\n", pos_request);
  }
  else
    printf("woah! regulation mode is not position\n");
}

void IPDCMOT::set_vel(int vel_request)
{
  if (vel_request > 1000)
    vel_request = 1000;
  else if (vel_request < -1000)
    vel_request = -1000;

	if (reg_mode != VELOCITY)
		set_regulation_mode(VELOCITY);
	if (reg_mode == VELOCITY)
		set_input(vel_request);
}

void *IPDCMOT::s_recv_thread(void *parent)
{
  ((IPDCMOT *)parent)->recv_thread();
  return NULL;
}

void IPDCMOT::recv_thread()
{
  printf("in receive thread\n");
	char buf[100];
	while(sock)
	{
		int n = recv(sock, buf, 100, 0);
		//printf("received %d bytes\n", n);
		if (n < 5)
			continue; // this shouldn't happen...
		// this only handles single-register responses:
		if (buf[1] == 0x23)
		{
			switch(buf[6])
			{
				case 0x08:
					if (!(buf[9] & 0x04))
						homing_in_progress = false;
					break;
				case 0x26:
				{
					int pos_enc = 0;
					// ahhhhhhh big endian.
					unsigned pos_enc_bigendian = *((unsigned *)(buf+7));
					pos_enc = htonl(pos_enc_bigendian);
					//printf("little endian = %x = %d\n", pos_enc, pos_enc);
					//printf("position = %d\n", pos_enc);
					//pthread_mutex_lock(&pos_mutex);
					last_pos_enc = pos_enc;
					last_pos_deg = (double)pos_enc * ENC_TO_DEG - mount_bias_deg;
          awaiting_position = false;
          /*
					if (awaiting_position)
					{
            printf("releasing mutex\n");
					  awaiting_position = false;
					  pthread_cond_signal(&pos_cond);
					}
					pthread_mutex_unlock(&pos_mutex);
          */
					//printf("pos = %f\n", pos);
					break;
				}
			}
		}
		else if (buf[1] == 0x24)
			awaiting_response = false;
	}
	printf("leaving receive thread\n");
}

void IPDCMOT::send_packet(uint8_t *pkt, unsigned len)
{
	if (!sendto(sock, pkt, len, 0, (struct sockaddr *)&server, sizeof(server)))
	{
		printf("couldn't send data\n");
		close(sock);
		sock = 0;
	}
}

double IPDCMOT::get_runtime()
{
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  
  long sec_diff = curtime.tv_sec - init_time.tv_sec;
  long nsec_diff = curtime.tv_nsec - init_time.tv_nsec;
  if (nsec_diff < 0) 
  {
    sec_diff--;
    nsec_diff += 1000000000;
  }
  return (double)(sec_diff) + 1.0e-9 * (double)(nsec_diff);
}

void *IPDCMOT::s_patrol_thread(void *parent)
{
  ((IPDCMOT *)parent)->patrol_thread();
  return NULL;
}

void IPDCMOT::patrol_thread()
{
  printf("entering patrol thread\n");
  while (ok)
  {
    usleep(100000);
    if (servo_mode != PATROL)
      continue;
    double pos;
    if (!get_pos_blocking(&pos, NULL, 1))
    {
      // set speed to zero
      printf("couldn't get position for patrol thread. bailing.\n");
      set_vel(0);
      break;
    }
    if (patrol.dir == INCREASING && pos > patrol.stop2)
    {
      patrol.dir = DECREASING;
      set_vel(-(int32_t)(patrol.speed / ENC_TO_DEG));
    }
    else if (patrol.dir == DECREASING && pos < patrol.stop1)
    {
      patrol.dir = INCREASING;
      set_vel((int32_t)(patrol.speed / ENC_TO_DEG));
    }
  }
  set_vel(0);
  printf("exiting patrol thread\n");
}

void IPDCMOT::set_patrol(double stop1, double stop2, 
                         double speed, int32_t init_dir)
{
  if (stop1 < 10) stop1 = 10; // always be well above 0 so homing works OK
  if (stop2 < 10) stop2 = 10; // you don't want to go around an extra time 
                                // and get all the wires tangled
  if (stop1 > 350) stop1 = 350; // don't wrap around for same reason
  if (stop2 > 350) stop2 = 350;
  if (stop1 > stop2)
  {
    double swapme = stop1;
    stop1 = stop2;
    stop2 = swapme;
  }
  if (speed < 0) speed = 0;
  // upper speed limit is clamped in the set_vel() function
  patrol.stop1 = stop1;
  patrol.stop2 = stop2;
  patrol.speed = speed;
  if (init_dir > 0)
  {
    patrol.dir = INCREASING;
    set_vel((int32_t)(patrol.speed * DEG_TO_ENC));
  }
  else
  {
    patrol.dir = DECREASING;
    set_vel(-(int32_t)(patrol.speed * DEG_TO_ENC));
  }
  servo_mode = PATROL;
}

void IPDCMOT::stop()
{
  set_vel(0);
}

