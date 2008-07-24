/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <mechanism_control/base_control.h>
#include <ethercat_hardware/ethercat_hardware.h>

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>

static int quit = 0;
static const int NSEC_PER_SEC = 1e+9;

void *controlLoop(void *arg)
{
  char *interface = (char *)arg;

  EthercatHardware ec;
  ec.init(interface, (char *)"foo.xml");

  MechanismControl mc;
  mc.init(ec.hw);

  // Switch to hard real-time
  int period = 1000000;
  struct timespec tick;
#if defined(__XENO__)
  pthread_set_mode_np(0, PTHREAD_PRIMARY|PTHREAD_WARNSW);
#endif
  clock_gettime(CLOCK_REALTIME, &tick);

  while (!quit)
  {
    ec.update();
    mc.update();
    tick.tv_nsec += period;
    while (tick.tv_nsec >= NSEC_PER_SEC)
    {
      tick.tv_nsec -= NSEC_PER_SEC;
      tick.tv_sec++;
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
  }

  memset(ec.hw->actuator, 0, ec.hw->numActuators * sizeof(Actuator));
  ec.update();

  return 0;
}

void quitRequested(int sig)
{
  quit = 1;
}

void warnOnSecondary(int sig)
{
  void *bt[32];
  int nentries;

  // Dump a backtrace of the frame which caused the switch to
  // secondary mode
  nentries = backtrace(bt, sizeof(bt) / sizeof(bt[0]));
  backtrace_symbols_fd(bt, nentries, fileno(stdout));
}

static pthread_t rtThread;
static pthread_attr_t rtThreadAttr;

int main(int argc, char *argv[])
{
  // Initialize ROS and check command-line arguments
  ros::init(argc, argv);
  if (argc != 2)
  {
    fprintf(stderr, "Usage: %s <interface>\n", argv[0]);
    exit(-1);
  }

  ros::node *node = new ros::node("mechanism_control");

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Catch if we fall back to secondary mode
  signal(SIGXCPU, warnOnSecondary);

  // Keep the kernel from swapping us out
  mlockall(MCL_CURRENT | MCL_FUTURE);

  // Set up thread scheduler for realtime
  pthread_attr_init(&rtThreadAttr);
  pthread_attr_setinheritsched(&rtThreadAttr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&rtThreadAttr, SCHED_FIFO);

  //Start thread
  int rv;
  if ((rv = pthread_create(&rtThread, &rtThreadAttr, controlLoop, argv[1])) != 0)
  {
    perror("pthread_create");
    exit(1);
  }

  pthread_join(rtThread, 0);

  ros::fini();
  delete node;

  return 0;
}
