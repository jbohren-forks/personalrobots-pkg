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

#include <stdio.h>
#include <sys/mman.h>

#include <ethercat/ethercat_xenomai_drv.h>
#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <al/ethercat_master.h>
#include <al/ethercat_slave_handler.h>

#include <ethercat_hardware/wg05.h>

#include <boost/crc.hpp>

static struct
{
  int model;
  WG05ActuatorInfo config;
} configurations[] = {
  {148877, {0, 1, 0, "motor1", "PR2", "Maxon", "148877", 3.12, 1.0 / 158, 1.16, 0.0603, 1200, -1, "", 0}},
  {148877, {0, 1, 0, "motor2", "PR2", "Maxon", "148877", 3.12, 1.0 / 158, 1.16, 0.0603, 1200, -1, "", 0}},
  {310007, {0, 1, 0, "", "PR2", "Maxon", "310007", 3.44, 1.0 / 369, 0.611, 0.0199, 1200, -1, "", 0}}
};


void init(char *interface)
{
  struct netif *ni;

  // Initialize network interface
  if ((ni = init_ec(interface)) == NULL)
  {
    fprintf(stderr, "Unable to initialize interface: %s", interface);
    exit(-1);
  }

  // Initialize Application Layer (AL)
  EtherCAT_DataLinkLayer::instance()->attach(ni);
  EtherCAT_AL *al;
  if ((al = EtherCAT_AL::instance()) == NULL)
  {
    fprintf(stderr, "Unable to initialize Application Layer (AL): %08x", (int)al);
    exit(-1);
  }

  uint32_t num_slaves = al->get_num_slaves();
  if (num_slaves == 0)
  {
    fprintf(stderr, "Unable to locate any slaves");
    exit(-1);
  }

  // Initialize Master
  EtherCAT_Master *em;
  if ((em = EtherCAT_Master::instance()) == NULL)
  {
    fprintf(stderr, "Unable to initialize EtherCAT_Master: %08x", int(em));
    exit(-1);
  }

  vector<WG05 *> devices;
  static int startAddress = 0x00010000;

  for (unsigned int slave = 0; slave < num_slaves; ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = em->get_slave_handler(fsa);
    if (sh == NULL)
    {
      fprintf(stderr, "Unable to get slave handler #%d", slave);
      exit(-1);
    }

    if (sh->get_product_code() == WG05::PRODUCT_CODE)
    {
      printf("found a WG05 at #%d\n", slave);
      WG05 *dev = new WG05();
      dev->configure(startAddress, sh);
      devices.push_back(dev);
    }
  }

  vector<WG05 *>::const_iterator device;
  for (device = devices.begin(); device != devices.end(); ++device)
  {
    Actuator a;
    if (!(*device)->sh_->to_state(EC_OP_STATE))
    {
      fprintf(stderr, "Unable set device %d into OP_STATE", (*device)->sh_->get_ring_position());
    }
    (*device)->initialize(&a, true);
  }

  for (int i = 0; i < sizeof(configurations)/sizeof(configurations[0]); ++i)
  {
    boost::crc_32_type crc32;
    crc32.process_bytes(&configurations[i].config, sizeof(configurations[i].config)-sizeof(configurations[i].config.crc32_));
    configurations[i].config.crc32_ = crc32.checksum();
  }

  for (device = devices.begin(); device != devices.end(); ++device)
  {
    printf("isProgrammed = %d\n", (*device)->isProgrammed());
  }
#if 0
  devices[0]->program(&configurations[0].config);
  devices[1]->program(&configurations[1].config);
#endif
}

int main(int argc, char *argv[])
{
  // Keep the kernel from swapping us out
  mlockall(MCL_CURRENT | MCL_FUTURE);

  if (argc != 2)
  {
    fprintf(stderr, "Usage: %s <interface>\n", argv[0]);
    exit(-1);
  }

  init(argv[1]);

  return 0;
}
