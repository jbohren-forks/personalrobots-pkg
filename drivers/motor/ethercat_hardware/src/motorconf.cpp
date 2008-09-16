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
#include <getopt.h>
#include <sys/mman.h>

#include <ethercat/ethercat_xenomai_drv.h>
#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <al/ethercat_master.h>
#include <al/ethercat_slave_handler.h>

#define WG0X_STANDALONE
#include <ethercat_hardware/wg0x.h>

#include <boost/crc.hpp>

static struct
{
  int model;
  WG0XActuatorInfo config;
} configurations[] = {
  // Elbow, shoulder, spine
  // Maxon 148877
  // http://pr.willowgarage.com/wiki/HardwareComponents/Motors?action=AttachFile&do=get&target=maxonRE40.pdf
  {148877,
    {0, 1,              // Revision
     0, "",             // Id and name
     "PR2",             // Robot name
     "Maxon", "148877", // Motor make and model
     3.12,              // Max current [Amps]
     158,               // Speed constant [rpm/V]
     1.16,              // Resistance [Ohms]
     0.0603,            // Motor torque constant [Nm/A]
     1200, -1,          // Encoder pulses and sign
     "", 0              // Pad and CRC32
    }
  },
 
  // Wrist
  // Maxon 310007
  // http://pr.willowgarage.com/wiki/HardwareComponents/Motors?action=AttachFile&do=get&target=maxonRE30.pdf
  {310007,
    {0, 1,              // Revision
     0, "",             // Id and name
     "PR2",             // Robot name
     "Maxon", "310007", // Motor make and model
     3.44,              // Max current [Amps]
     369,               // Speed constant [rpm/V]
     0.611,             // Resistance [Ohms]
     0.0259,            // Motor torque constant [Nm/A]
     1200, -1,          // Encoder pulses and sign
     "", 0              // Pad and CRC32
    }
  },

  // Head
  // Maxon 310009
  // http://pr.willowgarage.com/wiki/HardwareComponents/Motors?action=AttachFile&do=get&target=maxonRE30.pdf
  {310009,
    {0, 1,              // Revision
     0, "",             // Id and name
     "PR2",             // Robot name
     "Maxon", "310009", // Motor make and model
     1.72,              // Max current [Amps]
     178,               // Speed constant [rpm/V]
     2.52,              // Resistance [Ohms]
     0.0538,            // Motor torque constant [Nm/A]
     1200, -1,          // Encoder pulses and sign
     "", 0              // Pad and CRC32
    }
  },

  // Caster
  // Maxon 236672
  // http://pr.willowgarage.com/wiki/HardwareComponents/Motors?action=AttachFile&do=get&target=maxonAMAX32.pdf
  {236672,
    {0, 1,              // Revision
     0, "",             // Id and name
     "PR2",             // Robot name
     "Maxon", "236672", // Motor make and model
     0.655,             // Max current [Amps]
     136,               // Speed constant [rpm/V]
     16.7,              // Resistance [Ohms]
     0.0704,            // Motor torque constant [Nm/A]
     1200, -1,          // Encoder pulses and sign
     "", 0              // Pad and CRC32
    }
  },

  // Gripper
  // Maxon 222057
  // http://pr.willowgarage.com/wiki/HardwareComponents/Motors?action=AttachFile&do=get&target=maxonREMAX24.pdf
  {222057,
    {0, 1,              // Revision
     0, "",             // Id and name
     "PR2",             // Robot name
     "Maxon", "222057", // Motor make and model
     0.204,             // Max current [Amps]
     156,               // Speed constant [rpm/V]
     56.2,              // Resistance [Ohms]
     0.0613,            // Motor torque constant [Nm/A]
     1200, -1,          // Encoder pulses and sign
     "", 0              // Pad and CRC32
    }
  },

 };


vector<WG0X *> devices;
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
    else if (sh->get_product_code() == WG06::PRODUCT_CODE)
    {
      printf("found a WG06 at #%d\n", slave);
      WG06 *dev = new WG06();
      dev->configure(startAddress, sh);
      devices.push_back(dev);
    }
  }

  vector<WG0X *>::const_iterator device;
  for (device = devices.begin(); device != devices.end(); ++device)
  {
    Actuator a;
    if (!(*device)->sh_->to_state(EC_OP_STATE))
    {
      fprintf(stderr, "Unable set device %d into OP_STATE", (*device)->sh_->get_ring_position());
    }
    (*device)->initialize(&a, true);
  }

  for (device = devices.begin(); device != devices.end(); ++device)
  {
    printf("isProgrammed = %d\n", (*device)->isProgrammed());
  }
}

void programDevice(int device, WG0XActuatorInfo *configuration, char *name)
{
  WG0XActuatorInfo config;

  config = *configuration;
  printf("Programming device %d, to be named: %s\n", device, name);
  strcpy(config.name_, name);
  boost::crc_32_type crc32;
  crc32.process_bytes(&config, sizeof(config)-sizeof(config.crc32_));
  config.crc32_ = crc32.checksum();
  devices[device]->program(&config);
}

int lookupMotor(int motor)
{
  for (int i = 0; i < int(sizeof(configurations)/sizeof(configurations[0])); ++i)
  {
    if (configurations[i].model == motor)
      return i;
  }
  return -1;
}
static struct
{
  char *program_name_;
  char *interface_;
  char *name_;
  bool program_;
  int device_;
  int motor_;
} g_options;

void Usage(string msg = "")
{
  fprintf(stderr, "Usage: %s [options]\n", g_options.program_name_);
  fprintf(stderr, " -i, --interface <i>    Use the network interface <i>\n");
  fprintf(stderr, " -d, --device <d>       Select the device to program\n");
  fprintf(stderr, " -p, --program          Program a motor control board\n");
  fprintf(stderr, " -n, --name <n>         Set the name of the motor control board to <n>\n");
  fprintf(stderr, " -m, --motor <m>        Set the configuration for motor <m>\n");
  fprintf(stderr, "     Legal motor values are:\n");
  for (uint32_t i = 0; i < sizeof(configurations)/sizeof(configurations[0]); ++i)
  {
    fprintf(stderr, "       %d - %s %s\n", configurations[i].model, configurations[i].config.motor_make_, configurations[i].config.motor_model_);
  }
  fprintf(stderr, " -h, --help    Print this message and exit\n");
  if (msg != "")
  {
    fprintf(stderr, "Error: %s\n", msg.c_str());
    exit(-1);
  }
  else
  {
    exit(0);
  }
}

int main(int argc, char *argv[])
{
  // Keep the kernel from swapping us out
  mlockall(MCL_CURRENT | MCL_FUTURE);

  // Parse options
  g_options.program_name_ = argv[0];
  g_options.device_ = -1;
  while (1)
  {
    static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"interface", required_argument, 0, 'i'},
      {"name", required_argument, 0, 'n'},
      {"device", required_argument, 0, 'd'},
      {"motor", required_argument, 0, 'm'},
      {"program", no_argument, 0, 'p'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "d:hi:m:n:p", long_options, &option_index);
    if (c == -1) break;
    switch (c)
    {
      case 'h':
        Usage();
        break;
      case 'd':
        g_options.device_ = atoi(optarg);
        break;
      case 'i':
        g_options.interface_ = optarg;
        break;
      case 'n':
        g_options.name_ = optarg;
        break;
      case 'm':
        g_options.motor_ = atoi(optarg);
        break;
      case 'p':
        g_options.program_ = 1;
        break;
    }
  }

  if (optind < argc)
  {
    Usage("Extra arguments");
  }

  if (!g_options.interface_)
    Usage("You must specify a network interface");

  init(g_options.interface_);

  if (g_options.program_)
  {
    if (!g_options.name_)
      Usage("You must specify a name");
    if (g_options.device_ == -1)
      Usage("You must specify a device #");
    int c = lookupMotor(g_options.motor_);
    if (c == -1)
      Usage("You must specify a valid motor");

    programDevice(g_options.device_, &configurations[c].config, g_options.name_);
  }

  return 0;
}
