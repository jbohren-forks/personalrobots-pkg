#include "wge100_camera/fcamlib.h"
#include "wge100_camera/host_netutil.h"
#include <string.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <unistd.h>
  
int sensorread(IpCamList *camera, uint8_t reg)
{
  uint16_t val;

  if ( fcamReliableSensorRead( camera, reg, &val, NULL ) != 0) {
    fprintf(stderr, "Could not get register.\n");
    return -1;
  }

  return val;
}

int sensorwrite(IpCamList *camera, uint8_t reg, uint16_t val)
{
  if ( fcamReliableSensorWrite( camera, reg, val, NULL ) != 0) {
    fprintf(stderr, "Could not set register.\n");
    return -1;
  }
  return 0;
}

int usage(int argc, char **argv)
{  
  fprintf(stderr, "Usage: %s <camera_url>                                  # Read all imager register\n", argv[0]);
  fprintf(stderr, "       %s <camera_url> <register>                       # Read an imager registers\n", argv[0]);
  fprintf(stderr, "       %s <camera_url> <register> <value>               # Write an imager register\n", argv[0]);
  fprintf(stderr, "       %s <camera_url> stresstest <register>            # Repeatedly reads imager register\n", argv[0]);
  fprintf(stderr, "       %s <camera_url> stresstest <register> <value>    # Repeatedly writes value to imager register\n", argv[0]);
  fprintf(stderr, "Notes:\n");
  fprintf(stderr, "- All <register> and <value> values are in hexadecimal.\n");
  fprintf(stderr, "- This tool will never reconfigure the camera's IP address to allow its use during image streaming.\n");
  fprintf(stderr, "- Register accesses during image streaming may cause interruptions in the streaming.\n");
  return -1;
}

int main(int argc, char** argv)
{
  bool stress = false;
  bool write = false;

  char **nextarg = argv + 1;
  char **endarg = argv;
  while (*endarg)
    endarg++;

  // Get the URL
  if (nextarg == endarg || !strcmp(*nextarg, "--help")) // No arguments or --help
    return usage(argc, argv);
  const char *camera_url = *nextarg++;

  // Find the camera matching the URL
  IpCamList camera;
  const char *errmsg;
  int outval = fcamFindByUrl(camera_url, &camera, SEC_TO_USEC(0.1), &errmsg);
  if (outval)
  {
    fprintf(stderr, "Matching URL %s : %s\n", camera_url, errmsg);
    return -1;
  }

  if (nextarg == endarg) // Read all
  {
    for (int i = 0; i < 256; i++)
    {
      if (i % 16 == 0)
        fprintf(stderr, "%02x:  ", i); 
      
      if (i % 16 == 8)
        fprintf(stderr, "- "); 
        
      int value = sensorread(&camera, i);
      if (value == -1)
        fprintf(stderr, "??");
      else
        fprintf(stderr, "%04x ", value);
      
      if ((i + 1) % 16 == 0)
        fprintf(stderr, "\n"); 
    }
    return 0;
  }

  if (!strcmp(*nextarg, "stresstest")) // This is a stress test
  {
    stress = true;
    if (++nextarg == endarg)
      return usage(argc, argv);
  }
  // There is at least one argument left at this point.
  
  uint16_t val;
  uint8_t reg;
  
  sscanf(*nextarg++, "%hhx", &reg);

  if (nextarg != endarg) // It is a write.
  {
    sscanf(*nextarg++, "%hx", &val);
    write = true;
  }
                     
  if (nextarg != endarg) // Too many arguments.
    return usage(argc, argv);

  if (stress && !write)
  {
    int count = 0;
    while (1)
    {
      int oldval = sensorread(&camera, reg);
      fprintf(stderr, "Count %i reg %02x read: %04x\n", count++, reg, oldval);
    }
  }
  else if (stress)
  {
    int count = 0;
    uint16_t oldval = sensorread(&camera, reg);
    while (1)
    {
      sensorwrite(&camera, reg, val);
      int newval = sensorread(&camera, reg);
      fprintf(stderr, "Count %i Reg %02x was: %04x set: %04x read: %04x\n", count++, reg, oldval, val, newval);
      oldval = newval;
    }
  }
  else
  {
    uint16_t oldval = sensorread(&camera, reg);
    if (write)
    {
      sensorwrite(&camera, reg, val);
      int newval = sensorread(&camera, reg);
      fprintf(stderr, "Reg %02x was: %04x set: %04x read: %04x\n", reg, oldval, val, newval);
    }
    else
      fprintf(stderr, "Reg %02x read: %04x\n", reg, oldval);
  }

  return 0;
}
