#include <string>
#include "ros/common.h"
#include "borg.h"

using namespace borg;
using std::string;

Borg::Borg(uint32_t opts)
{
  string cfg_path = ros::get_package_path("borg");
  cfg_path += "/borg-config";
  FILE *f = fopen(cfg_path.c_str(), "r");
  if (!f)
  {
    printf("Unable to open the borg-config file in the borg package. Please "
           "create a symlink from one of the borg configuration files in "
           "borg/config, like this:\n"
           "  roscd borg\n"
           "  ln -s config/avt_pike.config borg-config\n");
    exit(1);
  }
  for (int line = 1; !feof(f); line++)
  {
    char linebuf[200];
    char key[50], value[50];
    if (!fgets(linebuf, sizeof(linebuf), f))
      continue;
    if (linebuf[0] == '#')
      continue;
    int n = sscanf(linebuf, "%50s %50s\n", key, value);
    if (n == 0)
      continue;
    if (n != 2)
    {
      printf("unable to parse line %d of the borg-config file\n", line);
      exit(1);
    }
    printf("key = [%s] value = [%s]\n", key, value);
  }
  fclose(f);
  if (opts & INIT_CAM)
  {
    printf("init cam\n");
  }
}

Borg::~Borg()
{
}

