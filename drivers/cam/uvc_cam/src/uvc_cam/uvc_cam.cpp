#include <cstring>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include "uvc_cam/uvc_cam.h"

using std::string;
using namespace uvc_cam;

void Cam::enumerate()
{
  string v4l_path = "/sys/class/video4linux";
  DIR *d = opendir(v4l_path.c_str());
  if (!d)
    throw std::runtime_error("couldn't open " + v4l_path);
  struct dirent *ent, *ent2;
  int fd, ret;
  struct v4l2_capability v4l2_cap;
  while ((ent = readdir(d)) != NULL)
  {
    if (strncmp(ent->d_name, "video", 5))
      continue; // ignore anything not starting with "video"
    string dev_name = string("/dev/") + string(ent->d_name);
    printf("enumerating %s ...\n", dev_name.c_str());
    if ((fd = open(dev_name.c_str(), O_RDWR)) == -1)
      throw std::runtime_error("couldn't open " + dev_name + "  perhaps the " +
                               "permissions are not set correctly?");
    if ((ret = ioctl(fd, VIDIOC_QUERYCAP, &v4l2_cap)) < 0)
      throw std::runtime_error("couldn't query " + dev_name);
    printf("name = [%s]\n", v4l2_cap.card);
    printf("driver = [%s]\n", v4l2_cap.driver);
    printf("location = [%s]\n", v4l2_cap.bus_info);
    close(fd);
    string v4l_dev_path = v4l_path + string("/") + string(ent->d_name) + 
                          string("/device");
    // my kernel is using /sys/class/video4linux/videoN/device/inputX/id
    DIR *d2 = opendir(v4l_dev_path.c_str());
    if (!d2)
      throw std::runtime_error("couldn't open " + v4l_dev_path);
    string input_dir;
    while ((ent2 = readdir(d2)) != NULL)
    {
      if (strncmp(ent2->d_name, "input", 5))
        continue; // ignore anything not beginning with "input"
      input_dir = ent2->d_name;
      break;
    }
    closedir(d2);
    if (!input_dir.length())
      throw std::runtime_error("couldn't find input dir in " + v4l_dev_path);
    string vid_fname = v4l_dev_path + string("/") + input_dir + 
                       string("/id/vendor");
    string pid_fname = v4l_dev_path + string("/") + input_dir + 
                       string("/id/product");
    string ver_fname = v4l_dev_path + string("/") + input_dir + 
                       string("/id/version");
    char vid[5], pid[5], ver[5];
    FILE *vid_fp = fopen(vid_fname.c_str(), "r");
    if (!vid_fp)
      throw std::runtime_error("couldn't open " + vid_fname);
    if (!fgets(vid, sizeof(vid), vid_fp))
      throw std::runtime_error("couldn't read VID from " + vid_fname);
    fclose(vid_fp);
    vid[4] = 0;
    printf("vid = [%s]\n", vid);
    FILE *pid_fp = fopen(pid_fname.c_str(), "r");
    if (!pid_fp)
      throw std::runtime_error("couldn't open " + pid_fname);
    if (!fgets(pid, sizeof(pid), pid_fp))
      throw std::runtime_error("couldn't read PID from " + pid_fname);
    fclose(pid_fp);
    printf("pid = [%s]\n", pid);
    FILE *ver_fp = fopen(ver_fname.c_str(), "r");
    if (!ver_fp)
      throw std::runtime_error("couldn't open " + ver_fname);
    if (!fgets(ver, sizeof(ver), ver_fp))
      throw std::runtime_error("couldn't read version from " + ver_fname);
    fclose(ver_fp);
    printf("ver = [%s]\n", ver);
  }

  closedir(d);
}

