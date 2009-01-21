#include <cstring>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include "uvc_cam/uvc_cam.h"

using std::string;
using namespace uvc_cam;

Cam::Cam(const char *_device) : device(_device)
{
  if ((fd = open(_device, O_RDWR)) == -1)
    throw std::runtime_error("couldn't open " + device);
  memset(&fmt, 0, sizeof(v4l2_format));
  memset(&cap, 0, sizeof(v4l2_capability));
  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
    throw std::runtime_error("couldn't query " + device);
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error(device + " does not support capture");
  if (!(cap.capabilities & V4L2_CAP_STREAMING))
    throw std::runtime_error(device + " does not support streaming");
  // enumerate formats
  v4l2_fmtdesc f;
  memset(&f, 0, sizeof(f));
  f.index = 0;
  f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int ret;
  while ((ret = ioctl(fd, VIDIOC_ENUM_FMT, &f)) == 0)
  {
    printf("pixfmt %d = '%4s' desc = '%s'\n",
           f.index++, (char *)&f.pixelformat, f.description);
    // enumerate frame sizes
    v4l2_frmsizeenum fsize;
    fsize.index = 0;
    fsize.pixel_format = f.pixelformat;
    while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0)
    {
      fsize.index++;
      if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
      {
        printf("  discrete: %ux%u:   ",
               fsize.discrete.width, fsize.discrete.height);
        // enumerate frame rates
        v4l2_frmivalenum fival;
        fival.index = 0;
        fival.pixel_format = f.pixelformat;
        fival.width = fsize.discrete.width;
        fival.height = fsize.discrete.height;
        while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0)
        {
          fival.index++;
          if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
          {
            printf("%u/%u ",
                   fival.discrete.numerator, fival.discrete.denominator);
          }
          else
            printf("I only handle discrete frame intervals...\n");
        }
        printf("\n");
      }
      else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS)
      {
        printf("  continuous: %ux%u to %ux%u\n",
               fsize.stepwise.min_width, fsize.stepwise.min_height,
               fsize.stepwise.max_width, fsize.stepwise.max_height);
      }
      else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
      {
        printf("  stepwise: %ux%u to %ux%u step %ux%u\n",
               fsize.stepwise.min_width,  fsize.stepwise.min_height,
               fsize.stepwise.max_width,  fsize.stepwise.max_height,
               fsize.stepwise.step_width, fsize.stepwise.step_height);
      }
      else
      {
        printf("  fsize.type not supported: %d\n", fsize.type);
      }
    }
  }
  if (errno != EINVAL)
    throw std::runtime_error("error enumerating frame formats");
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  static const unsigned WIDTH = 640, HEIGHT = 480, FPS = 15;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = 'Y' | ('U' << 8) | ('Y' << 16) | ('V' << 24);
  fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if ((ret = ioctl(fd, VIDIOC_S_FMT, &fmt)) < 0)
    throw std::runtime_error("couldn't set format");
  if (fmt.fmt.pix.width != WIDTH || fmt.fmt.pix.height != HEIGHT)
    throw std::runtime_error("pixel format unavailable");
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator = FPS;
  if ((ret = ioctl(fd, VIDIOC_S_PARM, &streamparm)) < 0)
    throw std::runtime_error("unable to set framerate");
  memset(&rb, 0, sizeof(rb));
  rb.count = NUM_BUFFER;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &rb) < 0)
    throw std::runtime_error("unable to allocate buffers");
  for (unsigned i = 0; i < NUM_BUFFER; i++)
  {
    memset(&buf, 0, sizeof(buf));
    buf.index = i;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.flags = V4L2_BUF_FLAG_TIMECODE;
    buf.timecode = timecode;
    buf.timestamp.tv_sec = 0;
    buf.timestamp.tv_usec = 0;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
      throw std::runtime_error("unable to query buffer");
    if (buf.length <= 0)
      throw std::runtime_error("buffer length is bogus");
    mem[i] = mmap(0, buf.length, PROT_READ, MAP_SHARED, fd, buf.m.offset);
    //printf("buf length = %d at %x\n", buf.length, mem[i]);
    if (mem[i] == MAP_FAILED)
      throw std::runtime_error("couldn't map buffer");
  }
  unsigned buf_len = buf.length;
  for (unsigned i = 0; i < NUM_BUFFER; i++)
  {
    memset(&buf, 0, sizeof(buf));
    buf.index = i;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.flags = V4L2_BUF_FLAG_TIMECODE;
    buf.timecode = timecode;
    buf.timestamp.tv_sec = 0;
    buf.timestamp.tv_usec = 0;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
      throw std::runtime_error("unable to queue buffer");
  }
  buf.length = buf_len;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    throw std::runtime_error("unable to start capture");

}

Cam::~Cam()
{
  // stop stream
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE, ret;
  if ((ret = ioctl(fd, VIDIOC_STREAMOFF, &type)) < 0)
    perror("VIDIOC_STREAMOFF");
  for (unsigned i = 0; i < NUM_BUFFER; i++)
    if (munmap(mem[i], buf_length) < 0)
      perror("failed to unmap buffer");
  close(fd);
}

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

int Cam::grab(unsigned char **frame)
{
  *frame = NULL;
  int ret = 0;
  fd_set rdset;
  timeval timeout;
  FD_ZERO(&rdset);
  FD_SET(fd, &rdset);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  ret = select(fd + 1, &rdset, NULL, NULL, &timeout);
  if (ret <= 0)
  {
    perror("couldn't grab image");
    return -1;
  }
  if (!FD_ISSET(fd, &rdset))
    return -1;
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
    throw std::runtime_error("couldn't dequeue buffer");
  // assume yuyv and bayer for now
  *frame = (unsigned char *)mem[buf.index];
  return buf.index;
  //tmp = new unsigned char[640 * 480 * 3];
  //bayer_to_rgb24(mem[buf.index], tmp, 640, 480, 
}

void Cam::release(int buf_idx)
{
  if (buf_idx >= 0 && buf_idx < NUM_BUFFER)
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
      throw std::runtime_error("couldn't requeue buffer");
}

