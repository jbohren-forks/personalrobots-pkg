#include <map>
#include <list>
#include <string>
#include <stdexcept>
#include <string.h>
#include "ros/common.h"
#include "ros/time.h"
#include "borg.h"
#include "cam_dc1394.h"

using namespace borg;
using std::string;
using std::map;
using std::list;

Borg::Borg(uint32_t opts) : cam(NULL), stage(NULL),
  left(50), right(50), scan_duty(700), return_duty(600)
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
           "  ln -s config/mobileborg.config borg-config\n");
    exit(1);
  }
  string cam_str;
  map<string, uint32_t> cam_settings;
  map<string, string> stage_settings;
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
      throw std::runtime_error("borg init failed");
    }
    if (!strcmp(key, "fps"))
      fps = atoi(value);
    else if (!strcmp(key, "left"))
      left = atof(value);
    else if (!strcmp(key, "right"))
      right = atof(value);
    else if (!strcmp(key, "scan_duty"))
      scan_duty = atoi(value);
    else if (!strcmp(key, "return_duty"))
      return_duty = atoi(value);
    else if (!strcmp(key, "cam"))
      cam_str = string(value);
    else if (!strncmp(key, "cam_", 4))
      cam_settings[string(key+4)] = atoi(value);
    else if (!strncmp(key, "stage_", 6))
      stage_settings[string(key+6)] = string(value);
    else if (!strncmp(key, "calib_", 6))
      calib_set(key+6, atof(value));
    else
      printf("unknown key = [%s] with value = [%s]\n", key, value);
  }
  fclose(f);
  if (opts & INIT_CAM)
  {
    if (cam_str == string("dc1394"))
    {
      printf("calling dc1394 init\n");
      cam = new CamDC1394();
      if (!cam->init())
        throw std::runtime_error("unable to init camera.\n");
      for (map<string, uint32_t>::iterator s = cam_settings.begin();
           s != cam_settings.end(); ++s)
        cam->set(s->first.c_str(), s->second);
    }
    else
    {
      printf("unknown camera\n");
      throw std::runtime_error("borg init failed");
    }
  }
  if (opts & INIT_STAGE)
  {
    stage = new Stage();
    for (map<string,string>::iterator s = stage_settings.begin();
        s != stage_settings.end(); ++s)
      stage->set(s->first.c_str(), s->second);
  }
  intrinsics = cvCreateMat(3, 3, CV_32FC1);
  distortion = cvCreateMat(1, 4, CV_32FC1);
  cvSetZero(intrinsics);
  CV_MAT_ELEM(*intrinsics, float, 0, 0) = fx;
  CV_MAT_ELEM(*intrinsics, float, 1, 1) = fy;
  CV_MAT_ELEM(*intrinsics, float, 0, 2) = x0;
  CV_MAT_ELEM(*intrinsics, float, 1, 2) = y0;
  CV_MAT_ELEM(*intrinsics, float, 2, 2) = 1.0f;
  CV_MAT_ELEM(*distortion, float, 0, 0) = k1;
  CV_MAT_ELEM(*distortion, float, 0, 1) = k2;
  CV_MAT_ELEM(*distortion, float, 0, 2) = k3;
  CV_MAT_ELEM(*distortion, float, 0, 3) = k4;

}

Borg::~Borg()
{
  if (cam)
  {
    cam->shutdown();
    delete cam;
  }
  cvReleaseMat(&intrinsics);
  cvReleaseMat(&distortion);
}

struct ScanImage
{
  uint8_t *raster;
  double t, pos;
};

bool Borg::scan()
{
  stage->setDuty(return_duty);
  stage->gotoPosition(left, true);
  stage->setDuty(scan_duty);
  stage->gotoPosition(right, false);
  stage->laser(true);
  cam->startImageStream();
  ros::Time t_start(ros::Time::now());
  list<ScanImage *> images;
  double pos = 0;
  printf("right = %f\n", right);
  for (ros::Time t_now(ros::Time::now()); 
       (t_now - t_start).to_double() < 15 && fabs(pos - right) > 0.5;
       t_now = ros::Time::now())
  {
    pos = stage->getPosition(1.0);
    //printf("%.3f %.3f\n", t_now.to_double(), pos);
    ScanImage *si = new ScanImage;
    si->raster = new uint8_t[640*480];
    si->t = ros::Time::now().to_double();
    si->pos = pos;
    if (!cam->savePhoto(si->raster))
    {
      printf("woah! couldn't grab photo\n");
      cam->stopImageStream();
      return false;
    }
    images.push_back(si);
  }
  stage->laser(false);
  double dt = (ros::Time::now() - t_start).to_double();
  printf("captured %d images in %.3f seconds (%.3f fps)\n", 
         images.size(), dt, images.size() / dt);
  cam->stopImageStream();
  // flush to disk
  for (list<ScanImage *>::iterator i = images.begin(); i != images.end(); ++i)
  {
    char fname[100];
    snprintf(fname, sizeof(fname), "out/img_%.6f_%.6f.pgm", (*i)->t, (*i)->pos);
    FILE *f = fopen(fname, "wb");
    if (!f)
      throw std::runtime_error("couldn't open pgm file for output");
    fprintf(f, "P5\n640 480\n255\n");
    if (640*480 != fwrite((*i)->raster, 1, 640 * 480, f))
    {
      printf("couldn't write pgm\n");
      break;
    }
    fclose(f);
    delete[] (*i)->raster;
    delete *i;
  }
  images.clear();
  return true;
}

bool Borg::calib_set(const char *setting, double value)
{
  if (!strcmp(setting, "fx"))
    fx = value;
  return true;
}

