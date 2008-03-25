#ifndef STAIR_AXIS213_AXIS213_H
#define STAIR_AXIS213_AXIS213_H

#include <string>
#include <curl/curl.h>

using namespace std;

class Axis213
{
public:
  Axis213(string ip);
  ~Axis213();

  bool get_jpeg(uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size);
  bool get_ptz(float* pan, float* tilt, float* zoom, float* focus = 0);
  bool ptz(float pan, float tilt, float zoom, float focus = -1, bool relative = false);
  static size_t write_ptz(void* buffer, size_t size, size_t nmemb, void* userp);
  static size_t write_jpeg(void* buffer, size_t size, size_t nmemb, void* userp);
private:
  string ip;
  uint8_t *jpeg_buf;
  char *ptz_buf;

  uint32_t jpeg_buf_size;
  uint32_t ptz_buf_size;

  uint32_t jpeg_file_size;
  uint32_t ptz_file_size;

  int req_count;

  CURL* ptz_handle_control;
  CURL* ptz_handle_observe;
  CURL* image_handle;

  char ptz_control_url[256];
  char ptz_observe_url[256];
  char image_url[256];

  inline float clamp(float d, float low, float high)
  {
    return (d < low ? low : (d > high ? high : d));
  }
};

#endif

