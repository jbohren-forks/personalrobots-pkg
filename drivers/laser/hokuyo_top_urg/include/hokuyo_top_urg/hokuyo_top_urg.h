#ifndef STAIR_HOKUYO_TOP_URG_TOP_URG_H
#define STAIR_HOKUYO_TOP_URG_TOP_URG_H

#include <string>
using namespace std;

class LightweightSerial;

class HokuyoTopUrg
{
public:
  HokuyoTopUrg(string serial_str);
  ~HokuyoTopUrg();
  bool poll();
  inline bool is_ok() { return happy; }
  static const int num_ranges = 1128;
  double latest_scan[num_ranges];
  static const double SCAN_FOV;

private:
  bool happy;
  uint16_t incoming_scan[num_ranges], incoming_idx;
  void process_data_line(char *line, int line_num);
  void process_response(char *d, unsigned n);
  LightweightSerial *serial;
  string serial_str;
  enum decode_state_t
  {
    DS_BYTE_1,
    DS_BYTE_2,
    DS_BYTE_3,
  } decode_state;
  uint32_t range_idx, cur_range;
};

#endif

