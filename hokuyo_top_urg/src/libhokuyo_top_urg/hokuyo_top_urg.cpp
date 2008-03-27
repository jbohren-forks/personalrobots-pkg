#include <math.h>
#include "stair__serial_port/lightweightserial.h"
#include "stair__hokuyo_top_urg/hokuyo_top_urg.h"

const double HokuyoTopUrg::SCAN_FOV = 270.0 * M_PI / 180.0;

HokuyoTopUrg::HokuyoTopUrg(string serial_str) : 
  serial(NULL)
{
  serial = new LightweightSerial(serial_str.c_str(), 115200);
  happy = serial->is_ok();
}

HokuyoTopUrg::~HokuyoTopUrg()
{
  if (serial)
    delete serial;
  serial = NULL;
}

void HokuyoTopUrg::process_data_line(char *line, int line_num)
{
  int linelen = strlen(line);
  for (int i = 0; i < linelen-1; i++)
  {
    switch(decode_state)
    {
      case DS_BYTE_1:
        incoming_scan[incoming_idx] = ((unsigned short)line[i] - 0x0030) << 12;
        decode_state = DS_BYTE_2;
        break;
      case DS_BYTE_2:
        incoming_scan[incoming_idx] += ((unsigned short)line[i] - 0x0030) << 6;
        decode_state = DS_BYTE_3;
        break;
      case DS_BYTE_3:
        incoming_scan[incoming_idx] += ((unsigned short)line[i] - 0x0030) << 0;
        decode_state = DS_BYTE_1;
        incoming_idx++;
        break;
    }
  }
}

bool HokuyoTopUrg::poll()
{
  if (!happy)
    return false;
  
  unsigned char serbuf[4096];
  serial->write_block("GD0000112700\n", 13); // get a whole scan
  bool scan_complete = false;
  int attempts = 0;
  incoming_idx = 0;
  decode_state = DS_BYTE_1;
  char count = 0;
  const int LINEBUF_LEN = 80;
  char linebuf[LINEBUF_LEN];
  char linepos = 0;
  int line_num = 0;
  enum { L_ECHO, L_STATUS, L_TIMESTAMP, L_DATA } line_type = L_ECHO;
  while (!scan_complete)
  {
    char c;
    count++;
    if (!serial->read(&c))
    {
      attempts++;
      if (++attempts > 1000)
      {
        printf("Hokuyo TOP-URG: no response (%d recv)\n", count-1);
        count = 0;
        happy = false;
        break;
      }
      usleep(1000);
      continue;
    }
    if (linepos >= LINEBUF_LEN-1)
    {
      printf("ahhh! tried to run through linebuf. DON'T THINK SO.\n");
      linepos = 0;
      continue;
    }
    if (c == '\n')
    {
      linebuf[linepos] = 0;
      if (linepos == 0)
      {
        //printf("end of tx range_idx = %d\n", range_idx);
        break;
      }
      switch(line_type)
      {
        case L_ECHO:       line_type = L_STATUS; line_num = 0; break;
        case L_STATUS:     line_type = L_TIMESTAMP; break;
        case L_TIMESTAMP:  line_type = L_DATA; break;
        case L_DATA:       process_data_line(linebuf, line_num++); break;
      }
      linepos = 0;
    }
    else
      linebuf[linepos++] = c;
  }
  for (int i = 0; i < num_ranges; i++)
    latest_scan[i] = incoming_scan[i] * 0.001;
  return true;
}

