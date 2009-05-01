/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef __LEDWIZ_H__
#define __LEDWIZ_H__

#include <unistd.h>
#include <fcntl.h>

namespace ledwiz
{
  const unsigned char LEDWIZ_MAX_INTENSITY = 48;
  const unsigned char LEDWIZ_SAWTOOTH = 129;
  const unsigned char LEDWIZ_BLINK = 130;
  const unsigned char LEDWIZ_DOWN_RAMP_BLINK = 131;
  const unsigned char LEDWIZ_UP_RAMP_BLINK = 132;

  class ledwiz
  {
  public: 
    ledwiz()
    {
      init();
    }

    ledwiz(std::string devname)
    {
      init();
      open(devname);
    }

    int open(std::string devname)
    {
      if (fd != -1)
        close(); // @todo this call might fail.
      
      fd = ::open(devname.c_str(), O_WRONLY);
      return -(fd == -1);
    }

    ~ledwiz()
    {
      close();
    }

    int close()
    {
      if (fd != -1)
      { 
        int retval = ::close(fd);
        if (!retval)
          fd = -1;
        return retval;
      }
      return 0;
    }
    
    int write()
    {
      return -(write_state_rate() || write_intensity());
    }

    int write_state_rate()
    {
      /* Checked when intensity is set.
      if (rate > 7) 
      { 
        errno = EINVAL; 
        return -1; 
      } 
      */

      bzero(buff, sizeof(buff)); 
      buff[1] = 0x40; 
      buff[2] = outstate & 0xFF; 
      buff[3] = (outstate >> 8) & 0xFF; 
      buff[4] = (outstate >> 16) & 0xFF; 
      buff[5] = (outstate >> 24) & 0xFF; 
      buff[6] = rate; 


      if (::write(fd, buff, sizeof(buff)) < 0) 
        return -1; 

      return 0; 
    }

    int write_intensity()
    {
      int i, j, k;

      /* Checked when intensity is set.
      for (i = 0; i < 32; i++)
        if ((s->intensity[i] > 48 && s->intensity[i] < 129) || s->intensity[i] > 132)
        {
          errno = EINVAL;
          return -1;
        }
        */

      bzero(buff, sizeof(buff));
      for (i = 0, j = 0; i < 4; i++)
      {
        for (k = 1; k <= 8; k++, j++)
        {
          buff[k] = intensity[j];
        }

        if (::write(fd, buff, sizeof(buff)) < 0)
          return -1;
      }

      return 0;
    }

    int set_rate(int newrate)
    {
      //printf("set_rate %i\n", newrate);
      
      if (newrate > 7)
      {
        errno = EINVAL;
        return -1;
      }
      rate = newrate;
      return 0;
    }

    int set_intensity(int newintensity, int channel)
    {         
      //printf("set_intensity %i %i\n", newintensity, channel);

      if (newintensity < 0 || (newintensity > 48 && newintensity < 129) || newintensity > 132 || 
          channel < 0 || channel > 32)
      {
        errno = EINVAL;
        return -1;
      } 
      intensity[channel] = newintensity;
      return 0;
    }

    int set_outstate(int newoutstate)
    {
      outstate = newoutstate;

      return 0;
    }
    
    int get_rate()
    {
      return rate;
    }

    int get_intensity(int newintensity, int channel)
    {
      if (channel < 0 || channel > 32)
        return -1;
      return intensity[channel];
    }

    int get_outstate()
    {
      return outstate;
    }

private:
    void init()
    {
      fd = -1;
      outstate = 0xFFFFFFFF;
      rate = 0;
      bzero(intensity, sizeof(intensity));
    }

    int fd;
    unsigned int outstate;
    unsigned char intensity[32];
    unsigned char rate;
    char buff[9];
  } ledwiz_state;
}


#endif
