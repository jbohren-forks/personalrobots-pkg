#ifndef __SOUND_PLAY__H__
#define __SOUND_PLAY__H__

#include <string>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sound_play/SoundRequest.h>

namespace sound_play
{
class SoundHandle
{
public:
  SoundHandle()
  {
    pub_ = ros::NodeHandle().advertise<sound_play::SoundRequest>("/robotsound", 1);
  }

  void say(const std::string s)
  {
    sendmsg(SoundRequest::SAY, SoundRequest::PLAY_ONCE, s);
  }

  void repeat(const std::string s)
  {
    sendmsg(SoundRequest::SAY, SoundRequest::PLAY_START, s);
  }

  void stopsaying(const std::string s)
  {
    sendmsg(SoundRequest::SAY, SoundRequest::PLAY_STOP, s);
  }

  void playwave(const std::string s)
  {
    sendmsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_ONCE, s);
  }

  void startwave(const std::string s)
  {
    sendmsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_START, s);
  }

  void stopwave(const std::string s)
  {
    sendmsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_STOP, s);
  }

  void play(int sound)
  {
    sendmsg(sound, SoundRequest::PLAY_ONCE);
  }

  void start(int sound)
  {
    sendmsg(sound, SoundRequest::PLAY_START);
  }

  void stop(int sound)
  {
    sendmsg(sound, SoundRequest::PLAY_STOP);
  }

private:
  void sendmsg(int snd, int cmd, std::string s = "")
  {
    SoundRequest msg;
    msg.sound = snd;
    msg.command = cmd;
    msg.arg = s;
    pub_.publish(msg);
  }
  
  ros::Publisher pub_;
};
};

#endif
