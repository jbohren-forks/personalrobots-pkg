///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Morgan Quigley, Eric Berger
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

using std::string;
using std::vector;
using std::pair;

#include <ros/node.h>
#include <ros/time.h>
#include <time.h>

class Logfile : public ros::msg
{
public:
  Logfile() : msg() { }
  string strip_topic(string topic){
    string stripped_topic = topic;
    size_t pos;
    while((pos = stripped_topic.find_first_of("\\/#&;")) != string::npos){
      stripped_topic.replace(pos, 1, "_", 1);
    }
    return stripped_topic;
  }
};

class LogfileCapture : public Logfile{
public:  
  LogfileCapture(const string& directory, const string& topic, ros::Time& start){
      this->directory = directory;
      this->topic = topic;
      this->start = start;
      string filename = directory + string("/") + strip_topic(topic);
      logfile = fopen(filename.c_str(), "w");
      fprintf(logfile, "%s\n", topic.c_str());
    }
  ~LogfileCapture(){
    fclose(logfile);
  }
  inline virtual const string __get_datatype() const {return string("*");}
  inline virtual const string __get_md5sum()   const {return string("*");}
  uint32_t serialization_length() {return 0;}
  uint8_t *serialize(uint8_t *write_ptr){throw "Tried to serialize a capture-only logfile"; return NULL; };
  uint8_t *deserialize(uint8_t *read_ptr){
    ros::Duration elapsed = ros::Time::now() - start;
    fwrite(&elapsed.sec, 4, 1, logfile);
    fwrite(&elapsed.nsec, 4, 1, logfile);
    fwrite(&__serialized_length, 4, 1, logfile);
    fwrite(read_ptr, __serialized_length, 1, logfile);
    return read_ptr + __serialized_length;
  }
private:
  FILE *logfile;
  ros::Time start;
  string directory;
  string topic;
};
