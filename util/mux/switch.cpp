///////////////////////////////////////////////////////////////////////////////
// The mux package provides a generic multiplexer
//
// Copyright (C) 2008, Morgan Quigley
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
/////////////////////////////////////////////////////////////////////////////

#include "ros/node.h"
#include "std_srvs/StringString.h"
using namespace std;
using namespace ros;

class MuxSwitcher : public ros::Node
{
public:
  MuxSwitcher() : Node("MuxSwitcher") { }
  void switchMux(string s)
  {
    std_srvs::StringString::request  req;
    std_srvs::StringString::response res;
    req.str = s;
    bool ok = service::call("mux", req, res);
    if (ok)
      printf("success. previous setting was [%s]\n", res.str.c_str());
    else
      printf("failed.\n");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc < 2)
  {
    printf("usage: switch SELECTED_TOPIC\n");
    return 1;
  }
  MuxSwitcher ms;
  ms.switchMux(argv[1]);
  ros::fini();
}

