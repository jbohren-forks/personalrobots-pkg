// Software License Agreement (BSD License)
// Copyright (c) 2008, Willow Garage, Inc.
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
//
// author: Rosen Diankov
#include <iostream>

#include "ros/node.h"
#include "rosthread/member_thread.h"

#includle "std_srvs/StaticMap.h"
#includle "std_srvs/PolledImage.h"

using namespace std;
using namespace ros;
using namespace ros::thread;

class OpenraveSession
{
public:
    OpenraveSession() {
    }
    ~OpenraveSession() {
    }

    abstractSessionHandle sessionhandle;
    
    void startros()
    {
        // check if thread launched
        ros::node* pnode = ros::node::instance();

        if( pnode && !pnode->check_master() ) {
            ros::fini();
            delete pnode;
            pnode = NULL;
        }

        if (!pnode) {
            int argc = 0;
            ros::init(argc,NULL);
            char strname[256] = "nohost";
            gethostname(strname, sizeof(strname));
            pnode = new ros::node(strname,);
            member_thread::startMemberFunctionThread<node>(pnode, &ros::node::spin);
        }

        if( !pnode->check_master() )
            return NULL;

        return pnode;
    }

    void startsession()
    {
        ros::node* pnode = startros();
        if( pnode == NULL )
            return;

        vector<string> vnames;
        vnames.push_back("orgetmap");
        vnames.push_back("orgetimage");
        sessionhandle = pnode->advertise_session("mysession"
    }

    bool terminate(int id) {
        cout << "terminate session: " << id << endl;
    }

    bool getmap(StaticMap::request& req, StaticMap::response& res)
    {
        cout << "getmap!" << endl;
        return true;
    }
    bool getimage(PolledImage::request& req, PolledImage::response& res)
    {
        cout << "getimage!" << endl;
        return true;
    }
};
