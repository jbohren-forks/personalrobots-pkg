// Software License Agreement (BSD License)
// Copyright (c) 2008, Willow Garage, Inc.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
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
#include "openraveros.h"
#include "session.h"
#include <signal.h>

void sigint_handler(int);

boost::shared_ptr<ros::Node> s_pmasternode;
boost::shared_ptr<SessionServer> s_sessionserver;

void printhelp()
{
    wprintf(L"openraveros [--list] [--debuglevel [level]]\n"
            "  Starts the OpenRAVE ROS server\n"
            "--list             List all the loadable interfaces (ie, collision checkers).\n"
            "-d, --debuglevel [level]    Set a debug level, higher numbers are more verbose (default is 3)\n");
}

void printinterfaces(EnvironmentBase* penv)
{
    PLUGININFO info;
    penv->GetLoadedInterfaces(info);

    vector<wstring>::const_iterator itnames;     
    vector<string> names;
    vector<string>::iterator itname;
    wstringstream ss;
            
    ss << endl << L"Loadable interfaces: " << endl;

    ss << L"Collision Checkers (" << info.collisioncheckers.size() << "):" << endl;
    for(itnames = info.collisioncheckers.begin(); itnames != info.collisioncheckers.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    ss << L"Controllers (" << info.controllers.size() << "):" << endl;
    for(itnames = info.controllers.begin(); itnames != info.controllers.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;
    
    ss << L"Inverse Kinematics Solvers (" << info.iksolvers.size() << "):" << endl;
    for(itnames = info.iksolvers.begin(); itnames != info.iksolvers.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    ss << L"Physics Engines (" << info.physicsengines.size() << "):" << endl;
    for(itnames = info.physicsengines.begin(); itnames != info.physicsengines.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    ss << L"Planners (" << info.planners.size() << "):" << endl;
    for(itnames = info.planners.begin(); itnames != info.planners.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    ss << L"Problems (" << info.problems.size() << "):" << endl;
    for(itnames = info.problems.begin(); itnames != info.problems.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    ss << L"Robots (" << info.robots.size() << "):" << endl;
    for(itnames = info.robots.begin(); itnames != info.robots.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    ss << L"Sensors (" << info.sensors.size() << "):" << endl;
    for(itnames = info.sensors.begin(); itnames != info.sensors.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    ss << L"Sensor Systems (" << info.sensorsystems.size() << "):" << endl;
    for(itnames = info.sensorsystems.begin(); itnames != info.sensorsystems.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    ss << L"Trajectories (" << info.trajectories.size() << "):" << endl;
    for(itnames = info.trajectories.begin(); itnames != info.trajectories.end(); ++itnames)
        ss << " " << itnames->c_str() << endl;

    wprintf(ss.str().c_str());
}

int main(int argc, char ** argv)
{
    // Set up the output streams to support wide characters
    if (fwide(stdout,1) < 0) {
        printf("Unable to set stdout to wide characters\n");
    }

    signal(SIGINT,sigint_handler); // control C

    ros::init(argc,argv);
    s_pmasternode.reset(new ros::Node("openraveserver", ros::Node::DONT_HANDLE_SIGINT));

    if( !s_pmasternode->checkMaster() )
        return -1;
    
    s_sessionserver.reset(new SessionServer());
    if( !s_sessionserver->Init() )
        return -1;

    // parse the command line options
    bool bExit = false;
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-?") == 0 || strcmp(argv[i], "/?") == 0 || strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
            printhelp();
            bExit = true;
            break;
        }
        else if( strcmp(argv[i], "--debuglevel") == 0 || strcmp(argv[i], "-d") == 0 ) {
            RaveSetDebugLevel((DebugLevel)atoi(argv[i+1]));
            i += 2;
        }
        else if( strcmp(argv[i], "--list") == 0 ) {
            printinterfaces(s_sessionserver->GetParentEnvironment().get());
            bExit = true;
            break;
        }
        else
            break;
    }

    if( !bExit )
        s_sessionserver->spin();
    s_sessionserver.reset();
    ros::fini();
    s_pmasternode.reset();
    return 0;
}

void sigint_handler(int)
{
    s_sessionserver->selfDestruct();
    s_pmasternode->selfDestruct();
}
