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
#include "plugindefs.h"
#include <signal.h>

void sigint_handler(int);
void* MainOpenRAVEThread(void*p);

static EnvironmentBase* penv = NULL;
static int s_WindowWidth = 1024, s_WindowHeight = 768;
static int s_WindowPosX, s_WindowPosY;
static bool s_bSetWindowPosition = false;
static bool bDisplayGUI = false;
static pthread_t s_mainThread;
static boost::shared_ptr<RaveServerBase> s_server;
static bool bThreadDestroyed = false;
int g_argc;
char** g_argv;

int main(int argc, char ** argv)
{
    g_argc = argc;
    g_argv = argv;

    int nServPort = 4765;
    int nDebugLevel = 0;

    // Set up the output streams to support wide characters
    if (fwide(stdout,1) < 0) {
        printf("Unable to set stdout to wide characters\n");
    }

    // create environment and start a command-line controlled simulation 
    penv = CreateEnvironment();
    if( penv == NULL )
        return -1;
    
    // parse command line arguments
    int i = 1;
    while(i < argc) {
        if( stricmp(argv[i], "-loadplugin") == 0 ) {
            penv->LoadPlugin(argv[i+1]);
            i += 2;
        }
        else if( stricmp(argv[i], "-dogui") == 0 ) {
            bDisplayGUI = true;
            i++;
        }
        else if( stricmp(argv[i], "-d") == 0 ) {
            nDebugLevel = atoi(argv[i+1]);
            i += 2;
        }
        else if( stricmp(argv[i], "-wdims") == 0 ) {
            s_WindowWidth = atoi(argv[i+1]);
            s_WindowHeight = atoi(argv[i+2]);
            i += 3;
        }
        else if( stricmp(argv[i], "-wpos") == 0 ) {
            s_WindowPosX = atoi(argv[i+1]);
            s_WindowPosY = atoi(argv[i+2]);
            s_bSetWindowPosition = true;
            i += 3;
        }
        else if( stricmp(argv[i], "-server") == 0 ) {
            nServPort = atoi(argv[i+1]);
            i += 2;
        }
        else {
            RAVEPRINT(L"Error in input parameters at %s\ntype --help to see a list of command line options\n", argv[i]);
            return 0;
        }
    }

    // add a signal handler
    signal(SIGINT,sigint_handler); // control C

    penv->SetDebugLevel(nDebugLevel);

    if( nServPort > 0 ) {
#ifdef _WIN32
        WORD      wVersionRequested;
        WSADATA   wsaData;

        wVersionRequested = MAKEWORD(1,1);
        if (WSAStartup(wVersionRequested, &wsaData) != 0) {
            RAVEPRINT(L"Failed to start win socket\n");
            return -1;
        }
#endif

        //penv->AttachServer(s_server.get());
    }

    bThreadDestroyed = false;
    if( pthread_create(&s_mainThread, NULL, MainOpenRAVEThread, NULL) ) {
        RAVEPRINT(L"failed to create main openrave thread\n");
    }
 
    while(!bThreadDestroyed) {
#ifdef _WIN32
        Sleep(100);
#else
        usleep(100000);
        //pthread_yield();
#endif
    }

    if( penv != NULL ) {
        RaveViewerBase* pviewer = penv->GetViewer();
        penv->GetViewer()->quitmainloop();
        penv->AttachViewer(NULL);
        
        RAVEPRINT(L"deleting the environment\n");
        delete penv; penv = NULL;
    }

    return 0;
}

// use to control openrave
void* MainOpenRAVEThread(void*p)
{
    penv->GetViewer()->main();

    if( !bThreadDestroyed ) {
        penv->GetViewer()->quitmainloop();
        penv->AttachViewer(NULL);
        //s_viewer.reset();
        
        if( penv != NULL ) {        
            delete penv; penv = NULL;
        }
        
        bThreadDestroyed = true;
    }
    
    return NULL;
}

void sigint_handler(int)
{

    if( !bThreadDestroyed ) {
#ifndef _WIN32
        pthread_kill(s_mainThread, SIGINT);
#else
        pthread_kill(s_mainThread, 0);
#endif
        bThreadDestroyed = true;
    }

}
