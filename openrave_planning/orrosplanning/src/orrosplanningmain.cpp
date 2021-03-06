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
// \author Rosen Diankov
#include "plugindefs.h"

#include "mocapsystem.h"
#include "objecttransformsystem.h"
#include "collisionmapsystem.h"
//#include "rosrobotcontroller.h"
#include "rosplanningproblem.h"

// declaring variables with stdcall can be a little complex
#ifdef _MSC_VER

#define PROT_STDCALL(name, paramlist) __stdcall name paramlist
#define DECL_STDCALL(name, paramlist) __stdcall name paramlist

#else

#ifdef __x86_64__
#define DECL_STDCALL(name, paramlist) name paramlist
#else
#define DECL_STDCALL(name, paramlist) __attribute__((stdcall)) name paramlist
#endif

#endif // _MSC_VER

// need c linkage
extern "C" {

InterfaceBase* DECL_STDCALL(ORCreate, (PluginType type, wchar_t* name, EnvironmentBase* penv))
{
    switch(type) {
//    case PT_Controller:
//        if( wcsicmp(name, L"ROSRobot") == 0 )
//            return new ROSRobotController(penv);
//        break;
    case PT_ProblemInstance:
        if( wcsicmp(name, L"ROSPlanning") == 0 )
            return new ROSPlanningProblem(penv);
        break;
    case PT_SensorSystem:
        if( wcsicmp(name, L"ROSMocap") == 0 )
            return new ROSMocapSystem(penv);
        else if( wcsicmp(name, L"ObjectTransform") == 0 )
            return new ObjectTransformSystem(penv);
        else if( wcsicmp(name, L"CollisionMap") == 0 )
            return new CollisionMapSystem(penv);
        break;
    default:
        break;
    }

    return NULL;
}

bool DECL_STDCALL(GetPluginAttributes, (PLUGININFO* pinfo, int size))
{
    if( pinfo == NULL ) return false;
    if( size != sizeof(PLUGININFO) ) {
        RAVELOG_ERRORA("bad plugin info sizes %d != %d\n", size, sizeof(PLUGININFO));
        return false;
    }

    //pinfo->controllers.push_back(L"ROSRobot");
    pinfo->problems.push_back(L"ROSPlanning");
    pinfo->sensorsystems.push_back(L"ROSMocap");
    pinfo->sensorsystems.push_back(L"ObjectTransform");
    pinfo->sensorsystems.push_back(L"CollisionMap");

    return true;
}

void DECL_STDCALL(DestroyPlugin, ())
{
}

}
