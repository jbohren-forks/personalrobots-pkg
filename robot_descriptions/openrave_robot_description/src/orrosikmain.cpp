/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2009, Rosen Diankov
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
#include "plugindefs.h"
#include "ikbase.h"

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

namespace pr2left {
#include "ik_pr2left.h"
}
namespace pr2right {
#include "ik_pr2right.h"
}

// need c linkage
extern "C" {

// for some reason windows complains when the prototypes are different
InterfaceBase* DECL_STDCALL(ORCreate, (PluginType type, wchar_t* name, EnvironmentBase* penv))
{
    if( name == NULL ) return NULL;
    
    switch(type) {
        case PT_InverseKinematicsSolver:
            if( wcsicmp(name,L"PR2Leftikfast") == 0 ) {
                vector<int> vfree(pr2left::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = pr2left::getFreeParameters()[i];
                return new IkFastSolver<pr2left::IKReal,pr2left::IKSolution>(pr2left::ik,vfree,0.02f,pr2left::getNumJoints(),penv);
            }
            else if( wcsicmp(name,L"PR2Rightikfast") == 0 ) {
                vector<int> vfree(pr2right::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = pr2right::getFreeParameters()[i];
                return new IkFastSolver<pr2right::IKReal,pr2right::IKSolution>(pr2right::ik,vfree,0.02f,pr2right::getNumJoints(),penv);
            }
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

    pinfo->iksolvers.push_back(L"PR2Leftikfast");
    pinfo->iksolvers.push_back(L"PR2Rightikfast");
    return true;
}

void DECL_STDCALL(DestroyPlugin, ())
{
}

}
