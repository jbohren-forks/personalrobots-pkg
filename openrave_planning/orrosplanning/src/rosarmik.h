// Software License Agreement (BSD License)
// Copyright (c) 2008, Rosen Diankov
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
#ifndef  OPENRAVE_ROSARMIK_H
#define  OPENRAVE_ROSARMIK_H

#include <libKinematics/pr2_ik.h>

class ROSArmIK : public IkSolverBase
{
public:
    ROSArmIK(EnvironmentBase* penv);

    virtual bool Init(RobotBase* probot, const RobotBase::Manipulator* pmanip, int options);

    virtual bool Solve(const Transform &transEE, const dReal* q0, bool bCheckEnvCollision, dReal* qResult);
    virtual bool Solve(const Transform &transEE, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions);

    virtual bool Solve(const Transform &endEffTransform, const dReal* q0, const dReal* pFreeParameters,
                       bool bCheckEnvCollision, dReal* qResult);
    virtual bool Solve(const Transform &endEffTransform, const dReal* pFreeParameters,
                       bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions);

    virtual int GetNumFreeParameters() const { return 1; }
    virtual bool GetFreeParameters(dReal* pFreeParameters) const;
    virtual RobotBase* GetRobot() const { return _probot; }

private:

    NEWMAT::Matrix GetNewMat(const TransformMatrix& tm);

    inline dReal GetPhiInc() { return 0.03f; }
    bool checkjointangles(vector<dReal>& vravesol);

    RobotBase* _probot;
    std::vector<dReal> _qlower, _qupper, _vjointmult;
    Vector voffset;
    dReal fiFreeParam;
    boost::shared_ptr<kinematics::arm7DOF> iksolver;
};

#endif
