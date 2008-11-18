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

#include "rosarmik.h"

#ifndef SQR
template <class T>
inline T SQR(T x) { return x * x; }
#endif

ROSArmIK::ROSArmIK() : IkSolverBase()
{
}

bool ROSArmIK::Init(RobotBase* probot, const RobotBase::Manipulator* pmanip, int options)
{
    assert( probot != NULL );
    _probot = probot;
    if( _probot == NULL || pmanip == NULL )
        return false;

    // get the joint limits
    const vector<int>& vjoints = pmanip->_vecarmjoints;

    vector<dReal> qlower, qupper;
    _probot->GetJointLimits(qlower, qupper);
    _qlower.resize(vjoints.size()); _qupper.resize(vjoints.size());

    for(int i = 0; i < (int)vjoints.size(); ++i) {
        _qlower[i] = qlower[vjoints[i]];
        _qupper[i] = qupper[vjoints[i]];
    }

    if( _qlower.size() > 0 )
        fiFreeParam = 1.0f / (_qupper[0]-_qlower[0]);
    else fiFreeParam = 1;

    std::vector<NEWMAT::Matrix> axis;
    std::vector<NEWMAT::Matrix> anchor;
    std::vector<std::string> joint_type;

    NEWMAT::Matrix aj(3,1);
    NEWMAT::Matrix an(3,1);

    joint_type.resize(7);

    // Shoulder pan
    aj << 0 << 0 << -1.0;
    axis.push_back(aj);
    an << 0 << 0 << 0;
    anchor.push_back(an);

    // Shoulder pitch
    aj << 0 << -1 << 0;
    axis.push_back(aj);
    an << 0.1 << 0 << 0;
    anchor.push_back(an);

    // Shoulder roll
    aj << -1 << 0 << 0;
    axis.push_back(aj);
    an << 0.1 << 0 << 0;
    anchor.push_back(an);

    // Elbow flex
    aj << 0 << 1 << 0;
    axis.push_back(aj);
    an << 0.5 << 0 << 0;
    anchor.push_back(an);

    // Forearm roll
    aj << 1 << 0 << 0;
    axis.push_back(aj);
    an << 0.5 << 0 << 0;
    anchor.push_back(an);

    // Wrist flex
    aj << 0 << 1 << 0;
    axis.push_back(aj);
    an << 0.82025 << 0 << 0;
    anchor.push_back(an);

    // Gripper roll
    aj << -1 << 0 << 0;
    axis.push_back(aj);
    an << 0.82025 << 0 << 0;
    anchor.push_back(an);

    for(int i=0; i < 7; i++)
        joint_type[i] = std::string("ROTARY");

    iksolver.reset(new kinematics::arm7DOF(anchor,axis,joint_type));
    
    if( pmanip->_vecarmjoints.size() > 0 ) {
        RobotBase::RobotStateSaver saver(_probot);
        _probot->SetTransform(Transform());
        vector<dReal> vjoints(_probot->GetDOF(),0);
        _probot->SetJointValues(NULL,NULL,&vjoints[0]);
        Transform tbase = pmanip->pBase->GetTransform();
        KinBody::Joint* pjoint = _probot->GetJoint(pmanip->_vecarmjoints.front());
        KinBody::Link* pother = pjoint->GetFirstAttached() == pmanip->pBase ? pjoint->GetSecondAttached() : pjoint->GetFirstAttached();
        if( pother != NULL )
            voffset = tbase.trans - pother->GetTransform().trans;
    }

    return true;
}

// end eff transform is the transform of the wrist with respect to the base arm link
bool ROSArmIK::Solve(const Transform &_T, const dReal* q0, bool bCheckEnvCollision, dReal* qResult)
{
    assert( _probot != NULL );
    
    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();
    assert( pmanip != NULL );
    
    // the world coordinate system is at the origin of the intersection of the first 3 joint axes
    assert( pmanip->_vecarmjoints.size() == _qlower.size() );

    RobotBase::RobotStateSaver saver(_probot);

    _probot->SetActiveDOFs(pmanip->_vecarmjoints);

    // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
    dReal startphi = q0 != NULL ? q0[0] : 0;
    dReal upperphi = _qupper[0], lowerphi = _qlower[0], deltaphi = 0;
    int iter = 0;
    bool bsuccess = false;

    dReal bestdist = 1000; // only valid if q0 != NULL
    NEWMAT::Matrix nmT = GetNewMat(_T);

    while(1) {

        dReal curphi = startphi;
        if( iter & 1 ) { // increment
            curphi += deltaphi;
            if( curphi > upperphi ) {

                if( startphi-deltaphi < lowerphi)
                    break; // reached limit
                ++iter;
                continue;
            }
        }
        else { // decrement
            curphi -= deltaphi;
            if( curphi < lowerphi ) {

                if( startphi+deltaphi > upperphi )
                    break; // reached limit
                deltaphi += GetPhiInc(); // increment
                ++iter;
                continue;
            }

            deltaphi += GetPhiInc(); // increment
        }

        iter++;
        
        iksolver->ComputeIK(nmT,curphi);
        vector<dReal> vravesol(_probot->GetActiveDOF());
        vector<double>* pbest = NULL;
        FOREACH(itsol, iksolver->solution_ik_) {
            vector<double>& sol = *itsol;
            assert( (int)sol.size() == _probot->GetActiveDOF());
            
            // find the first valid solution that satisfies joint constraints and collisions
            wstringstream ss;
            int j;
            for(j = 0; j < (int)pmanip->_vecarmjoints.size(); ++j) {
                if( sol[j] < _qlower[j] || sol[j] > _qupper[j] )
                    break;
            }

            if( j < (int)pmanip->_vecarmjoints.size() )
                continue; // out of bounds

            // check for self collisions
            for(int i = 0; i < (int)sol.size(); ++i)
                vravesol[i] = sol[i];
            _probot->SetActiveDOFValues(NULL, &vravesol[0]);

            if( _probot->CheckSelfCollision() )
                continue;

            COLLISIONREPORT report;
            if( bCheckEnvCollision && g_pEnviron->CheckCollision(_probot, &report) ) {
                if( report.plink1 != NULL && report.plink2 != NULL ) {
                    RAVELOG(L"WAMIK: collision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
                }
                continue;
            }

            // solution is valid, check with q0
            if( q0 != NULL ) {

                dReal d = 0;
                for(int k = 0; k < (int)pmanip->_vecarmjoints.size(); ++k)
                    d += SQR(sol[k]-q0[k]);

                if( bestdist > d ) {
                    pbest = &sol;
                    bestdist = d;
                }
            }
            else {
                pbest = &sol;
                break;
            }
        }

        // return as soon as a solution is found, since we're visiting phis starting from q0[0], we are guaranteed
        // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
        if( pbest != NULL ) {

            if( qResult != NULL ) {
                for(int i = 0; i < (int)pbest->size(); ++i)
                    qResult[i] = (*pbest)[i];
            }
            bsuccess = true;
            break;
        }
    }
    
    return bsuccess;
}

bool ROSArmIK::Solve(const Transform &_T, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
{
    assert( _probot != NULL );
    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();

    assert( pmanip != NULL );
    
    qSolutions.resize(0);

    // the world coordinate system is at the origin of the intersection of the first 3 joint axes
    assert( pmanip->_vecarmjoints.size() && _qlower.size() );

    RobotBase::RobotStateSaver saver(_probot);

    _probot->SetActiveDOFs(pmanip->_vecarmjoints);

    // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
    double startphi = 0;
    double upperphi = _qupper[0], lowerphi = _qlower[0], deltaphi = 0;
    int iter = 0;

    NEWMAT::Matrix nmT = GetNewMat(_T);

    while(1) {
        double curphi = startphi;
        if( iter & 1 ) { // increment
            curphi += deltaphi;
            if( curphi > upperphi ) {

                if( startphi-deltaphi < lowerphi)
                    break; // reached limit
                ++iter;
                continue;
            }
        }
        else { // decrement
            curphi -= deltaphi;
            if( curphi < lowerphi ) {

                if( startphi+deltaphi > upperphi )
                    break; // reached limit
                ++iter;
                deltaphi += GetPhiInc(); // increment
                continue;
            }

            deltaphi += GetPhiInc(); // increment
        }

        iter++;

        iksolver->ComputeIK(nmT,curphi);
        vector<dReal> vravesol(_probot->GetActiveDOF());

        FOREACH(itsol, iksolver->solution_ik_) {
            vector<double>& sol = *itsol;
            assert( (int)sol.size() == _probot->GetActiveDOF());

            // find the first valid solutino that satisfies joint constraints and collisions
            int j;
            for(j = 0; j < (int)pmanip->_vecarmjoints.size(); ++j) {
                if( sol[j] < _qlower[j] || sol[j] > _qupper[j] )
                    break;
            }

            if( j < (int)pmanip->_vecarmjoints.size() )
                continue; // out of bounds

            // check for self collisions
            for(int i = 0; i < (int)sol.size(); ++i)
                vravesol[i] = sol[i];
            _probot->SetActiveDOFValues(NULL, &vravesol[0]);

            if( _probot->CheckSelfCollision() )
                continue;

            if( bCheckEnvCollision && g_pEnviron->CheckCollision(_probot) )
                continue;

            qSolutions.push_back(vector<dReal>());
            qSolutions.back().resize(pmanip->_vecarmjoints.size());
            for(int i = 0; i < (int)sol.size(); ++i)
                qSolutions.back()[i] = sol[i];
        }
    }

    return qSolutions.size()>0;
}

bool ROSArmIK::Solve(const Transform &_T, const dReal* q0, const dReal* pFreeParameters,
                     bool bCheckEnvCollision, dReal* qResult)
{
    if( pFreeParameters == NULL )
        return Solve(_T, q0, bCheckEnvCollision, qResult);

    assert( _probot != NULL );
    
    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();
    assert( pmanip != NULL );
    
    // the world coordinate system is at the origin of the intersection of the first 3 joint axes
    assert( pmanip->_vecarmjoints.size() == _qlower.size() );

    RobotBase::RobotStateSaver saver(_probot);

    _probot->SetActiveDOFs(pmanip->_vecarmjoints);

    // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
    dReal bestdist = 1000; // only valid if q0 != NULL
    
    NEWMAT::Matrix nmT = GetNewMat(_T);
    iksolver->ComputeIK(nmT,_qlower[0] + (_qupper[0]-_qlower[0])*pFreeParameters[0]);
    
    vector<dReal> vravesol(_probot->GetActiveDOF());
    vector<double>* pbest = NULL;
    FOREACH(itsol, iksolver->solution_ik_) {
        vector<double>& sol = *itsol;
        assert( (int)sol.size() == _probot->GetActiveDOF());
        
        // find the first valid solutino that satisfies joint constraints and collisions
        int j;
        for(j = 0; j < (int)pmanip->_vecarmjoints.size(); ++j) {
            if( sol[j] < _qlower[j] || sol[j] > _qupper[j] )
                break;
        }
        
        if( j < (int)pmanip->_vecarmjoints.size() )
            continue; // out of bounds
        
        // check for self collisions (does WAM ever self-collide?)
        for(int i = 0; i < (int)sol.size(); ++i)
            vravesol[i] = sol[i];
        _probot->SetActiveDOFValues(NULL, &vravesol[0]);
        
        if( _probot->CheckSelfCollision() )
            continue;
        
        COLLISIONREPORT report;
        if( bCheckEnvCollision && g_pEnviron->CheckCollision(_probot, &report) ) {
            if( report.plink1 != NULL && report.plink2 != NULL ) {
                RAVELOG(L"WAMIK: collision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
            }
            continue;
        }
        
        // solution is valid, check with q0
        if( q0 != NULL ) {
            
            dReal d = 0;
            
            for(int k = 0; k < (int)pmanip->_vecarmjoints.size(); ++k)
                d += SQR(vravesol[k]-q0[k]);
            
            if( bestdist > d ) {
                pbest = &sol;
                bestdist = d;
            }
        }
        else {
            pbest = &sol;
            break;
        }
    }

    // return as soon as a solution is found, since we're visiting phis starting from q0[0], we are guaranteed
    // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
    if( pbest != NULL ) {
        if( qResult != NULL ) {
            for(int i = 0; i < (int)pbest->size(); ++i)
                qResult[i] = (*pbest)[i];
        }
        return true;
    }
    
    return false;
}

bool ROSArmIK::Solve(const Transform &_T, const dReal* pFreeParameters,
                     bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
{
    if( pFreeParameters == NULL )
        return Solve(_T, bCheckEnvCollision, qSolutions);

        assert( _probot != NULL );
    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();

    assert( pmanip != NULL );
    
    qSolutions.resize(0);

    // the world coordinate system is at the origin of the intersection of the first 3 joint axes
    assert( pmanip->_vecarmjoints.size() && _qlower.size() );

    RobotBase::RobotStateSaver saver(_probot);

    _probot->SetActiveDOFs(pmanip->_vecarmjoints);
    vector<dReal> vravesol(_probot->GetActiveDOF());

    // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
    NEWMAT::Matrix nmT = GetNewMat(_T);
    iksolver->ComputeIK(nmT,_qlower[0] + (_qupper[0]-_qlower[0])*pFreeParameters[0]);
    
    FOREACH(itsol, iksolver->solution_ik_) {
        vector<double>& sol = *itsol;
        assert( (int)sol.size() == _probot->GetActiveDOF());

        // find the first valid solutino that satisfies joint constraints and collisions
        int j;   
        for(j = 0; j < (int)pmanip->_vecarmjoints.size(); ++j) {
            if( sol[j] < _qlower[j] || sol[j] > _qupper[j] )
                break;
        }
        
        if( j < (int)pmanip->_vecarmjoints.size() )
            continue; // out of bounds
        
        // check for self collisions
        for(int i = 0; i < (int)sol.size(); ++i)
            vravesol[i] = sol[i];
        _probot->SetActiveDOFValues(NULL, &vravesol[0]);
        
        if( _probot->CheckSelfCollision() )
            continue;
        
        if( bCheckEnvCollision && g_pEnviron->CheckCollision(_probot) )
            continue;
        
        qSolutions.push_back(vravesol);
    }

    return qSolutions.size()>0;
}

bool ROSArmIK::GetFreeParameters(dReal* pFreeParameters) const
{
    assert( _probot != NULL && pFreeParameters != NULL );

    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();
    if( pmanip == NULL )
        return false;
    assert( pmanip->_vecarmjoints.size() > 0 && pmanip->_vecarmjoints[0] < _probot->GetDOF());
    assert( _qlower.size() > 0 && _qupper.size() > 0 );

    dReal values[3];
    _probot->GetJointFromDOFIndex(pmanip->_vecarmjoints[0])->GetValues(values);
    pFreeParameters[0] = (values[0]-_qlower[0])*fiFreeParam;
    return true;
}

NEWMAT::Matrix ROSArmIK::GetNewMat(const TransformMatrix& tm)
{
    NEWMAT::Matrix nmT(4,4);
    nmT(1,1) = tm.m[0]; nmT(1,2) = tm.m[1]; nmT(1,3) = tm.m[2]; nmT(1,4) = tm.trans.x+voffset.x;
    nmT(2,1) = tm.m[4]; nmT(2,2) = tm.m[5]; nmT(2,3) = tm.m[6]; nmT(2,4) = tm.trans.y+voffset.y;
    nmT(3,1) = tm.m[8]; nmT(3,2) = tm.m[9]; nmT(3,3) = tm.m[10]; nmT(3,4) = tm.trans.z+voffset.z;
    nmT(4,1) = 0; nmT(4,2) = 0; nmT(4,3) = 0; nmT(4,4) = 1;
    return nmT;
}
