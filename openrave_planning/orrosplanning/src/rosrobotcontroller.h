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
#ifndef RAVE_ROS_ROBOT_CONTROLLER
#define RAVE_ROS_ROBOT_CONTROLLER

#include <robot_msgs/MechanismState.h>

// controller for the SSC-32 board
class ROSRobotController : public ControllerBase
{
    enum ControllerState{
        None = 0,
        Servo, // done when servoed to position and the position is held
        Traj, // done when reaches last point
        Velocity // done when joints stop moving
    };

public:
    ROSRobotController(EnvironmentBase* penv) : ControllerBase(penv), _topic("mechanism_state"), _fTimeCommandStarted(0),
        _ptraj(NULL), _bIsDone(false), _bSendTimestamps(false), _bSubscribed(false), _bCalibrated(false) {
    }

    virtual ~ROSRobotController() {
        Destroy();
    }

    /// args format: host port [proxytype index]
    /// where proxytype is actarray, pos2d, or ...
    /// the order specified is the order the degrees of freedom will be arranged
    virtual bool Init(RobotBase* robot, const char* args)
    {
        Destroy();
        _probot = robot;
        if( _probot == NULL )
            return false;

        startsubscriptions();

        return _bSubscribed;
    }

    virtual void Destroy()
    {
        stopsubscriptions();
        _bCalibrated = false;
        _probot = NULL;
        _bIsDone = false;
    }

    virtual void Reset(int options)
    {
    }

    virtual bool SetDesired(const dReal* pValues)
    {
        return true;
    }

    virtual bool SetPath(const Trajectory* ptraj)
    {
        return true;
    }

    virtual bool SetPath(const Trajectory* ptraj, int nTrajectoryId, float fDivergenceTime)
    {
        RAVELOG_ERRORA("ros controller does not support path with divergence");
        return false;
    }

    virtual int GetDOF() { return _probot != NULL ? _probot->GetDOF() : 0; }

    virtual bool SimulationStep(float fTimeElapsed)
    {
        if( !_bSubscribed )
            return false;

        {
            boost::mutex::scoped_lock lock(_mutexstate);
            if( _bCalibrated ) {
                vector<dReal> values;
                values.reserve(_mstate.get_joint_states_size());
                FOREACHC(itj, _vjointmap)
                    values.push_back(_mstate.joint_states[*itj].position);

                ROS_ASSERT( (int)values.size() == _probot->GetDOF() );
                _probot->SetJointValues(NULL, NULL, &values[0], true);
            }
        }

        return IsDone();
    }

    virtual bool SendCmd(const char* pcmd)
    {
        return false;
    }

    virtual bool SupportsCmd(const char* pcmd)
    {
        return false;
    }

    virtual bool IsDone() { return _bIsDone; }

    virtual ActuatorState GetActuatorState(int index)
    {
        return AS_Idle;
    }

    virtual float GetTime() const
    {
        boost::mutex::scoped_lock lock(_mutexstate);
        return _mstate.time - _fTimeCommandStarted;
    }
    virtual RobotBase* GetRobot() const { return _probot; }

    virtual void GetVelocity(std::vector<dReal>& vel) const
    {
        vel.resize(0);
        if( !_bCalibrated )
            return;

        boost::mutex::scoped_lock lock(_mutexstate);
        assert( (int)_vjointmap.size() == _probot->GetDOF() );

        vel.reserve(_mstate.get_joint_states_size());
        FOREACHC(itj, _vjointmap)
            vel.push_back(_mstate.joint_states[*itj].velocity);
    }
    
    virtual void GetTorque(std::vector<dReal>& torque) const
    {
        torque.resize(0);
        if( !_bCalibrated )
            return;

        boost::mutex::scoped_lock lock(_mutexstate);
        assert( (int)_vjointmap.size() == _probot->GetDOF() );

        torque.reserve(_mstate.get_joint_states_size());
        FOREACHC(itj, _vjointmap)
            torque.push_back(_mstate.joint_states[*itj].applied_effort); // commanded_effort?
    }

private:

    virtual void startsubscriptions()
    {
        // check if thread launched
        _bSubscribed = false;
        ros::node* pnode = check_roscpp();
        if( pnode != NULL ) {
            _bSubscribed = pnode->subscribe(_topic, _mstate_cb, &ROSRobotController::mechanismstatecb, this, 10);
            if( _bSubscribed )
                RAVELOG_DEBUGA("subscribed to %s\n", _topic.c_str());
            else
                RAVELOG_ERRORA("failed to subscribe to %s\n", _topic.c_str());
        }
    }

    virtual void stopsubscriptions()
    {
        if( _bSubscribed ) {
            ros::node* pnode = check_roscpp_nocreate();
            if( pnode != NULL ) {
                pnode->unsubscribe(_topic.c_str());
                RAVELOG_DEBUGA("unsubscribe from %s\n", _topic.c_str());
            }
            _bSubscribed = false;
        }
    }

    virtual void mechanismstatecb()
    {
        if( !_bCalibrated ) {
            // check the robot joint/link names
            GetEnv()->LockPhysics(true);
            
            do {
                _vjointmap.resize(0);
                for(int i = 0; i < _probot->GetDOF(); ++i) {
                    for(size_t j = 0; j < _mstate_cb.get_joint_states_size(); ++j) {
                        if( _stdwcstombs(_probot->GetJoints()[i]->GetName()) == _mstate_cb.joint_states[j].name ) {
                            _vjointmap.push_back(j);
                            break;
                        }
                    }
                    
                    if( i+1 != (int)_vjointmap.size()) {
                        RAVELOG_WARNA("could not find robot joint %S in mechanism state\n", _probot->GetJoints()[i]->GetName());
                        break;
                    }
                }

                if( _probot->GetDOF() != (int)_vjointmap.size() ) {
                    _vjointmap.resize(0);
                    break;
                }

                _bCalibrated = true;
            } while(0);

            GetEnv()->LockPhysics(false);
        }
        else {
            if( _probot->GetDOF() != (int)_mstate_cb.get_joint_states_size() )
                _bCalibrated = false;
        }

        if( !_bCalibrated )
            return;

        {
            boost::mutex::scoped_lock lock(_mutexstate);
            _mstate = _mstate_cb;
        }

        // do some monitoring of the joint state (try to look for stalls)
    }

    RobotBase* _probot;           ///< robot owning this controller

    string _topic;
    robot_msgs::MechanismState _mstate_cb, _mstate;
    vector<dReal> _vecdesired;
    mutable boost::mutex _mutexstate;

    ofstream flog;
    int logid;
    
    vector<int> _vjointmap; ///< maps mechanism state joints to robot joints

    double _fTimeCommandStarted;
    const Trajectory* _ptraj;

    bool _bIsDone;
    bool _bSendTimestamps; ///< if true, will send timestamps along with traj
    bool _bSubscribed; ///< if true, succesfully subscribed to the mechanism state msgs
    bool _bCalibrated; ///< if true, mechanism state matches robot
};

#endif
