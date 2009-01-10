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
#ifndef RAVE_ROS_ROBOT_CONTROLLER
#define RAVE_ROS_ROBOT_CONTROLLER

#include <robot_msgs/MechanismState.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>
#include <pr2_mechanism_controllers/TrajectoryWait.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>

class ROSRobotController : public ControllerBase
{
    enum ControllerState {
        None = 0,
        Servo, // done when servoed to position and the position is held
        Traj, // done when reaches last point
        Velocity // done when joints stop moving
    };

    class TrajectoryController
    {
    public:
        TrajectoryController(RobotBase* probot, const string& strTrajectoryServiceDir) : _probot(probot), _strTrajectoryServiceDir(strTrajectoryServiceDir) {}
        virtual ~TrajectoryController() {
            Destroy();
        }

        bool Init(ros::node* pnode)
        {
            assert(pnode != NULL);
            Destroy();
            
            _srvTrajectoryStart = ros::service::createHandle<pr2_mechanism_controllers::TrajectoryStart::request, pr2_mechanism_controllers::TrajectoryStart::response>(_strTrajectoryServiceDir+"TrajectoryStart", true);
            _srvTrajectoryCancel = ros::service::createHandle<pr2_mechanism_controllers::TrajectoryCancel::request, pr2_mechanism_controllers::TrajectoryCancel::response>(_strTrajectoryServiceDir+"TrajectoryCancel", true);
            _srvTrajectoryWait = ros::service::createHandle<pr2_mechanism_controllers::TrajectoryWait::request, pr2_mechanism_controllers::TrajectoryWait::response>(_strTrajectoryServiceDir+"TrajectoryWait", true);
            _srvTrajectoryQuery = ros::service::createHandle<pr2_mechanism_controllers::TrajectoryQuery::request, pr2_mechanism_controllers::TrajectoryQuery::response>(_strTrajectoryServiceDir+"TrajectoryQuery", true);
        
            if( !_srvTrajectoryQuery ) {
                RAVELOG_ERRORA("failed to find %s service\n", (_strTrajectoryServiceDir+"TrajectoryQuery").c_str());
                return false;
            }
        
            pr2_mechanism_controllers::TrajectoryQuery::request req;
            pr2_mechanism_controllers::TrajectoryQuery::response res;
            if( !_srvTrajectoryQuery->call(req,res) ) {
                RAVELOG_ERRORA("failed to query trajectory service %s", _strTrajectoryServiceDir.c_str());
                return false;
            }

            if( res.jointnames.size() == 0 ) {
                RAVELOG_ERRORA("no joint names for trajectory service %s", _strTrajectoryServiceDir.c_str());
                return false;
            }

            vector<string> vrobotjoints(_probot->GetJoints().size());
            for(size_t i = 0; i < _probot->GetJoints().size(); ++i)
                vrobotjoints[i] = _stdwcstombs(_probot->GetJoints()[i]->GetName());

            _vjointmap.reserve(res.jointnames.size());
            FOREACH(itname, res.jointnames) {
                vector<string>::iterator itindex = find(vrobotjoints.begin(), vrobotjoints.end(), *itname);
                if( itindex == vrobotjoints.end() ) {            
                    RAVELOG_ERRORA("failed to find joint %s\n", itname->c_str());
                    Destroy();
                    return false;
                }
                _vjointmap.push_back((int)(itindex-vrobotjoints.begin()));
            }

            return true;
        }

        bool Destroy()
        {
            _vjointmap.clear();
            _listTrajectories.clear();
            _srvTrajectoryStart.reset();
            _srvTrajectoryCancel.reset();
            _srvTrajectoryWait.reset();
            _srvTrajectoryQuery.reset();
            return true;
        }

        void GetTrajPoint(const vector<dReal>& vrobotvalues, pr2_mechanism_controllers::JointTrajPoint& pt)
        {
            pt.positions.resize(_vjointmap.size());
            typeof(pt.positions.begin()) itcontrollerpos = pt.positions.begin();
            FOREACH(itindex, _vjointmap)
                *itcontrollerpos++ = vrobotvalues[*itindex];
        }

        // trajectory services
        RobotBase* _probot;
        string _strTrajectoryServiceDir;
        service::ServiceHandlePtr _srvTrajectoryStart, _srvTrajectoryCancel, _srvTrajectoryWait, _srvTrajectoryQuery;
        vector<int> _vjointmap;
        list<uint32_t> _listTrajectories; ///< trajectories currently pending for completion
    };

public:
    ROSRobotController(EnvironmentBase* penv) : ControllerBase(penv), _topic("mechanism_state"), _fTimeCommandStarted(0),
        _ptraj(NULL), _bIsDone(false), _bSendTimestamps(false), _bSubscribed(false), _bCalibrated(false), _bDestroyThread(false) {
        _threadTrajectories = boost::thread(boost::bind(&ROSRobotController::_TrajectoryThread,this));
    }

    virtual ~ROSRobotController() {
        Destroy();

        _bDestroyThread = true;
        _threadTrajectories.join();
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

        if( args != NULL ) {
            stringstream ss(args);

            string cmd;
            while(!ss.eof()) {
                ss >> cmd;
                if( !ss )
                    break;

                if( stricmp(cmd.c_str(), "joints") == 0 ) {
                    string jointname;
                    while(1) {
                        ss >> jointname;
                        if( !ss )
                            break;

                        // look for the correct index
                        int index = -1;
                        wstring wjointname = _stdmbstowcs(jointname.c_str());
                        for(int i = 0; i < (int)_probot->GetJoints().size(); ++i) {
                            if( wjointname == _probot->GetJoints()[i]->GetName() ) {
                                index = i;
                                break;
                            }
                        }

                        if( index >= 0 )
                            _setEnabledJoints.insert(pair<string, int>(jointname,index));
                        else
                            RAVELOG_WARNA("failed to find joint %s\n", jointname.c_str());
                    }

                    break;
                }
                else if( stricmp(cmd.c_str(), "trajectoryservice") == 0 ) {
                    string servicedir;
                    ss >> servicedir;
                    boost::mutex::scoped_lock lock(_mutexTrajectories);
                    _listControllers.push_back(boost::shared_ptr<TrajectoryController>(new TrajectoryController(_probot, servicedir)));
                }
                else break;

                if( !ss ) {
                    RAVELOG_ERRORA("failed\n");
                    return false;
                }
            }
        }

        if( _setEnabledJoints.size() == 0 ) {
            RAVELOG_DEBUGA("controlling using all joints of the robot\n");

            for(int i = 0; i < _probot->GetDOF(); ++i)
                _setEnabledJoints.insert(pair<string,int>(_stdwcstombs(_probot->GetJoints()[i]->GetName()), i));
        }

        startsubscriptions();

        return _bSubscribed;
    }

    virtual void Destroy()
    {
        stopsubscriptions();
        _bCalibrated = false;
        _probot = NULL;
        _bIsDone = false;
        _setEnabledJoints.clear();

        {
            boost::mutex::scoped_lock lock(_mutexTrajectories);
            _listControllers.clear();
        }
    }

    virtual void Reset(int options)
    {
    }

    virtual bool SetDesired(const dReal* pValues)
    {
        boost::mutex::scoped_lock lock(_mutexTrajectories);

        // set a path between the current and desired positions
        bool bSuccess = true;
        pr2_mechanism_controllers::TrajectoryStart::request req;
        pr2_mechanism_controllers::TrajectoryStart::response res;
        
        vector<dReal> vnewvalues;
        
        {
            RobotBase::RobotStateSaver saver(_probot);
            _probot->SetJointValues(NULL, NULL, pValues,true);
            _probot->GetJointValues(vnewvalues);
        }

        // check if values are sufficiently different
        if( _vcurvalues.size() == vnewvalues.size() ) {
            dReal fthresh = 0.01;
            bool bSendTraj = false;
            for(size_t i = 0; i < _vcurvalues.size(); ++i) {
                if( RaveFabs(_vcurvalues[i] - vnewvalues[i]) > fthresh ) {
                    bSendTraj = true;
                }
            }

            if( !bSendTraj ) {
                RAVELOG_VERBOSEA("setdesired sent same joint value, ignoring\n");
                return false;
            }
        }
        else
            RAVELOG_WARNA("number of current values (%d) != desird values (%d)\n", _vcurvalues.size(), vnewvalues.size());

        FOREACH(ittrajcontroller, _listControllers) {
            if( !(*ittrajcontroller)->_srvTrajectoryStart ) {
                RAVELOG_ERRORA("no start trajectory service\n");
                bSuccess = false;
                continue;
            }

            //req.requesttiming = 1; // request back the timestamps
            _bIsDone = false;
            req.hastiming = 0;
            req.traj.points.resize(2);
            (*ittrajcontroller)->GetTrajPoint(_vcurvalues, req.traj.points[0]);
            (*ittrajcontroller)->GetTrajPoint(vnewvalues, req.traj.points[1]);

            if( (*ittrajcontroller)->_srvTrajectoryStart->call(req,res) )
                (*ittrajcontroller)->_listTrajectories.push_back(res.trajectoryid);
            else {
                RAVELOG_ERRORA("failed to start trajectory\n");
                bSuccess = false;
                continue;
            }

            RAVELOG_DEBUGA("started trajectory %d\n", res.trajectoryid);
            //res.timestamps // final timestamps
        }

        return true;
    }

    virtual bool SetPath(const Trajectory* ptraj)
    {
        if( ptraj == NULL )
            return false;

        boost::mutex::scoped_lock lock(_mutexTrajectories);

        bool bSuccess = true;
        pr2_mechanism_controllers::TrajectoryStart::request req;
        pr2_mechanism_controllers::TrajectoryStart::response res;
        
        FOREACH(ittrajcontroller, _listControllers) {
            if( !(*ittrajcontroller)->_srvTrajectoryStart ) {
                RAVELOG_ERRORA("no start trajectory service\n");
                bSuccess = false;
                continue;
            }

            //req.requesttiming = 1; // request back the timestamps
            _bIsDone = false;
            req.hastiming = 0;
            req.traj.points.resize(ptraj->GetPoints().size());
            typeof(req.traj.points.begin()) ittraj = req.traj.points.begin(); 
            FOREACHC(itpoint, ptraj->GetPoints())
                (*ittrajcontroller)->GetTrajPoint(itpoint->q, *ittraj++);

            if( (*ittrajcontroller)->_srvTrajectoryStart->call(req,res) )
                (*ittrajcontroller)->_listTrajectories.push_back(res.trajectoryid);
            else {
                RAVELOG_ERRORA("failed to start trajectory\n");
                bSuccess = false;
                continue;
            }

            RAVELOG_DEBUGA("started trajectory %d\n", res.trajectoryid);
            //res.timestamps // final timestamps
        }

        return bSuccess;
    }

    virtual bool SetPath(const Trajectory* ptraj, int nTrajectoryId, float fDivergenceTime)
    {
        RAVELOG_ERRORA("ros controller does not support path with divergence");
        return false;
    }

    virtual bool SimulationStep(float fTimeElapsed)
    {
        if( !_bSubscribed )
            return false;

        {
            boost::mutex::scoped_lock lock(_mutexstate);
            if( _bCalibrated ) {
                vector<dReal> values;
                vector<dReal> vlower, vupper;
                _probot->GetJointValues(values);
                _probot->GetJointLimits(vlower,vupper);
                FOREACHC(itj, _mapjoints) {
                    values[itj->second] = _mstate.joint_states[itj->first].position;
                    while(values[itj->second] > vupper[itj->second] ) {
                        if( values[itj->second]-2*PI >= vlower[itj->second] )
                            values[itj->second] -= 2*PI;
                        else {
                            values[itj->second] = vupper[itj->second];
                            break;
                        }
                    }
                    while(values[itj->second] < vlower[itj->second] ) {
                        if( values[itj->second]+2*PI <= vupper[itj->second] )
                            values[itj->second] += 2*PI;
                        else {
                            values[itj->second] = vlower[itj->second];
                            break;
                        }
                    }
                }

                ROS_ASSERT( (int)values.size() == _probot->GetDOF() );
                _vcurvalues = values;
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

        vel.resize(_probot->GetJoints().size());
        FOREACHC(itj, _mapjoints)
            vel[itj->second] = _mstate.joint_states[itj->first].velocity;
    }
    
    virtual void GetTorque(std::vector<dReal>& torque) const
    {
        torque.resize(0);
        if( !_bCalibrated )
            return;

        boost::mutex::scoped_lock lock(_mutexstate);

        torque.resize(_probot->GetJoints().size());
        FOREACHC(itj, _mapjoints)
            torque[itj->second] = _mstate.joint_states[itj->first].applied_effort; // commanded_effort?
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

            if( _bSubscribed ) {
                boost::mutex::scoped_lock lock(_mutexTrajectories);
                FOREACH(it, _listControllers)
                    (*it)->Init(pnode);
            }
        }
    }

    virtual void stopsubscriptions()
    {
        if( _bSubscribed ) {
            ros::node* pnode = check_roscpp_nocreate();
            if( pnode != NULL ) {
                pnode->unsubscribe(_topic.c_str());
                RAVELOG_DEBUGA("unsubscribe from %s\n", _topic.c_str());

                boost::mutex::scoped_lock lock(_mutexTrajectories);
                FOREACH(it, _listControllers)
                    (*it)->Destroy();
            }
            _bSubscribed = false;
        }
    }

    virtual void mechanismstatecb()
    {
        {
            boost::mutex::scoped_lock lock(_mutexstate);

            if( !_bCalibrated ) {
                // check the robot joint/link names
                do {
                    _mapjoints.clear();
                    FOREACH(itj, _setEnabledJoints) {
                        bool bAdded = false;
                        for(size_t j = 0; j < _mstate_cb.get_joint_states_size(); ++j) {
                            if( itj->first == _mstate_cb.joint_states[j].name ) {
                                _mapjoints[j] = itj->second;
                                bAdded = true;
                                break;
                            }
                        }
                    
                        if( !bAdded ) {
                            RAVELOG_WARNA("could not find robot joint %s in mechanism state\n", itj->first.c_str());
                            break;
                        }
                    }

                    if( _mapjoints.size() != _setEnabledJoints.size() ) {
                        _mapjoints.clear();
                        break;
                    }

                    _bCalibrated = true;
                } while(0);
            }
            else {
                if( _probot->GetDOF() != (int)_mstate_cb.get_joint_states_size() )
                    _bCalibrated = false;
            }

            if( !_bCalibrated )
                return;

            _mstate = _mstate_cb;
        }
        
        // do some monitoring of the joint state (try to look for stalls)
    }

    void _TrajectoryThread()
    {
        while(!_bDestroyThread) {

            pr2_mechanism_controllers::TrajectoryQuery::request req;
            pr2_mechanism_controllers::TrajectoryQuery::response res;

            // check if the first trajectory is done
            boost::mutex::scoped_lock lock(_mutexTrajectories);
            
            if( _listControllers.size() > 0 ) {
                bool bPopTrajectory = true;

                // check if done
                FOREACH(ittraj, _listControllers) {
                    assert( (*ittraj)->_listTrajectories.size() == _listControllers.front()->_listTrajectories.size());
                    if( (*ittraj)->_listTrajectories.size() == 0 ) {
                        bPopTrajectory = false;
                        break;
                    }
                    req.trajectoryid = (*ittraj)->_listTrajectories.front();
                    if( !(*ittraj)->_srvTrajectoryQuery->call(req,res) ) {
                        RAVELOG_ERRORA("trajectory query failed\n");
                        bPopTrajectory = false;
                    }
                    
                    if( !(res.done == pr2_mechanism_controllers::TrajectoryQuery::response::State_Done ||
                        res.done == pr2_mechanism_controllers::TrajectoryQuery::response::State_Deleted ||
                        res.done == pr2_mechanism_controllers::TrajectoryQuery::response::State_Failed ||
                          res.done == pr2_mechanism_controllers::TrajectoryQuery::response::State_Canceled) ) {
                        bPopTrajectory = false;
                        break;
                    }
                }

                if( bPopTrajectory ) {
                    FOREACH(ittraj, _listControllers)
                        (*ittraj)->_listTrajectories.pop_front();
                    RAVELOG_DEBUGA("robot trajectory finished, left: %d\n", _listControllers.front()->_listTrajectories.size());
                }

                if( _bIsDone != _listControllers.front()->_listTrajectories.size()==0 ) {
                    _bIsDone = _listControllers.front()->_listTrajectories.size()==0;
                    RAVELOG_DEBUGA("done: %d\n", _bIsDone);
                }
            }

            usleep(10000); // query every 10ms
        }
    }

    RobotBase* _probot;           ///< robot owning this controller

    string _topic;
    
    robot_msgs::MechanismState _mstate_cb, _mstate;
    vector<dReal> _vecdesired;
    set< pair<string, int> > _setEnabledJoints; // set of enabled joints and their indices
    mutable boost::mutex _mutexstate;
    vector<dReal> _vcurvalues; ///< current robot values

    ofstream flog;
    int logid;
    
    map<int, int> _mapjoints; ///< maps mechanism state joints to robot joints
    list<boost::shared_ptr<TrajectoryController> > _listControllers;

    double _fTimeCommandStarted;
    const Trajectory* _ptraj;

    boost::mutex _mutexTrajectories;
    boost::thread _threadTrajectories;

    bool _bIsDone;
    bool _bSendTimestamps; ///< if true, will send timestamps along with traj
    bool _bSubscribed; ///< if true, succesfully subscribed to the mechanism state msgs
    bool _bCalibrated; ///< if true, mechanism state matches robot
    bool _bDestroyThread; ///< if true, destroy the thread
};

#endif
