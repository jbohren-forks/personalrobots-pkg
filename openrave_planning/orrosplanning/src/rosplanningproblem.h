#ifndef OPENRAVE_ROS_PLANNING_PROBLEM
#define OPENRAVE_ROS_PLANNING_PROBLEM

class ROSPlanningProblem : public CmdProblemInstance
{
public:
    ROSPlanningProblem(EnvironmentBase* penv) : CmdProblemInstance(penv)
    {
        PhaseSpaceMocapClient::RegisterXMLReader(GetEnv());
        
        RegisterCommand("createsystem",(CommandFn)&ROSPlanningProblem::CreateSystem, "creates a sensor system and initializes it with the current bodies");
    }

    virtual void Destroy()
    {
        listsystems.clear();
    }

    int main(const char* cmd)
    {
        return 0;
    }

    bool CreateSystem(ostream& sout, istream& sinput)
    {
        string systemname;
        sinput >> systemname;
        if( !sinput )
            return false;

        boost::shared_ptr<SensorSystemBase> psystem(GetEnv()->CreateSensorSystem(systemname.c_str()));
        if( !psystem )
            return false;

        if( !psystem->Init(sinput) )
            return false;

        psystem->AddRegisteredBodies(GetEnv()->GetBodies());
        listsystems.push_back(psystem);

        RAVELOG_DEBUGA("added %s system\n", systemname.c_str());
        sout << 1; // signal success
        return true;
    }

protected:
    list<boost::shared_ptr<SensorSystemBase> > listsystems;
};

#endif
