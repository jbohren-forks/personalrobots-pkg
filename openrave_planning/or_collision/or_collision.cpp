#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>

using namespace OpenRAVE;
using namespace std;

void printhelp();
void printinterfaces(EnvironmentBase*);

int main(int argc, char ** argv)
{
    if( argc < 2 ) {
        printhelp();
        return -1; // no robots to load
    }

    // create the main environment
    EnvironmentBase* penv = CreateEnvironment();
    vector<dReal> vsetvalues; 

    // parse the command line options
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-?") == 0 || strcmp(argv[i], "/?") == 0 || strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
            printhelp();
            return 0;
        }
        else if( strcmp(argv[i], "--checker") == 0 ) {
            // create requested collision checker
            CollisionCheckerBase* pchecker = penv->CreateCollisionChecker(argv[i+1]);
            if( pchecker == NULL ) {
                RAVEPRINT(L"failed to create checker %s\n", argv[i+1]);
                return -3;
            }
            penv->SetCollisionChecker(pchecker);
            i += 2;
        }
        else if( strcmp(argv[i], "--list") == 0 ) {
            printinterfaces(penv);
            return 0;
        }
        else if( strcmp(argv[i], "--joints") == 0 ) {
            vsetvalues.resize(atoi(argv[i+1]));
            for(int j = 0; j < (int)vsetvalues.size(); ++j)
                vsetvalues[j] = atoi(argv[i+j+2]);

            i += 2+vsetvalues.size();
        }
        else
            break;
    }

    
    if( i >= argc ) {
        RAVEPRINT(L"not enough parameters\n");
        printhelp();
        return -1;
    }

    // load the scene
    if( !penv->Load(argv[i]) ) {
        printhelp();
        return -2;
    }
    
    // get the first robot
    if( penv->GetRobots().size() == 0 )
        return -3;

    RobotBase* probot = penv->GetRobots().front();
    
    vector<dReal> values; probot->GetJointValues(values);
    
    // set new values
    for(int i = 0; i < (int)vsetvalues.size() && i < (int)values.size(); ++i)
        values[i] = vsetvalues[i];

    probot->SetJointValues(NULL,NULL,&values[0],true);

    int contactpoints = 0;
    COLLISIONREPORT report;
    penv->SetCollisionOptions(CO_Contacts);
    if( probot->CheckSelfCollision(&report) ) {
        contactpoints = (int)report.contacts.size();
        wstringstream ss;
        ss << "robot in self-collision "
           << (report.plink1 != NULL ? report.plink1->GetName() : L"") << ":"
           << (report.plink2 != NULL ? report.plink2->GetName() : L"") << " at "
           << contactpoints << "contacts" << endl;
        for(int i = 0; i < contactpoints; ++i) {
            COLLISIONREPORT::CONTACT& c = report.contacts[i];
            ss << "contact" << i << ": pos=("
               << c.pos.x << ", " << c.pos.y << ", " << c.pos.z << "), norm=("
               << c.norm.x << ", " << c.norm.y << ", " << c.norm.z << ")" << endl;
        }
        
        RAVEPRINT(ss.str().c_str());
    }
    else RAVEPRINT(L"robot not in collision\n");

    // get the transformations of all the links
    vector<Transform> vlinktransforms;
    probot->GetBodyTransformations(vlinktransforms);

    return contactpoints;
}

void printhelp()
{
    printf("or_collision [--list] [--checker checker_name] [--joints #values [values]] robot_model\n"
           "  Load a robot into the openrave environment, set it at [joint values] and\n"
           "  check for self collisions. Returns number of contact points.\n"
           "-listplugins             Will list all the loadable interfaces (ie, collision checkers).\n"
           "-checker name            Load a different collision checker instead of the default one\n"
           "-joints #values [values] Set the robot to specific joint values\n");
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

    RAVEPRINT(ss.str().c_str());
}
