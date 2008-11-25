#include <openrave-core.h>
#include <vector>
using namespace OpenRAVE;
using namespace std;

int main(int argc, char ** argv)
{
    if( argc < 2 )
        return -1; // no robots to load

    EnvironmentBase* penv = CreateEnvironment();
    // load the scene
    if( !penv->Load(argv[1]) )
        return -2;

    // choose collision checker
    if( argc > 2 ) {
        CollisionCheckerBase* pchecker = penv->CreateCollisionChecker(argv[2]);
        if( pchecker == NULL ) {
            RAVEPRINT(L"failed to create checker %s\n", argv[2]);
            return -3;
        }
        penv->SetCollisionChecker(pchecker);
    }

    // get the first robot
    if( penv->GetRobots().size() == 0 )
        return -3;

    RobotBase* probot = penv->GetRobots().front();
    
    vector<dReal> values;
    probot->GetJointValues(values);

    values[1] = 1;
    probot->SetJointValues(NULL,NULL,&values[0]);
    
    COLLISIONREPORT report;
    if( probot->CheckSelfCollision(&report) ) {
        RAVEPRINT(L"robot in self-collision %S:%S at %d contacts\n",
                  report.plink1 != NULL ? report.plink1->GetName() : L"",
                  report.plink2 != NULL ? report.plink2->GetName() : L"", report.contacts.size());
    }

    // get the transformations of all the links
    vector<Transform> vlinktransforms;
    probot->GetBodyTransformations(vlinktransforms);

    return 0;
}
