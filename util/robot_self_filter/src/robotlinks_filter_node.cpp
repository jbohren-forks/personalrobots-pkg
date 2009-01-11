// Software License Agreement (BSD License)
// Copyright (c) 2008, Rosen Diankov (rdiankov@cs.cmu.edu)
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
/// \author Rosen Diankov (rdiankov@cs.cmu.edu)

#include <vector>
#include <sstream>
#include <cstdio>

#include <ros/node.h>
#include <std_msgs/PointCloud.h>
#include <tf/transform_listener.h>

#include <openrave-core.h>

#include <cstdio>
#include <cstdlib>

extern "C"
{
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
}

using namespace OpenRAVE;
using namespace std;

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)

boost::shared_ptr<ros::node> s_pmasternode;

inline string _stdwcstombs(const wchar_t* pname)
{
    string s;
    size_t len = wcstombs(NULL, pname, 0);
    if( len != (size_t)-1 ) {
        s.resize(len);
        wcstombs(&s[0], pname, len);
    }

    return s;
}

class RobotLinksFilter
{
public:
    struct LINK
    {
        string tfframe;
        vector<Vector> vconvexhull; ///< convex hull of the link
        Transform tstart, tend;
        vector<Vector> vnewconvexhull;
    };
    struct LASERPOINT
    {
        LASERPOINT() {}
        LASERPOINT(const Vector& ptnew, dReal timenew) : pt(ptnew), time(timenew),inside(0) {}
        Vector pt;
        dReal time; // 0-1 value specifying (stamp-stampstart)/(stampend-stampstart) where stamp is time of point,
                    // startstart and startend are start and end times of the entire scan
        int inside; // inside robot frame
    };

    RobotLinksFilter(const string& robotname, dReal convexpadding, bool bAccurateTiming) : _robotname(robotname), _convexpadding(convexpadding), _bAccurateTiming(bAccurateTiming)
    {
        if( InitRobotLinksFromOpenRAVE(robotname) ) {
            double tf_cache_time_secs;
            s_pmasternode->param("~tf_cache_time_secs", tf_cache_time_secs, 10.0);
            if (tf_cache_time_secs < 0)
                ROS_ERROR("RobotLinksFilter: Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs);
            unsigned long long tf_cache_time = tf_cache_time_secs*1000000000ULL;
            _tf.reset(new tf::TransformListener(*s_pmasternode, true, tf_cache_time));
            ROS_INFO("RobotLinksFilter: TF Cache Time: %f Seconds", tf_cache_time_secs);

            // **** Set TF Extrapolation Limit ****
            double tf_extrap_limit_secs ;
            s_pmasternode->param("~tf_extrap_limit", tf_extrap_limit_secs, 0.00);
            if (tf_extrap_limit_secs < 0.0)
                ROS_ERROR("RobotLinksFilter: parameter tf_extrap_limit<0 (%f)", tf_extrap_limit_secs);

            ros::Duration extrap_limit;
            extrap_limit.fromSec(tf_extrap_limit_secs);
            _tf->setExtrapolationLimit(extrap_limit);
            ROS_INFO("RobotLinksFilter: tf extrapolation Limit: %f Seconds", tf_extrap_limit_secs);
        }
        else
            ROS_ERROR("failed to init robot %s", robotname.c_str());

        s_pmasternode->subscribe("tilt_laser_cloud_filtered", _pointcloudin,  &RobotLinksFilter::PointCloudCallback, this, 2);
        s_pmasternode->advertise<std_msgs::PointCloud> ("robotlinks_cloud_filtered", 10);
    }
    virtual ~RobotLinksFilter()
    {
        if( s_pmasternode != NULL ) {
            s_pmasternode->unsubscribe("tilt_laser_cloud_filtered");
            s_pmasternode->unadvertise("robotlinks_cloud_filtered");
        }
    }

    bool InitRobotLinksFromOpenRAVE(const string& robotname)
    {
        _vLinkHulls.clear();

        if( robotname.size() == 0 )
            return false;

        ROS_INFO("opening OpenRAVE robot file %s", robotname.c_str());
        
        // create the main environment
        boost::shared_ptr<EnvironmentBase> penv(CreateEnvironment());
        if( !penv )
            return false;

        // load the scene
        if( !penv->Load(robotname.c_str()) ) {
            ROS_ERROR("RobotLinksFilter failed create robot %s", robotname.c_str());
            return false;
        }

        // get the first robot
        if( penv->GetRobots().size() == 0 ) {
            ROS_ERROR("RobotLinksFilter no robots in file %s", robotname.c_str());
            return false;
        }
        
        RobotBase* probot = penv->GetRobots().front();    
        ROS_INFO("generating convex hulls for robot %S, num links: %d", probot->GetName(), probot->GetLinks().size());

        ros::Time starthull = ros::Time::now();
        _vLinkHulls.resize(probot->GetLinks().size());
        vector<LINK>::iterator ithull = _vLinkHulls.begin();
        size_t totalplanes = 0;
        FOREACH(itlink, probot->GetLinks()) {
            // compute convex hull
            if( compute_convex_hull((*itlink)->GetCollisionData().vertices, ithull->vconvexhull) ) {
                totalplanes += ithull->vconvexhull.size();
                ROS_DEBUG("link %S convex hull has %d planes", (*itlink)->GetName(), ithull->vconvexhull.size());
            }
            else
                ROS_ERROR("failed to compute convex hull for link %S", (*itlink)->GetName());

            ithull->tfframe = _stdwcstombs((*itlink)->GetName());
            ++ithull;
        }

        ROS_INFO("total convex planes: %d, time: %fs", totalplanes, (ros::Time::now()-starthull).toSec());

        return true;
    }

    bool InitRobotLinksFromURDF(const string& robotname)
    {
        ROS_ERROR("InitRobotLinksFromURDF not implemented yet!");
        return false;
    }

private:

    void PointCloudCallback()
    {
        if( _vLinkHulls.size() == 0 || !_tf) {
            // just pass the data
            s_pmasternode->publish("robotlinks_cloud_filtered",_pointcloudin);
            return;
        }

        if( _pointcloudin.pts.size() == 0 )
            return;

        ros::Time stampprocess = ros::Time::now();

        if( _bAccurateTiming )
            PruneWithAccurateTiming(_pointcloudin, _vlaserpoints);
        else
            PruneWithSimpleTiming(_pointcloudin, _vlaserpoints);

        int totalpoints = 0;
        FOREACH(itpoint, _vlaserpoints)
            totalpoints += itpoint->inside==0;

        _pointcloudout.header = _pointcloudin.header;
        _pointcloudout.set_pts_size(totalpoints);
        _pointcloudout.set_chan_size(_pointcloudin.chan.size());
        for(int ichan = 0; ichan < (int)_pointcloudin.chan.size(); ++ichan) {
            _pointcloudout.chan[ichan].name = _pointcloudin.chan[ichan].name;
            _pointcloudout.chan[ichan].set_vals_size(totalpoints);
        }

        for(int oldindex = 0, newindex = 0; oldindex < (int)_vlaserpoints.size(); ++oldindex) {
            if( _vlaserpoints[oldindex].inside )
                continue;

            _pointcloudout.pts[newindex] = _pointcloudin.pts[oldindex];
            for(int ichan = 0; ichan < (int)_pointcloudin.chan.size(); ++ichan)
                _pointcloudout.chan[ichan].vals[newindex] = _pointcloudin.chan[ichan].vals[oldindex];
            ++newindex;
        }

        ROS_INFO("published %d points, processing time=%fs", totalpoints, (ros::Time::now()-stampprocess).toSec());
        s_pmasternode->publish("robotlinks_cloud_filtered",_pointcloudout);
    }

    /// prune all the points that are inside the convex hulls of the robot links
    /// Uses a different timestamp for every laser point cloud
    void PruneWithAccurateTiming(const std_msgs::PointCloud& pointcloudin, vector<LASERPOINT>& vlaserpoints)
    {
        int istampchan = 0;
        while(istampchan < (int)pointcloudin.chan.size()) {
            if( pointcloudin.chan[istampchan].name == "stamps" )
                break;
            ++istampchan;
        }

        if( istampchan >= (int)pointcloudin.chan.size()) {
            ROS_DEBUG("accurate timing needs 'stamp' channel to be published in point cloud, reverting to simple timing");
            PruneWithSimpleTiming(pointcloudin, vlaserpoints);
            return;
        }

        // look for timestamp channel
        float fdeltatime = 0;
        for(int i = 0; i < (int)pointcloudin.chan[istampchan].vals.size(); ++i)
            fdeltatime = pointcloudin.chan[istampchan].vals[i];
            
        if( fdeltatime == 0 ) {
            PruneWithSimpleTiming(pointcloudin, vlaserpoints);
            return;
        }

        vlaserpoints.resize(pointcloudin.pts.size());
        float ideltatime=1.0f/fdeltatime;
        int index = 0;
        FOREACH(itp, pointcloudin.pts) {
            vlaserpoints[index] = LASERPOINT(Vector(itp->x,itp->y,itp->z,1),pointcloudin.chan[istampchan].vals[index]*ideltatime);
            ++index;
        }

        ros::Time stampstart = pointcloudin.header.stamp, stampend = pointcloudin.header.stamp+ros::Duration().fromSec(fdeltatime);
        tf::Stamped<btTransform> bttransform;

        try {
            FOREACH(ithull, _vLinkHulls) {
                _tf->lookupTransform(pointcloudin.header.frame_id, ithull->tfframe, stampstart, bttransform);
                ithull->tstart = GetTransform(bttransform);
                _tf->lookupTransform(pointcloudin.header.frame_id, ithull->tfframe, stampend, bttransform);
                ithull->tend = GetTransform(bttransform);
            }
        }
        catch(tf::TransformException& ex) {
            ROS_WARN("failed to get tf frame");
            return;
        }

        // points are independent from each and loop can be parallelized
        #pragma omp parallel for schedule(dynamic,64)
        for(int i = 0; i < (int)vlaserpoints.size(); ++i) {
            LASERPOINT& laserpoint = vlaserpoints[i];
            FOREACH(ithull, _vLinkHulls) {
                Transform tinv, tinvstart = ithull->tstart.inverse(), tinvend = ithull->tend.inverse();
                tinv.rot = dQSlerp(tinvstart.rot,tinvend.rot,laserpoint.time);
                tinv.trans = tinvstart.trans*(1-laserpoint.time) + tinvend.trans*laserpoint.time;

                bool bInside = true;
                FOREACH(itplane, ithull->vconvexhull) {
                    Vector v = tinv * laserpoint.pt; v.w = 1;
                    if( dot4(*itplane,v) > 0 ) {
                        bInside = false;
                        break;
                    }
                }

                if( bInside ) {
                    laserpoint.inside = 1;
                    break;
                }
            }
        }
    }

    /// prune all the points that are inside the convex hulls of the robot links
    /// Uses the header timestamp for all laser point clouds
    void PruneWithSimpleTiming(const std_msgs::PointCloud& pointcloudin, vector<LASERPOINT>& vlaserpoints)
    {
        tf::Stamped<btTransform> bttransform;
        vlaserpoints.resize(0);
        
        // compute new hulls
        try {
            FOREACH(ithull, _vLinkHulls) {
                _tf->lookupTransform(pointcloudin.header.frame_id, ithull->tfframe, pointcloudin.header.stamp, bttransform);
                ithull->tstart = GetTransform(bttransform);
            }
        }
        catch(tf::TransformException& ex) {
            ROS_WARN("failed to get tf frame");
            return;
        }
        
        vlaserpoints.resize(pointcloudin.pts.size());

        int index = 0;
        FOREACH(itp, pointcloudin.pts)
            vlaserpoints[index++] = LASERPOINT(Vector(itp->x,itp->y,itp->z,1),0);

        FOREACH(ithull, _vLinkHulls) {
            TransformMatrix tinvstart = ithull->tstart.inverse();
            ithull->vnewconvexhull.resize(ithull->vconvexhull.size());
            vector<Vector>::iterator itnewplane = ithull->vnewconvexhull.begin();
            FOREACH(itplane, ithull->vconvexhull) {
                itnewplane->x = itplane->x*tinvstart.m[0]+itplane->y*tinvstart.m[4]+itplane->z*tinvstart.m[8];
                itnewplane->y = itplane->x*tinvstart.m[1]+itplane->y*tinvstart.m[5]+itplane->z*tinvstart.m[9];
                itnewplane->z = itplane->x*tinvstart.m[2]+itplane->y*tinvstart.m[6]+itplane->z*tinvstart.m[10];
                itnewplane->w = itplane->x*tinvstart.trans.x+itplane->y*tinvstart.trans.y+itplane->z*tinvstart.trans.z+itplane->w;
                ++itnewplane;
            }
        }
                    
        // points are independent from each and loop can be parallelized
        #pragma omp parallel for schedule(dynamic,64)
        for(int i = 0; i < (int)vlaserpoints.size(); ++i) {
            LASERPOINT& laserpoint = vlaserpoints[i];
            FOREACH(ithull, _vLinkHulls) {

                bool bInside = true;
                FOREACH(itplane, ithull->vnewconvexhull) {
                    if( dot4(*itplane,laserpoint.pt) > 0 ) {
                        bInside = false;
                        break;
                    }
                }

                if( bInside ) {
                    laserpoint.inside = 1;
                    break;
                }
            }
        }
    }

    bool compute_convex_hull(const vector<Vector>& verts, vector<Vector>& vconvexplanes)
    {
        if( verts.size() <= 3 )
            return false;

        vconvexplanes.resize(0);

        int dim = 3;  	              // dimension of points
        vector<coordT> qpoints(3*verts.size());
        for(size_t i = 0; i < verts.size(); ++i) {
            qpoints[3*i+0] = verts[i].x;
            qpoints[3*i+1] = verts[i].y;
            qpoints[3*i+2] = verts[i].z;
        }
        
        bool bSuccess = false;
        boolT ismalloc = 0;           // True if qhull should free points in qh_freeqhull() or reallocation  
        char flags[]= "qhull Tv"; // option flags for qhull, see qh_opt.htm 
        FILE *outfile = NULL;    // stdout, output from qh_produce_output(), use NULL to skip qh_produce_output()  
        FILE *errfile = tmpfile();    // stderr, error messages from qhull code  
        
        int exitcode= qh_new_qhull (dim, qpoints.size()/3, &qpoints[0], ismalloc, flags, outfile, errfile);
        if (!exitcode) { // no error
            vconvexplanes.reserve(100);

            facetT *facet;	          // set by FORALLfacets 
            FORALLfacets { // 'qh facet_list' contains the convex hull
                vconvexplanes.push_back(Vector(facet->normal[0], facet->normal[1], facet->normal[2], facet->offset-_convexpadding));
            }

            bSuccess = true;
        }

        qh_freeqhull(!qh_ALL);
        int curlong, totlong;	  // memory remaining after qh_memfreeshort 
        qh_memfreeshort (&curlong, &totlong);
        if (curlong || totlong)
            ROS_ERROR("qhull internal warning (main): did not free %d bytes of long memory (%d pieces)", totlong, curlong);
     
        return bSuccess;
    }

    Transform GetTransform(const btTransform& bt)
    {
        btQuaternion q = bt.getRotation();
        btVector3 o = bt.getOrigin();
        return Transform(Vector(q.w(),q.x(),q.y(),q.z()),Vector(o.x(),o.y(),o.z()));
    }

    vector<LINK> _vLinkHulls;
    boost::shared_ptr<tf::TransformListener> _tf;
    std_msgs::PointCloud _pointcloudin, _pointcloudout;
    vector<LASERPOINT> _vlaserpoints;
    string _robotname;
    dReal _convexpadding;
    bool _bAccurateTiming; ///< if true, will interpolate the convex hulls for every time stamp
};

int main(int argc, char ** argv)
{
    // parse the command line options
    string robotname;
    dReal padding = 0.01; // 0.01m padding
    bool bAccurateTiming = false;
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0 ) {
            // print the arguments and exit
            printf("robotlinks_filter_node [--robotfile openravefile] [--padding distance] [--accuratetiming enable]\n"
                   "  Start a node to prune points that are on the robot surface.\n"
                   "  Currently the robot file specified has to be in OpenRAVE format\n"
                   "  (see openrave_robot_descirption ros package for details)\n");

            return 0;
        }
        if( strcmp(argv[i], "--robotfile") == 0 ) {
            robotname = argv[i+1];
            i += 2;
        }
        else if( strcmp(argv[i], "--padding") == 0 ) {
            padding = atof(argv[i+1]);
            i += 2;
        }
        else if( strcmp(argv[i], "--accuratetiming") == 0 ) {
            bAccurateTiming = atoi(argv[i+1])>0;
            i += 2;
        }
        else
            break;
    }

    ros::init(argc,argv);
    s_pmasternode.reset(new ros::node("robobtlinks_filter"));

    if( !s_pmasternode->check_master() )
        return -1;
    
    boost::shared_ptr<RobotLinksFilter> plinksfilter(new RobotLinksFilter(robotname, padding, bAccurateTiming));

    s_pmasternode->spin();

    plinksfilter.reset();
    ros::fini();
    s_pmasternode.reset();
    return 0;
}
