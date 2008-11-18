/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <list>

#include <urdf/URDF.h>
#include <libTF/Pose3D.h>

#include <reslocator/reslocator.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

using namespace std;
static list<string> s_listResourceNames;

string values2str(unsigned int count, const double *values, double (*conv)(double) = NULL)
{
    stringstream ss;
    for (unsigned int i = 0 ; i < count ; i++) {
        if (i > 0)
            ss << " ";
        ss << (conv ? conv(values[i]) : values[i]);
    }
    return ss.str();
}

double rad2deg(double f)
{
    return f / 3.1415926 * 180.0f;
}

void setupTransform(libTF::Pose3D &transform, const double *xyz, const double *rpy)
{
    transform.setFromEuler(xyz[0], xyz[1], xyz[2], rpy[2], rpy[1], rpy[0]);
}

void addKeyValue(TiXmlElement *elem, const string& key, const string &value)
{
    TiXmlElement *ekey      = new TiXmlElement(key);
    TiXmlText    *text_ekey = new TiXmlText(value);
    ekey->LinkEndChild(text_ekey);    
    elem->LinkEndChild(ekey); 
}

void addTransform(TiXmlElement *elem, const::libTF::Pose3D& transform)
{
    libTF::Position pz;
    transform.getPosition(pz);
    double cpos[3] = { pz.x, pz.y, pz.z };
    
    libTF::Quaternion q;
    transform.getQuaternion(q);
    double cquat[4] = { q.w, q.x, q.y, q.z };
    
    // set geometry transform
    addKeyValue(elem, "translation", values2str(3, cpos));
    addKeyValue(elem, "quaternion", values2str(4, cquat));  
}

void copyOpenraveMap(const robot_desc::URDF::Map& data, TiXmlElement *elem, const vector<string> *tags = NULL)
{
    vector<string> openrave_names;
    data.getMapTagNames("openrave", openrave_names);
    for (unsigned int k = 0 ; k < openrave_names.size() ; ++k) {
        map<string, string> m = data.getMapTagValues("openrave", openrave_names[k]);
        vector<string> accepted_tags;
        if (tags)
            accepted_tags = *tags;
        else
            for (map<string, string>::iterator it = m.begin() ; it != m.end() ; it++)
                accepted_tags.push_back(it->first);
	
        for (unsigned int i = 0 ; i < accepted_tags.size() ; ++i)
            if (m.find(accepted_tags[i]) != m.end())
                addKeyValue(elem, accepted_tags[i], m[accepted_tags[i]]);
	
        map<string, const TiXmlElement*> x = data.getMapTagXML("openrave", openrave_names[k]);
        for (map<string, const TiXmlElement*>::iterator it = x.begin() ; it != x.end() ; it++) {
            for (const TiXmlNode *child = it->second->FirstChild() ; child ; child = child->NextSibling())
                elem->LinkEndChild(child->Clone());
        }
    }
}

// ignore all collision data for the time being
void convertLink(TiXmlElement *root, robot_desc::URDF::Link *link, const libTF::Pose3D &transform, bool enforce_limits)
{
    libTF::Pose3D currentTransform = transform;
        
    TiXmlElement *body = new TiXmlElement("body");
    body->SetAttribute("name", link->name);
	
    // compute global transform
    libTF::Pose3D localTransform;
    setupTransform(localTransform, link->xyz, link->rpy);
    currentTransform.multiplyPose(localTransform);
    addTransform(body, currentTransform);
	
    // begin create geometry node
    TiXmlElement *geom = new TiXmlElement("geom");

    string type;

    switch(link->visual->geometry->type) {
    case robot_desc::URDF::Link::Geometry::MESH: {
        robot_desc::URDF::Link::Geometry::Mesh* mesh = static_cast<robot_desc::URDF::Link::Geometry::Mesh*>(link->visual->geometry->shape);

        // Trim Both leading and trailing spaces  
        size_t startpos = mesh->filename.find_first_not_of(" \t");
        size_t endpos = mesh->filename.find_last_not_of(" \t");        

        if(( string::npos == startpos ) || ( string::npos == endpos))
            mesh->filename = "";
        else
            mesh->filename = mesh->filename.substr( startpos, endpos-startpos+1 ); 

        if( mesh->filename.empty() ) {
            cerr << "mesh file is empty for link " << link->name << ", adding box" << endl;
            type = "box";
            double extents[3] = {0.01,0.01,0.01};
            addKeyValue(geom, "extents", values2str(3,extents));
        }
        else {
            type = "trimesh";
            stringstream ss;
            
            s_listResourceNames.push_back(mesh->filename + "_hi.iv");
            s_listResourceNames.push_back(mesh->filename + "_low.iv");
            
            ss << mesh->filename << "_hi.iv " << mesh->scale[0];
            addKeyValue(geom, "render", ss.str());

            ss.str("");
            ss << mesh->filename << "_low.iv " << mesh->scale[0];
            addKeyValue(geom, "data", ss.str());
        }
        break;
    }
    case robot_desc::URDF::Link::Geometry::BOX: {
        type = "box";
        robot_desc::URDF::Link::Geometry::Box* box = static_cast<robot_desc::URDF::Link::Geometry::Box*>(link->visual->geometry->shape);
        addKeyValue(geom, "extents", values2str(3,box->size));
        break;
    }
    case robot_desc::URDF::Link::Geometry::CYLINDER: {
        type = "cylinder";
        robot_desc::URDF::Link::Geometry::Cylinder* cylinder = static_cast<robot_desc::URDF::Link::Geometry::Cylinder*>(link->visual->geometry->shape);
        addKeyValue(geom,"radius",values2str(1,&cylinder->radius));
        addKeyValue(geom,"height",values2str(1,&cylinder->length));
        break;
    }
    case robot_desc::URDF::Link::Geometry::SPHERE: {
        type = "sphere";
        double radius = static_cast<robot_desc::URDF::Link::Geometry::Sphere*>(link->visual->geometry->shape)->radius;
        addKeyValue(geom,"radius",values2str(1,&radius));
        break;
    }
    default: {
        fprintf(stderr,"Unknown body type: %d in geometry '%s'\n", link->visual->geometry->type, link->visual->geometry->name.c_str());
        exit(-1);
    }
    }
		
    geom->SetAttribute("type", type);
        
    // compute the visualisation transfrom
    //        libTF::Pose3D coll;
    //        setupTransform(coll, link->collision->xyz, link->collision->rpy);
    //        coll.invert();
		
    libTF::Pose3D vis;
    setupTransform(vis, link->visual->xyz, link->visual->rpy);
    //        coll.multiplyPose(vis);
		
    addTransform(geom, vis);
	           
    copyOpenraveMap(link->visual->data, geom);
    body->LinkEndChild(geom); // end geom

    // mass
    TiXmlElement* mass = new TiXmlElement("mass");
    mass->SetAttribute("type","custom");
    addKeyValue(mass, "total", values2str(1,&link->inertial->mass));
	    
    double* inertia = link->inertial->inertia;
    double Im[9] = {inertia[0],inertia[1],inertia[2],inertia[1],inertia[3],inertia[4],inertia[2],inertia[4],inertia[5]};
    addKeyValue(mass, "inertia", values2str(9, Im));
    addKeyValue(mass, "com", values2str(3, link->inertial->com));
    body->LinkEndChild(mass);
    
    copyOpenraveMap(link->data, body);
    root->LinkEndChild(body);
	
    /* compute the joint tag */
    bool fixed = false;
    string jtype;
    switch (link->joint->type) {
    case robot_desc::URDF::Link::Joint::REVOLUTE:
        jtype = "hinge";
        break;
    case robot_desc::URDF::Link::Joint::PRISMATIC:
        jtype = "slider";
        break;
    case robot_desc::URDF::Link::Joint::FLOATING:
    case robot_desc::URDF::Link::Joint::PLANAR:
        // sometimes used to attach the base
        break;
    case robot_desc::URDF::Link::Joint::FIXED:
        jtype = "hinge";
        fixed = true;
        break;
    default:
        fprintf(stderr, "Unknown joint type: %d in link '%s'\n", link->joint->type, link->name.c_str());
        break;
    }
	
    if (!jtype.empty()) {
        TiXmlElement *joint = new TiXmlElement("joint");
        joint->SetAttribute("type", jtype);
        joint->SetAttribute("name", link->joint->name);
	    
        if( link->joint->pjointMimic != NULL ) {
            stringstream ss;
            ss << link->joint->pjointMimic->name << " " << link->joint->fMimicMult << " " << link->joint->fMimicOffset;
            joint->SetAttribute("mimic",ss.str());
            joint->SetAttribute("enable","false");
        }

        addKeyValue(joint, "body", link->name);
        addKeyValue(joint, "body", link->parentName);
        addKeyValue(joint, "offsetfrom", link->name);
	    
        if (fixed) {
            addKeyValue(joint, "lowStop", "0");
            addKeyValue(joint, "highStop", "0");
            addKeyValue(joint, "axis", "1 0 0");
        }
        else {        
            addKeyValue(joint, "axis", values2str(3, link->joint->axis));
            addKeyValue(joint, "anchor", values2str(3, link->joint->anchor));
		
            if (link->joint->pjointMimic == NULL  && enforce_limits && link->joint->isSet["limit"]) {
                if (jtype == "slider") {
                    addKeyValue(joint, "lostop",  values2str(1, link->joint->limit             ));
                    addKeyValue(joint, "histop", values2str(1, link->joint->limit + 1         ));
                }
                else {
                    addKeyValue(joint, "lostop",  values2str(1, link->joint->limit,rad2deg));
                    addKeyValue(joint, "histop", values2str(1, link->joint->limit + 1,rad2deg));
                }
            }
        }
        
        root->LinkEndChild(joint);
    }

    
    for (unsigned int i = 0 ; i < link->children.size() ; ++i)
        convertLink(root, link->children[i], currentTransform, enforce_limits);
}

void usage(const char *progname)
{
    printf("\nUsage: %s URDF.xml OpenRAVE.robot.xml\n", progname);
    printf("       where URDF.xml is the file containing a robot description in the Willow Garage format (URDF)\n");
    printf("       and OpenRAVE.robot.xml is the file where the OpenRAVE robot model should be written\n\n");
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        usage(argv[0]);
        exit(1);
    }
    
    bool enforce_limits = true;
    robot_desc::URDF wgxml;
    
    if (!wgxml.loadFile(argv[1])) {
        fprintf(stderr,"Unable to load robot model from %s\n", argv[1]);  
        exit(2);
    }
    
    TiXmlDocument doc;
    
    // extract name only
    string robotname = argv[1];
    size_t pos = robotname.find_last_of('/');
    if( pos == string::npos )
        pos = robotname.find_last_of('\\');
    if( pos != string::npos )
        robotname = robotname.substr(pos+1);
    pos = robotname.find_first_of('.');
    if( pos != string::npos )
        robotname = robotname.substr(0,pos);

    string outresdir = argv[2];
    pos = outresdir.find_last_of('/');
    if( pos == string::npos )
        pos = outresdir.find_last_of('\\');
    if( pos != string::npos )
        outresdir = outresdir.substr(0,pos);

    TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);
    
    // create root element and define needed namespaces
    TiXmlElement *robot = new TiXmlElement("Robot");
    // Create a node to enclose the robot body
    cout << "creating robot: " << robotname << endl;
    robot->SetAttribute("name", robotname.c_str());

    TiXmlElement *kinbody = new TiXmlElement("KinBody");
    if( argc >= 4 ) {
        if( outresdir.size() > 0 )
            outresdir += "/";
        outresdir += argv[3];
        addKeyValue(kinbody,"modelsdir",string(argv[3])+string("/")+robotname);
    }

    outresdir += "/" + robotname;

    // set the transform for the whole model to identity
    libTF::Pose3D transform;        
    for (unsigned int k = 0 ; k < wgxml.getDisjointPartCount() ; ++k)
        convertLink(kinbody, wgxml.getDisjointPart(k), transform, enforce_limits);
    
    /* find all data item types */
    copyOpenraveMap(wgxml.getMap(), kinbody);
    robot->LinkEndChild(kinbody);
    doc.LinkEndChild(robot);

    if (!doc.SaveFile(argv[2])) {
        fprintf(stderr,"Unable to save gazebo model in %s\n", argv[2]);  
        exit(3);
    }

    string inresdir = res_locator::resource2path(wgxml.getResourceLocation());
    if( inresdir.size() > 0 )
        inresdir += "/";
    if( outresdir.size() > 0 )
        outresdir += "/";

    int ret = mkdir(outresdir.c_str(), 0777);

    if( ret != 0 && errno != EEXIST ) {
        cerr << "failed to create resource directory " << outresdir << endl;
        exit(-2);
    }

    cout << "copying resource files from " << inresdir << " to " << outresdir << endl;
    for(list<string>::iterator it = s_listResourceNames.begin(); it != s_listResourceNames.end(); ++it) {
        ifstream ifile((inresdir+*it).c_str(), ios_base::binary);
        ofstream ofile((outresdir+*it).c_str(), ios_base::binary);
        ofile << ifile.rdbuf();
    }
        
    return 0;
}
