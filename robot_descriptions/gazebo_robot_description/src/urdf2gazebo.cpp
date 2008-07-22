#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <vector>
#include <string>
#include <sstream>

#include <urdf/URDF.h>
#include <libTF/Pose3D.h>

std::string values2str(unsigned int count, const double *values, double (*conv)(double) = NULL)
{
    std::stringstream ss;
    for (unsigned int i = 0 ; i < count ; i++)
    {
	if (i > 0)
	    ss << " ";
	ss << (conv ? conv(values[i]) : values[i]);
    }
    return ss.str();
}

double rad2deg(double v)
{
    return v * 180.0 / M_PI;
}

void addKeyValue(TiXmlElement *elem, const std::string& key, const std::string &value)
{
    TiXmlElement *ekey      = new TiXmlElement(key);
    TiXmlText    *text_ekey = new TiXmlText(value);
    ekey->LinkEndChild(text_ekey);    
    elem->LinkEndChild(ekey); 
}

void convert(robot_desc::URDF &wgxml, TiXmlDocument &doc)
{
    TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);
    
    /* create root element and define needed namespaces */
    TiXmlElement *root = new TiXmlElement("model:physical");
    root->SetAttribute("xmlns:model", "http://playerstage.sourceforge.net/gazebo/xmlschema/#model");
    root->SetAttribute("xmlns:sensor", "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor");
    root->SetAttribute("xmlns:body", "http://playerstage.sourceforge.net/gazebo/xmlschema/#body");
    root->SetAttribute("xmlns:geom", "http://playerstage.sourceforge.net/gazebo/xmlschema/#geom");
    root->SetAttribute("xmlns:joint", "http://playerstage.sourceforge.net/gazebo/xmlschema/#joint");
    root->SetAttribute("xmlns:controller", "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller");
    root->SetAttribute("xmlns:interface", "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface");
    doc.LinkEndChild(root);
    
    /* set the transform for the whole model to identity */
    addKeyValue(root, "xyz", "0 0 0");
    addKeyValue(root, "rpy", "0 0 0");
    
    std::vector<robot_desc::URDF::Link*> links;
    wgxml.getLinks(links);
    
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	std::string type;
	unsigned int nsize = 0;
	switch (links[i]->collision->geometry->type)
	{
	case robot_desc::URDF::Link::Geometry::BOX:
	    nsize = 3;
	    type = "box";
	    break;
	case robot_desc::URDF::Link::Geometry::CYLINDER:
	    nsize = 2;
	    type = "cylinder";
	    break;
	case robot_desc::URDF::Link::Geometry::SPHERE:
	    nsize = 1;
	    type = "sphere";
	    break;
	default:
	    nsize = 0;
	    printf("Unknown body type: %d in link '%s'\n", links[i]->collision->geometry->type, links[i]->name.c_str());
	    break;
	}
	if (!type.empty())
	{
	    /* create new body */
	    TiXmlElement *elem     = new TiXmlElement("body:" + type);

	    /* set body name */
	    elem->SetAttribute("name", links[i]->name);
	    
	    /* set transform */
	    addKeyValue(elem, "xyz", values2str(3, links[i]->xyz));
	    addKeyValue(elem, "rpy", values2str(3, links[i]->rpy, rad2deg));
	    
	    /* create geometry node */
	    TiXmlElement *geom     = new TiXmlElement("geom:" + type);

	    {		
		/* set its name */
		geom->SetAttribute("name", links[i]->collision->geometry->name);

		/* set transform */
		addKeyValue(geom, "xyz", values2str(3, links[i]->collision->xyz));
		addKeyValue(geom, "rpy", values2str(3, links[i]->collision->rpy, rad2deg));

		/* set mass properties */
		addKeyValue(geom, "massMatrix", "true");
		addKeyValue(geom, "mass", values2str(1, &links[i]->inertial->mass));
		static const char tagList1[6][4] = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"};
		for (int j = 0 ; j < 6 ; ++j)
		    addKeyValue(geom, tagList1[j], values2str(1, links[i]->inertial->inertia + j));
		static const char tagList2[3][3] = {"cx", "cy", "cz"};
		for (int j = 0 ; j < 3 ; ++j)
		    addKeyValue(geom, tagList2[j], values2str(1, links[i]->inertial->com + j));

		/* set additional data */
		static const char tagList3[4][4] = {"kp", "kd", "mu1", "mu2"};
		for (int j = 0 ; j < 4 ; ++j)
		    if (links[i]->collision->data.hasDefaultValue(tagList3[j]))
			addKeyValue(geom, tagList3[j], links[i]->collision->data.getDefaultValue(tagList3[j]));
		
		/* set geometry size */
		addKeyValue(geom, "size", values2str(nsize, links[i]->collision->geometry->size));
		
		/* create visual node */
		TiXmlElement *visual = new TiXmlElement("visual");
		{
		    /* compute the visualisation transfrom */
		    libTF::Pose3D coll;
		    const double *pos = links[i]->collision->xyz;
		    const double *rot = links[i]->collision->rpy;
		    coll.setFromEuler(pos[0], pos[1], pos[2], rot[2], rot[1], rot[0]);
		    coll.invert();
		    
		    libTF::Pose3D vis;
		    pos = links[i]->visual->xyz;
		    rot = links[i]->visual->rpy;
		    vis.setFromEuler(pos[0], pos[1], pos[2], rot[2], rot[1], rot[0]);
		    coll.multiplyPose(vis);
		    
		    libTF::Pose3D::Position pz;
		    coll.getPosition(pz);
		    double cpos[3] = { pz.x, pz.y, pz.z };
		    
		    libTF::Pose3D::Euler eu;
		    coll.getEuler(eu);
		    double crot[3] = { eu.roll, eu.pitch, eu.yaw };		    
		    
		    /* set geometry transform */
		    addKeyValue(visual, "xyz", values2str(3, cpos));
		    addKeyValue(visual, "rpy", values2str(3, crot, rad2deg));
		    
		    /* set geometry scale */
		    addKeyValue(visual, "scale", values2str(3, links[i]->visual->scale));

		    /* set geometry mesh file */
		    if (!links[i]->visual->geometry->filename.empty())
			addKeyValue(visual, "mesh", links[i]->visual->geometry->filename);

		    /* set geometry material */		    
		    if (!links[i]->visual->material.empty())
			addKeyValue(visual, "material", links[i]->visual->material);
		}
		
		geom->LinkEndChild(visual);
	    }
	    
	    /* add geometry to body */
	    elem->LinkEndChild(geom);	    
	    
	    /* add body to document */
	    root->LinkEndChild(elem);

	    /* compute the joint tag */
	    std::string jtype;
	    switch (links[i]->joint->type)
	    {
	    case robot_desc::URDF::Link::Joint::REVOLUTE:
		jtype = "hinge";
		break;
	    case robot_desc::URDF::Link::Joint::PRISMATIC:
		jtype = "slider";
		break;
	    default:
		printf("Unknown joint type: %d in link '%s'\n", links[i]->joint->type, links[i]->name.c_str());
		break;
	    }
	    
	    if (!jtype.empty())
	    {
		TiXmlElement *joint = new TiXmlElement("joint:" + jtype);
		
		addKeyValue(joint, "body1", links[i]->parentName);
		addKeyValue(joint, "body2", links[i]->name);
		addKeyValue(joint, "anchor", links[i]->name);
		
		addKeyValue(joint, "axis", values2str(3, links[i]->joint->axis));
		addKeyValue(joint, "axisOffset", values2str(3, links[i]->joint->anchor));
		
		addKeyValue(joint, "lowStop", values2str(1, links[i]->joint->limit));
		addKeyValue(joint, "highStop", values2str(1, links[i]->joint->limit + 1));

		/* add joint to document */
		root->LinkEndChild(joint);
	    }	    
	}	
    }
}

void usage(const char *progname)
{
    printf("\nUsage: %s wgDescription.xml Gazebo.model\n", progname);
    printf("       where wgDescription.xml is the file containing a robot description in the Willow Garage format\n");
    printf("       and Gazebo.model is the file where the Gazebo model should be written\n\n");
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
	usage(argv[0]);
	exit(1);
    }
    
    robot_desc::URDF wgxml;

    if (!wgxml.loadFile(argv[1]))
    {
	printf("Unable to load robot model from %s\n", argv[1]);	
	exit(2);
    }
    
    TiXmlDocument doc;
    
    convert(wgxml, doc);
    
    if (!doc.SaveFile(argv[2]))
    {
	printf("Unable to save gazebo model in %s\n", argv[2]);	
	exit(3);
    }
    
    return 0;
}
