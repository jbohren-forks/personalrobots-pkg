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
    TiXmlElement *xyz      = new TiXmlElement("xyz");
    TiXmlText    *text_xyz = new TiXmlText("0 0 0");
    xyz->LinkEndChild(text_xyz);    
    root->LinkEndChild(xyz);
    TiXmlElement *rpy = new TiXmlElement("rpy");
    TiXmlText    *text_rpy = new TiXmlText("0 0 0");
    rpy->LinkEndChild(text_rpy);
    root->LinkEndChild(rpy);    

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
	    
	    /* set translation */
	    TiXmlElement *xyz      = new TiXmlElement("xyz");
	    TiXmlText    *text_xyz = new TiXmlText(values2str(3, links[i]->xyz));
	    xyz->LinkEndChild(text_xyz);    
	    elem->LinkEndChild(xyz);

	    /* set rotation */
	    TiXmlElement *rpy      = new TiXmlElement("rpy");
	    TiXmlText    *text_rpy = new TiXmlText(values2str(3, links[i]->rpy, rad2deg));
	    rpy->LinkEndChild(text_rpy);    
	    elem->LinkEndChild(rpy);	    
	    
	    /* create geometry node */
	    TiXmlElement *geom     = new TiXmlElement("geom:" + type);

	    {		
		/* set its name */
		geom->SetAttribute("name", links[i]->collision->geometry->name);
		
		/* set geometry translation */
		TiXmlElement *xyz      = new TiXmlElement("xyz");
		TiXmlText    *text_xyz = new TiXmlText(values2str(3, links[i]->collision->xyz));
		xyz->LinkEndChild(text_xyz);
		geom->LinkEndChild(xyz);
		
		/* set geometry rotation */
		TiXmlElement *rpy      = new TiXmlElement("rpy");
		TiXmlText    *text_rpy = new TiXmlText(values2str(3, links[i]->collision->rpy, rad2deg));
		rpy->LinkEndChild(text_rpy);    
		geom->LinkEndChild(rpy);

		/* set mass properties */
		TiXmlElement *massMatrix      = new TiXmlElement("massMatrix");
		TiXmlText    *text_massMatrix = new TiXmlText("true");
		massMatrix->LinkEndChild(text_massMatrix);
		geom->LinkEndChild(massMatrix);
		TiXmlElement *mass            = new TiXmlElement("mass");
		TiXmlText    *text_mass       = new TiXmlText(values2str(1, &links[i]->inertial->mass));
		mass->LinkEndChild(text_mass);
		geom->LinkEndChild(mass);
 		static const char tagList1[6][4] = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"};
		for (int j = 0 ; j < 6 ; ++j)
		{
		    TiXmlElement *ie      = new TiXmlElement(tagList1[j]);
		    TiXmlText    *text_ie = new TiXmlText(values2str(1, links[i]->inertial->inertia + j));
		    ie->LinkEndChild(text_ie);
		    geom->LinkEndChild(ie); 
		}
		static const char tagList2[3][3] = {"cx", "cy", "cz"};
		for (int j = 0 ; j < 3 ; ++j)
		{
		    TiXmlElement *ie      = new TiXmlElement(tagList2[j]);
		    TiXmlText    *text_ie = new TiXmlText(values2str(1, links[i]->inertial->com + j));
		    ie->LinkEndChild(text_ie);
		    geom->LinkEndChild(ie); 
		}
		
		/* set additional data */
		static const char tagList3[4][4] = {"kp", "kd", "mu1", "mu2"};
		for (int j = 0 ; j < 4 ; ++j)
		    if (links[i]->collision->data.hasDefaultValue(tagList3[j]))
		    {
			TiXmlElement *ie      = new TiXmlElement(tagList3[j]);
			TiXmlText    *text_ie = new TiXmlText(links[i]->collision->data.getDefaultValue(tagList3[j]));
			ie->LinkEndChild(text_ie);
			geom->LinkEndChild(ie);
		    }
		
		/* set geometry size */
		TiXmlElement *size      = new TiXmlElement("size");
		TiXmlText    *text_size = new TiXmlText(values2str(nsize, links[i]->collision->geometry->size));
		size->LinkEndChild(text_size);
		geom->LinkEndChild(size);
		
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
		    
		    /* set geometry translation */
		    TiXmlElement *xyz      = new TiXmlElement("xyz");
		    TiXmlText    *text_xyz = new TiXmlText(values2str(3, cpos));
		    xyz->LinkEndChild(text_xyz);
		    visual->LinkEndChild(xyz);
		    
		    /* set geometry rotation */
		    TiXmlElement *rpy      = new TiXmlElement("rpy");
		    TiXmlText    *text_rpy = new TiXmlText(values2str(3, crot, rad2deg));
		    rpy->LinkEndChild(text_rpy);    
		    visual->LinkEndChild(rpy); 

		    /* set geometry scale */
		    TiXmlElement *scale      = new TiXmlElement("scale");
		    TiXmlText    *text_scale = new TiXmlText(values2str(3, links[i]->visual->scale));
		    scale->LinkEndChild(text_scale);    
		    visual->LinkEndChild(scale);
		    
		    if (!links[i]->visual->geometry->filename.empty())
		    {
			/* set geometry mesh file */
			TiXmlElement *mesh      = new TiXmlElement("mesh");
			TiXmlText    *text_mesh = new TiXmlText(links[i]->visual->geometry->filename);
			mesh->LinkEndChild(text_mesh);
			visual->LinkEndChild(mesh);
		    }
		    
		    if (!links[i]->visual->material.empty())
		    {
			/* set geometry material */
			TiXmlElement *material      = new TiXmlElement("material");
			TiXmlText    *text_material = new TiXmlText(links[i]->visual->material);
			material->LinkEndChild(text_material);
			visual->LinkEndChild(material);
		    }
		}
		
		geom->LinkEndChild(visual);
	    }
	    
	    /* add geometry to body */
	    elem->LinkEndChild(geom);	    
	    
	    /* add body to document */
	    root->LinkEndChild(elem);

	    /* compute the joint tag */
	    TiXmlElement *joint = new TiXmlElement("joint");
	    
	    /* add joint to document */
	    root->LinkEndChild(joint);	    
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
