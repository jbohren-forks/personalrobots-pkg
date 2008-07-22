#include <cstdio>
#include <cstdlib>
#include <urdf/URDF.h>
#include <vector>

void convert(robot_desc::URDF &wgxml, TiXmlDocument &doc)
{
    TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);
    
    TiXmlElement *root = new TiXmlElement("model:physical");
    root->SetAttribute("xmlns:model", "http://playerstage.sourceforge.net/gazebo/xmlschema/#model");
    root->SetAttribute("xmlns:sensor", "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor");
    root->SetAttribute("xmlns:body", "http://playerstage.sourceforge.net/gazebo/xmlschema/#body");
    root->SetAttribute("xmlns:geom", "http://playerstage.sourceforge.net/gazebo/xmlschema/#geom");
    root->SetAttribute("xmlns:joint", "http://playerstage.sourceforge.net/gazebo/xmlschema/#joint");
    root->SetAttribute("xmlns:controller", "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller");
    root->SetAttribute("xmlns:interface", "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface");
    doc.LinkEndChild(root);
    
    TiXmlElement *xyz      = new TiXmlElement("xyz");
    TiXmlText    *text_xyz = new TiXmlText("0.0 0.0 0.0");
    xyz->LinkEndChild(text_xyz);    
    root->LinkEndChild(xyz);
    TiXmlElement *rpy = new TiXmlElement("rpy");
    TiXmlText    *text_rpy = new TiXmlText("0.0 0.0 0.0");
    rpy->LinkEndChild(text_rpy);    
    root->LinkEndChild(rpy);    

    std::vector<robot_desc::URDF::Link*> links;
    wgxml.getLinks(links);
    
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	TiXmlElement *elem = new TiXmlElement("body:box");
	root->LinkEndChild(elem);	
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
