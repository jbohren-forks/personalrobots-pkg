#include <cstdio>
#include <cstdlib>
#include <urdf/URDF.h>

void convert(robot_desc::URDF &wgxml, TiXmlDocument &doc)
{
    
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
