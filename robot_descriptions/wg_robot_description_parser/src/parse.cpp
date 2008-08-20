#include <urdf/URDF.h>
#include <cstdio>

int main(int argc, char **argv)
{
    if (argc >= 2)
    {
        robot_desc::URDF file(argv[1]);
        if (argc >= 3)
            file.print();
	file.sanityCheck();
	printf("%u errors\n", file.getErrorCount());
    }
    
    return 0;    
}
