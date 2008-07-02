#include "urdf/URDF.h"

int main(int argc, char **argv)
{
    if (argc >= 2)
    {
	URDF file(argv[1]);
    }
    
    return 0;    
}
