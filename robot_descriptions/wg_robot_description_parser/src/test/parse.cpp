#include "urdf/URDF.h"

int main(int argc, char **argv)
{
    if (argc >= 2)
    {
        robot_desc::URDF file(argv[1]);
        if (argc >= 3)
            file.print();
    }
    
    return 0;    
}
