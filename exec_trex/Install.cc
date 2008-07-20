/**
 * @brief A tiny program to generate an include file for Jam
 */

#include <fstream>
#include <string>
int main(int argc, char **argv)
{
  std::ofstream of("env.jam");
  of << "# This is an auto-generated file built using rospack" << std::endl;

  {
    system("rospack find trex > env.scratch");
    std::ifstream inFile("env.scratch");
    std::string s;
    getline(inFile, s);
    of << "TREX_PKG_ROOT = " << s << " ;" << std::endl << std::endl;
    inFile.close();
  }


  {
    system("rospack export/cpp/cflags exec_trex > env.scratch");
    std::ifstream inFile("env.scratch");
    std::string s;
    getline(inFile, s);
    of << "PKG_CPP_FLAGS = " << s << " ;" << std::endl << std::endl;
    inFile.close();
  }

  {
    system("rospack export/cpp/lflags exec_trex > env.scratch");
    std::ifstream inFile("env.scratch");
    std::string s;
    getline(inFile, s);
    of << "PKG_LINK_FLAGS = " << s << " ;";
    inFile.close();
  }
  
  of.close();
}
