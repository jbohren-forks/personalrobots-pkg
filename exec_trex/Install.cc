/**
 * @brief A tiny program to generate an include file for Jam
 */

#include <fstream>
#include <string>
int main(int argc, char **argv)
{
  system("rospack find trex > env.scratch");
  std::ifstream inFile("env.scratch");
  std::string s;
  getline(inFile, s);
  std::ofstream of("env.jam");
  of << "TREX_PKG_ROOT = " << s << " ;";
}
