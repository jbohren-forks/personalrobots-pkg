#include "simple_options/simple_options.h"
using namespace std;

map<string, string> simple_options::parse(int argc, char **argv, int start)
{
  map<string, string> opts;
  for (int i = start; i < argc; i++)
  {
    char *eq;
    if (!(eq = strchr(argv[i], '=')))
      continue;
    string key(argv[i], (eq - argv[i]));
    string value(eq + 1);
    opts[key] = value;
  }
  return opts;
}

