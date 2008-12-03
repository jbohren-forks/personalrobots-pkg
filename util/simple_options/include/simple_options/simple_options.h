#ifndef SIMPLE_OPTIONS_SIMPLE_OPTIONS_H
#define SIMPLE_OPTIONS_SIMPLE_OPTIONS_H

#include <string>
#include <cstring>
#include <map>

namespace simple_options
{
std::map<std::string, std::string> parse(int argc, char **argv, int start = 0);
}

#endif

