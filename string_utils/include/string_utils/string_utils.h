#ifndef STRING_UTILS_STRING_UTILS_H
#define STRING_UTILS_STRING_UTILS_H

#include <string>
#include <vector>

namespace string_utils
{
void split(const std::string &str, 
           std::vector<std::string> &token_vec, 
           const std::string &delim);
}

#endif

