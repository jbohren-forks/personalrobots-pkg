#ifndef STRING_UTILS_STRING_UTILS_H
#define STRING_UTILS_STRING_UTILS_H

#include <string>
#include <vector>
#include <sstream>

namespace string_utils
{
    void split(const std::string &str, 
	       std::vector<std::string> &token_vec, 
	       const std::string &delim);
    
    std::string trim(const std::string &str);
    
    template<typename T>
    static inline std::string convert2str(const T &value)
    {
	std::stringstream ss;
	ss << value;
	return ss.str();
    }
}

#endif
