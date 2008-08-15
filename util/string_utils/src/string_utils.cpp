#include <string_utils/string_utils.h>

void string_utils::split(const std::string &s, std::vector<std::string> &t, const std::string &d)
{
  t.clear();
  std::size_t start = 0, end;
  while ((end = s.find_first_of(d, start)) != std::string::npos)
  {
    t.push_back(s.substr(start, end-start));
    start = end + 1;
  }
  t.push_back(s.substr(start));
}

std::string string_utils::trim(const std::string &str)
{
    std::string res = str;
    
    while (!res.empty() && (res[res.size() - 1] == ' ' || 
			    res[res.size() - 1] == '\t' || 
			    res[res.size() - 1] == '\n' || 
			    res[res.size() - 1] == '\r'))
	res.erase(res.size() - 1); 
    
    while (!res.empty() && (res[0] == ' ' || 
			    res[0] == '\t' || 
			    res[0] == '\n' || 
			    res[0] == '\r'))
	res.erase(0, 1); 
    return res;    
}
