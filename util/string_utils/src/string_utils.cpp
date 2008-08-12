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
