#ifndef STL_UTILS_H
#define STL_UTILS_H

#include <string>
#include <vector>

using namespace std;

template <class C>
void deleteElements(C *c)
{
  typename C::iterator it;
  for (it = c->begin(); it != c->end(); ++it)
  {
    if (*it != NULL)
    {
      delete (*it);
      *it = NULL;
    }
  }
}

#endif

