#include "filter_prefix.h"
#include <igl/is_dir.h>
#include <igl/is_file.h>
#include <igl/basename.h>
#include <algorithm>
std::string filter_prefix(const std::string _prefix)
{
  using namespace std;
  using namespace igl;
  string prefix = _prefix;
  if(is_dir(prefix.c_str()))
  {
    // strip trailing "/"s
    while((*prefix.rbegin()) == '/')
    {
      prefix = prefix.substr(0, prefix.size()-1);
    }
    // append "basename"
    prefix = prefix + "/" + basename(prefix);
  }else if(is_file(prefix.c_str()))
  {
    // strip trailing "-*"
    prefix = 
      string(prefix.begin(),find(prefix.rbegin(),prefix.rend(),'-').base()-1);
  }
  // strip trailing "-"
  while((*prefix.rbegin()) == '-')
  {
    prefix = prefix.substr(0, prefix.size()-1);
  }
  return prefix;
}

