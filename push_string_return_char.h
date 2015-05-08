#include <string>
// Given a vector/list push a given char as a string and return the char
//
// Inputs:
//   list  list/vector of strings to be pushed with c 
//   c  string
// Outputs
template <typename T>
const char * push_string_return_char(T & list, const char * c);

// Implementation
template <typename T>
const char * push_string_return_char(T & list, const char * c)
{
  list.push_back(std::string(c));
  return c;
}
