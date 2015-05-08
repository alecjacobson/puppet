#include <string>
#include <list>


// A glorified list of timestamp and Items
template <typename Item>
class Log 
{
  private:
    typedef std::pair<double,Item> TimeAndItem;
    std::list<TimeAndItem > items;
    double start_time;
  public:
    inline Log();
    // Write log to file
    // 
    // Inputs:
    //   filename   path to .log file
    inline bool save(const std::string & filename);
    // Push a new loggable item onto the log
    //
    // Inputs:
    //   item  item to be pushed
    // Returns *this
    //
    inline Log & push(const Item & message);
    inline Log & operator<<(const Item & message);
};

// Implementation
#include <igl/get_seconds.h>
#include <iostream>
#include <fstream>

template <typename Item>
inline Log<Item>::Log():
  start_time(igl::get_seconds()),
  items()
{
}

template <typename Item>
inline Log<Item> & Log<Item>::push(const Item & item)
{
  items.push_back(TimeAndItem(igl::get_seconds()-start_time,item));
  return *this;
}

template <typename Item>
inline Log<Item> & Log<Item>::operator<<(const Item & item)
{
  return push(item);
}

template <typename Item>
inline bool Log<Item>::save(const std::string & filename)
{
  using namespace std;
  ofstream f;
  f.open(filename);
  f.precision(15);
  for(auto & pair : items)
  {
    f<<pair.first<<","<<pair.second<<endl;
  }
  return true;
}
