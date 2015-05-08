#include "bezier.h"
#include <cmath>
double bezier(
  const double t,
  const double & A,
  const double & B,
  const double & C,
  const double & D)
{
  using std::pow;
  return 
    1.*pow(t,0)*pow(1-t,3) * A + 
    3.*pow(t,1)*pow(1-t,2) * B + 
    3.*pow(t,2)*pow(1-t,1) * C + 
    1.*pow(t,3)*pow(1-t,0) * D;
};

template <typename T>
T bezier(
  const double t,
  const typename std::vector<T>::const_iterator & qbegin,
  const typename std::vector<T>::const_iterator & qend,
  T (*lerp)(const double, const T &,const T&))
{
  using namespace Eigen;
  using namespace std;
  const auto dist = distance(qbegin,qend);
  if(dist == 1)
  {
    // singleton
    return *qbegin;
  }
  return lerp(t,bezier(t,qbegin,qend-1,lerp),bezier(t,qbegin+1,qend,lerp));
}

template double bezier<double>(double, std::vector<double, std::allocator<double> >::const_iterator const&, std::vector<double, std::allocator<double> >::const_iterator const&, double (*)(double, double const&, double const&));
template Eigen::Quaternion<double, 0> bezier<Eigen::Quaternion<double, 0> >(double, std::vector<Eigen::Quaternion<double, 0>, std::allocator<Eigen::Quaternion<double, 0> > >::const_iterator const&, std::vector<Eigen::Quaternion<double, 0>, std::allocator<Eigen::Quaternion<double, 0> > >::const_iterator const&, Eigen::Quaternion<double, 0> (*)(double, Eigen::Quaternion<double, 0> const&, Eigen::Quaternion<double, 0> const&));
