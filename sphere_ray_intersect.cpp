#include "sphere_ray_intersect.h"

int sphere_ray_intersect(
  const Eigen::Vector3d & ac,
  const double r,
  const Eigen::Vector3d & ao,
  const Eigen::Vector3d & d,
  double & t0,
  double & t1)
{
  Eigen::Vector3d o = ao-ac;
  // http://wiki.cgsociety.org/index.php/Ray_Sphere_Intersection
  //Compute A, B and C coefficients
  double a = d.dot(d);
  double b = 2 * d.dot(o);
  double c = o.dot(o) - (r * r);

  //Find discriminant
  double disc = b * b - 4 * a * c;
    
  // if discriminant is negative there are no real roots, so return 
  // false as ray misses sphere
  if (disc < 0)
  {
    return 0;
  }

  // compute q as described above
  double distSqrt = sqrt(disc);
  double q;
  if (b < 0)
  {
    q = (-b - distSqrt)/2.0;
  } else
  {
    q = (-b + distSqrt)/2.0;
  }

  // compute t0 and t1
  t0 = q / a;
  double _t1 = c/q;
  if(_t1 == t0)
  {
    return 1;
  }
  t1 = _t1;
  // make sure t0 is smaller than t1
  if (t0 > t1)
  {
    // if t0 is bigger than t1 swap them around
    double temp = t0;
    t0 = t1;
    t1 = temp;
  }
  return 2;
}
