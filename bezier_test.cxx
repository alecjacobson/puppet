#include "Animation.h"
#include "BezierKeyframe.h"
#include "bezier.h"
#include <igl/matlab_format.h>
#include <Eigen/Core>
#include <string>
#include <iostream>
int main(int argc, char * argv[])
{
  // ./bind_test mesh.obj nodes.xml
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  cout<<"<html><body>"<<endl;
  Animation<BezierKeyframe<double> > cubic,linear;
  for(int p = 0;p<6;p++)
  {
    const double v = (p%2)*200;
    const double t = p*100;
    double prev = v;
    if(!linear.keyframes.empty())
    {
      prev = linear.keyframes.back().second.value;
      linear.keyframes.back().second.next = prev + (v-prev)/3.;
    }
    linear.keyframes.push_back(make_pair(t,BezierKeyframe<double>(prev+(v-prev)*2./3.,v,v)));
    cubic.keyframes.push_back(make_pair(t,BezierKeyframe<double>(v,v,v)));
  }
  const double t_max = cubic.keyframes.back().first;
  cubic.last_interval = 0;
  linear.last_interval = 0;
  for(double t = 0;t<t_max;t+=t_max/100.)
  {
    double cval,lval;
    BezierKeyframe<double>::find(cubic,t,cval,cubic.last_interval);
    BezierKeyframe<double>::find(linear,t,lval,linear.last_interval);
    cout<<"<div style='position:absolute;top:"<<cval<<
      ";left:"<<t<<";background-color:#00F;'>o</div>"<<endl;
    cout<<"<div style='position:absolute;top:"<<lval<<
      ";left:"<<t<<";background-color:#0F0;'>o</div>"<<endl;
  }
  return 0;
}
