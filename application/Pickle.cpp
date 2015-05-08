#include "Pickle.h"
#include "unique_name.h"
#include "diffuse_material.h"
#include <cmath>
#include <iostream>

bool Pickle::is_picking = false;

Pickle::Pickle()
{
  using namespace std;
  unique_color(pick_color);
}

Pickle::Pickle(const Pickle & /*that*/)
{
  using namespace std;
  unique_color(pick_color);
}

const float* Pickle::get_pick_color() const
{
  return pick_color;
}

void Pickle::activate_pickle() const
{
  using namespace std;
  //// This is designed so that if you call glColor*
  //glDisable(GL_BLEND);
  //glDisable(GL_COLOR_MATERIAL);
  //glEnable(GL_LIGHTING);
  //glEnable(GL_LIGHT0);
  //float ones[4] = {1.0,1.0,1.0,1.0};
  //float zeros[4] = {0.0,0.0,0.0,0.0};
  //glLightfv(GL_LIGHT0,GL_AMBIENT,ones);
  //glLightfv(GL_LIGHT0,GL_DIFFUSE,zeros);
  //glLightfv(GL_LIGHT0,GL_SPECULAR,zeros);
  //glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, half);
  //glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, zeros);
  //glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,zeros);
  glEnable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);
  glColor3fv(pick_color);
}

void Pickle::activate_pickle_or_diffuse_material(
  const float * diffuse, 
  const float alpha) const
{
  if(Pickle::is_picking)
  {
    activate_pickle();
  }else
  {
    diffuse_material(diffuse,alpha);
  }
}

float Pickle::diff(const float * that)
{
  using namespace std;
  float d = sqrt(
    (pick_color[0]-that[0])*(pick_color[0]-that[0])+
    (pick_color[1]-that[1])*(pick_color[1]-that[1])+
    (pick_color[2]-that[2])*(pick_color[2]-that[2]));
  //Alpha value cannot be trusted
  //cout<<
  //  "("<<
  //  pick_color[0]<<" "<<
  //  pick_color[1]<<" "<<
  //  pick_color[2]<<") - ("<<
  //  that[0]<<" "<<
  //  that[1]<<" "<<
  //  that[2]<<") = "<<d<<endl;
  return d;
}
