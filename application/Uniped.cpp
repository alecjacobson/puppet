#include "Uniped.h"
#include "rigid2d.h"
#include <igl/draw_mesh.h>
#include <igl/get_seconds.h>
#include <igl/STR.h>
#include "GLUT_include.h"

Uniped::Uniped():
    V(),U(),UV(),C(),color(),F(),G(),BE(),theta(),offsets(0,0,0,0),
    control_type(CONTROL_TYPE_MOUSE),tex(0),pos(0,0,0),
    prev_theta(10,10,10)
{
}

void Uniped::update()
{
  using namespace Eigen;
  using namespace std;
  using namespace igl;
  const double t_abs = get_seconds(); 
  bool still_playing = false;
  for(int th = 0;th<3;th++)
  {
    auto & anim  = theta_animation[th];
    if(anim.is_playing)
    {
      // playing trumps recording
      anim.is_recording = false;

      const auto & keyframes = anim.keyframes;
      if(keyframes.size() == 0)
      {
        continue;
      }
      still_playing |= 
        BezierKeyframe<double>::find(anim,t_abs,theta(th),anim.last_interval);
    }else if(anim.is_recording)
    {
      const double t = t_abs - anim.t_start;
      auto & keyframes = anim.keyframes;
      const double t_min = 0.3;
      // Skip if identical or if we haven't taken a sample in a while
      if(keyframes.size() == 0 || 
        fabs(keyframes.back().second.value - theta(th))>5e-3 ||
        (t-keyframes.back().first) > t_min)
      {
        const double v = theta(th);
        double prev = v;
        if(!anim.keyframes.empty())
        {
          prev = anim.keyframes.back().second.value;
          anim.keyframes.back().second.next = prev + (v-prev)/3.;
        }
        const BezierKeyframe<double> key(prev+(v-prev)*2./3.,v,v);
        anim.keyframes.push_back(make_pair(t,key));
      }
    }
  }
  // Only stop if all done.
  for(auto & anim : theta_animation)
  {
    if(!still_playing && anim.is_playing)
    {
      anim.is_playing = false;
    }
  }

  int n = V.rows();
  Vector3d th = theta;
  th += offsets.tail(3);
  vector<Affine3d,aligned_allocator<Affine3d> > R(6);
  R[UNIPED_BODY] = rigid2d(C.row(BE(0,0)),offsets(0));
  R[UNIPED_UPPER_LEG] = rigid2d(C.row(BE(1,0)),th(0));
  R[UNIPED_KNEE] = rigid2d(C.row(BE(2,0)),th(1)*0.5);
  R[UNIPED_LOWER_LEG] = rigid2d(C.row(BE(2,0)),th(1)*0.5);
  R[UNIPED_ANKLE] = rigid2d(C.row(BE(3,0)),th(2)*0.5);
  R[UNIPED_FOOT] = rigid2d(C.row(BE(3,0)),th(2)*0.5);
  for(int v = 0;v<n;v++)
  {
    Affine3d t = Affine3d::Identity();
    for(int r = 0; r<=G(v);r++)
    {
        t = t * R[r];
    }
    U.row(v) = t * Vector3d(V.row(v));
  }

}

void Uniped::draw()
{
  using namespace igl;
  using namespace Eigen;
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);
  glBindTexture(GL_TEXTURE_2D,tex);
  glPushMatrix();
  glTranslated(pos(0),pos(1),pos(2));
  MatrixXd n;
  draw_mesh(U,F,n,color,UV);
  glBindTexture(GL_TEXTURE_2D,0);
  glDisable(GL_TEXTURE_2D);
  glPopMatrix();
}

void Uniped::draw_progress_bar() const
{
  using namespace std;
  const double bar_w = 400;
  const double bar_h = 30;
  glDisable(GL_LIGHTING);
  using namespace igl;
  glColor3d(0.2,0.2,0.2);
  glPushMatrix();
  glTranslated(pos(0),pos(1),pos(2));
  glTranslated(0,V.col(1).maxCoeff()+3.*1.5*bar_h,0);
  // bottom left corner
  const float font_size = 0.2f;
  //glTranslatef(0,(119.05-33.33)/2,0);
  glLineWidth(1);
  enum Align
  {
    LEFT = 0,
    CENTER = 1,
    RIGHT = 2,
  };
  auto draw_string = [](const string & str, const Align align)
  {
    auto text_width = [](const string & str) -> double
    {
      double sum = 0;
      for(auto & p : str)
      {
        sum += glutStrokeWidth(GLUT_STROKE_ROMAN, p);
      }
      return sum;
    };
    const double w = text_width(str);
    glPushMatrix();
    switch(align)
    {
      case CENTER:
        glTranslated(-0.5*w,0,0);
        break;
      case RIGHT:
        glTranslated(-w,0,0);
      case LEFT:
      default:
        //glTranslated(0,0,0);
        break;
    }
    for_each(
      str.begin(),
      str.end(),
      bind1st(ptr_fun(&glutStrokeCharacter),GLUT_STROKE_ROMAN));
    glPopMatrix();
  };

  for(int th = 0;th<3;th++)
  {
    auto & anim = theta_animation[th];
    glPushMatrix();
    glPushMatrix();
    glScalef(font_size,font_size,font_size);
    glTranslatef(0,-119.05+bar_h/font_size,0);
    draw_string(STR((th+1)<<": "),CENTER);
    glPopMatrix();
    if(!anim.keyframes.empty())
    {
      double t_rel = 0;
      const double t_end = anim.keyframes.back().first;
      if(anim.is_playing)
      {
        t_rel = get_seconds() - anim.t_start;
      }
      const double f = t_rel/t_end;

      glColor3d(1,0.3,0.25);
      glBegin(GL_QUADS);
      glVertex2d(0,0);
      glVertex2d(0,bar_h);
      glVertex2d(f*bar_w,bar_h);
      glVertex2d(f*bar_w,0);
      glEnd();
      glColor3d(0.2,0.2,0.2);
      glBegin(GL_QUADS);
      glVertex2d(0,0);
      glVertex2d(0,bar_h);
      glVertex2d(bar_w,bar_h);
      glVertex2d(bar_w,0);
      glEnd();
    }
    glPopMatrix();

    // Move down
    //glTranslatef(0,-1.5*(119.05-33.33)*font_size,0);
    glTranslatef(0,-1.5*bar_h,0);
  }

  glPopMatrix();

}
