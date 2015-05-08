#include <algorithm>

void shift_color(
  const float * in,
  const double alpha,
  const bool is_hover,
  const bool is_selected,
  const bool /*is_down*/,
  float * color,
  double & eff_alpha)
{
  using namespace std;
  eff_alpha = alpha;
  if(is_hover || is_selected)
  {
    eff_alpha = (1.0+eff_alpha)*0.5;
  }
  copy(in,in+3,color);
  if(is_selected)
  {
    color[0] *= 0.65; color[1] *= 0.65; color[2] *= 0.65;
  }else if(is_hover)
  {
    color[0] *= 0.8; color[1] *= 0.8; color[2] *= 0.8;
  }
}

