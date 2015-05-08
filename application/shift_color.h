#ifndef SHIFT_COLOR_H
#define SHIFT_COLOR_H
// Shift color according to UI settings
//
// Inputs:
//   in  RGB input color
//   alpha  input alpha
//   is_hover  whether object being colored is hovered
//   is_selected  whether object being colored is selected 
//   is_down  whether object being color is down 
// Output:
//   color  rgb output color
//   eff_alpha  output alpha

void shift_color(
  const float * in,
  const double alpha,
  const bool is_hover,
  const bool is_selected,
  const bool is_down,
  float * color,
  double & eff_alpha);

#endif
