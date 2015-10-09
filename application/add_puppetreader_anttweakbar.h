#ifndef ADD_PUPPETREADER_ANTTWEAKBAR_H
#define ADD_PUPPETREADER_ANTTWEAKBAR_H
#include <igl/anttweakbar/ReAntTweakBar.h>
class PuppetReader;
// Add buttons and switches for variables corresponding to PuppetReader. Adds
// to group='PuppetReader' and uses some default key shortcuts, labels and
// names.
//
// Inputs:
//   pr  Pointer to PuppetReader object
//   rebar  Reference to ReAntTweakBar object (already initialized)
//
// 'W'  Log wrong topology
// 'R'  Log right topology
// '!'  Party mode (verify lights)
// 'i'  Send identify command
void add_puppetreader_anttweakbar(
  PuppetReader * pr,
  igl::anttweakbar::ReTwBar & rebar);
#endif
