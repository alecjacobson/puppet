#ifndef NODECALLBACKS_H
#define NODECALLBACKS_H

#include "Node.h"
#include <AntTweakBar.h>

namespace NodeCallbacks 
{
  // Delete this node and all children
  void TW_CALL deleteCB(void *clientData);
  // Snap r_off to canonical
  void TW_CALL snapCB(void *clientData);
  // Set all angles to 0
  void TW_CALL zeroCB(void *clientData);
  // Tare to current angles
  void TW_CALL tareCB(void *clientData);
  // Delete this child and create new node
  void TW_CALL new_childCB(void *clientData);
  // Reset this node's offsets
  void TW_CALL reset_offsetsCB(void *clientData);
};

#endif
