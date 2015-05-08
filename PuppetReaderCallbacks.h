#ifndef PUPPETREADER_CALLBACKS_H
#define PUPPETREADER_CALLBACKS_H
#include "PuppetReader.h"
#include <AntTweakBar.h>
// Send a identify command to the puppet
//
// Inputs:
//   clientData  pointer to PuppetReader
namespace PuppetReaderCallbacks
{
  void TW_CALL send_identify(void *clientData);
  void TW_CALL send_next_command(void *clientData);
  void TW_CALL toggle_is_partying(void *clientData);
  void TW_CALL toggle_is_no_LEDs(void *clientData);
  void TW_CALL push_wrong_topology_measurement(void *clientData);
  void TW_CALL push_correct_topology_measurement(void *clientData);
  void TW_CALL clear_id_to_last_node(void *clientData);
};
#endif
