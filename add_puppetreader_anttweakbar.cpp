#include "add_puppetreader_anttweakbar.h"
#include "PuppetReader.h"
#include "PuppetReaderCallbacks.h"
#include <cassert>

void add_puppetreader_anttweakbar(
  PuppetReader * pr,
  igl::ReTwBar & rebar)
{
  using namespace PuppetReaderCallbacks;
  assert(pr);
  rebar.TwAddVarRO("lines_read",TW_TYPE_INT32,&pr->lines_read,
    "group='PuppetReader' "
    "label='lines_read' "
    );
  rebar.TwAddVarRO("total_lines_read",TW_TYPE_INT32,&pr->total_lines_read,
    "group='PuppetReader' "
    "label='total_lines_read' "
    );
  rebar.TwAddVarRO("loop_count",TW_TYPE_INT32,&pr->loop_count,
    "group='PuppetReader' "
    "label='loop_count' "
    );
  rebar.TwAddVarRW(
    "print_lines",
    TW_TYPE_BOOLCPP,
    &pr->print_lines,
    "group='PuppetReader' "
    "label='print_lines' "
    "help='Print lines as they are parsed' ");
  rebar.TwAddVarRW(
    "log_lines",
    TW_TYPE_BOOLCPP,
    &pr->log_lines,
    "group='PuppetReader' "
    "label='log_lines' "
    "help='log lines as they are parsed' ");
  rebar.TwAddVarRW(
    "puppet_command",
    TW_TYPE_STDSTRING,
    &pr->next_command,
    "group='PuppetReader' "
    "label='command' "
    "help='Command to send to puppet device, will automatically "
    " suffixed by \"\\r\\n\"' ",false);
  rebar.TwAddButton("send_command",&send_next_command,pr,
    "group='PuppetReader' "
    "label='send' "
    "help='send puppet command above' ");
  rebar.TwAddButton("identify",&send_identify,pr,
    "group='PuppetReader' "
    "label='identify' "
    "help='Send identify command to puppet serial' "
    "key=i ");
  rebar.TwAddButton("push_wrong_topo",&push_wrong_topology_measurement,pr,
    "group='PuppetReader' "
    "label='log wrong topology' "
    "key='W' "
    );
  rebar.TwAddButton("push_correct_topo",&push_correct_topology_measurement,pr,
    "group='PuppetReader' "
    "label='log right topology' "
    "key='R' "
    );
  rebar.TwAddButton("clear_id_to_last_node",&PuppetReaderCallbacks::clear_id_to_last_node,pr,
    "group='PuppetReader' "
    "label='clear memory' "
    );
  rebar.TwAddButton("party",&PuppetReaderCallbacks::toggle_is_partying,pr,
    "group='PuppetReader' "
    "label='party' "
    "key='!' "
    "help='Dance like a millionaire on the night before the "
    " world ends and you just dont care' "
    );
  rebar.TwAddButton("no_leds",&PuppetReaderCallbacks::toggle_is_no_LEDs,pr,
    "group='PuppetReader' "
    "label='lights off' "
    "key='~' "
    );
}
