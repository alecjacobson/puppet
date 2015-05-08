#include "PuppetReaderCallbacks.h"
#include "PupSkin.h"

#include <igl/REDRUM.h>

#include <fstream>

void TW_CALL PuppetReaderCallbacks::send_identify(void *clientData)
{
  PuppetReader * pr = static_cast<PuppetReader*>(clientData);
  if(pr != NULL && pr->is_attached())
  {
    pr->send("i\r\n");
  }
}

void TW_CALL PuppetReaderCallbacks::send_next_command(void *clientData)
{
  PuppetReader * pr = static_cast<PuppetReader*>(clientData);
  if(pr != NULL && pr->is_attached())
  {
    pr->send(pr->next_command + "\r\n");
  }
}

void TW_CALL PuppetReaderCallbacks::toggle_is_partying(void *clientData)
{
  using namespace std;
  PuppetReader * pr = static_cast<PuppetReader*>(clientData);
  if(pr != NULL && pr->is_attached())
  {
    pr->toggle_is_partying();
  }
}

void TW_CALL PuppetReaderCallbacks::toggle_is_no_LEDs(void *clientData)
{
  using namespace std;
  PuppetReader * pr = static_cast<PuppetReader*>(clientData);
  if(pr != NULL && pr->is_attached())
  {
    pr->toggle_is_no_LEDs();
  }
}

void TW_CALL PuppetReaderCallbacks::push_wrong_topology_measurement(
  void *clientData)
{
  using namespace std;
  PuppetReader * pr = static_cast<PuppetReader*>(clientData);
  if(pr != NULL && pr->is_attached())
  {
    pr->wrong_topology_measurement = 
      pr->get_pp().get_last_topology_measurement_text();
  }
}

static void print_topology_measurement(
  const std::vector<std::string > & measurement,
  std::ofstream & outfile)
{
  using namespace std;
  for(
    vector<string>::const_iterator lit = measurement.begin();
    lit != measurement.end();
    lit++)
  {
    outfile<<"    "<<*lit<<endl;
  }
}

#define WRONG_TOPOLOGY_LOG_FILE "wrong-topology-log.txt"
void TW_CALL PuppetReaderCallbacks::push_correct_topology_measurement(
  void *clientData)
{
  using namespace std;
  PuppetReader * pr = static_cast<PuppetReader*>(clientData);
  if( pr->wrong_topology_measurement.size() == 0 )
  {
    cout<<REDRUM("Push a wrong topology measurement first.")<<endl;
    return;
  }
  if(pr != NULL && pr->is_attached())
  {
    pr->correct_topology_measurement = 
      pr->get_pp().get_last_topology_measurement_text();
    if(
      pr->wrong_topology_measurement.size() == 
        pr->correct_topology_measurement.size() &&
      equal(
        pr->wrong_topology_measurement.begin(),
        pr->wrong_topology_measurement.end(),
        pr->correct_topology_measurement.begin()))
    {
      cout<<REDRUM("Wrong and correct topology measurements are equal."
        " Not logging...")<<endl;
      return;
    }
    // Append to file
    ofstream outfile;
    outfile.open(WRONG_TOPOLOGY_LOG_FILE, ios_base::app);
    time_t t = time(0);
    outfile<<asctime(localtime(&t));
    outfile<<"Wrong:"<<endl;
    print_topology_measurement(pr->wrong_topology_measurement,outfile);
    outfile<<"Correct:"<<endl;
    print_topology_measurement(pr->correct_topology_measurement,outfile);
    outfile<<endl;
    cout<<GREENGIN("Wrong/correct topology measurements logged to '"<<
      WRONG_TOPOLOGY_LOG_FILE<<"'.")<<endl;
    // Clear for next one
    pr->wrong_topology_measurement.clear();
  }
}

void TW_CALL PuppetReaderCallbacks::clear_id_to_last_node( void *clientData)
{
  PuppetReader * pr = static_cast<PuppetReader*>(clientData);
  pr->clear_id_to_last_node();
}
