#ifndef SOUND_H
#define SOUND_H
#include <OpenAL/al.h>
#include <string>
#include <vector>

class Sound
{
  private:
    ALuint buffer;
    std::vector<ALuint> sources;
  public:
    // Create a new playable sound clip. Can play up to num_sources
    // simultaneously. Otherwise restarts one.
    //
    // Inputs:
    //   num_sources  number of simultaneously playing clips {10}
    //   filename  path to file {""}
    Sound(const int num_sources=10,const std::string & filename="");
    virtual ~Sound();
    // Load a sound clip from a file and initialize sources.
    //
    // Inputs:
    //   filename  path to file {""}
    bool load(const std::string & filename);
    // Clean up
    bool unload();
    // Play from the next available source or pick one and restart it.
    bool play();
};
#endif
