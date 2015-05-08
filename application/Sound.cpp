#include "Sound.h"
#include <OpenAL/alc.h>
#include <AL/alut.h>
#include <iostream>

Sound::Sound(const int num_sources,const std::string & filename):
  buffer(0),
  sources(num_sources,0)
{
  if(filename!="")
  {
    load(filename);
  }
}

Sound::~Sound()
{
  unload();
}

bool Sound::load(const std::string & filename)
{
  using namespace std;
  unload();
  /* Create an AL buffer from the given sound file. */
  buffer = alutCreateBufferFromFile (filename.c_str());
  if(buffer == AL_NONE)
  {
    ALint error = alutGetError();
    cerr<<"Error loading file: '"<<alutGetErrorString(error)<<"'"<<endl;
    return false;
  }

  alGenSources(sources.size(), &sources[0]);

  for(auto & source : sources)
  {
    alSourcei(source, AL_BUFFER, buffer);
    {
      ALint error = alutGetError();
      if(error != ALUT_ERROR_NO_ERROR)
      {
        ALint error = alutGetError();
        cerr<<"Error creating source: '"<<alutGetErrorString(error)<<"'"<<endl;
        return false;
      }
    }
  }
  return true;
}

bool Sound::unload()
{
  using namespace std;

  for(auto & source : sources)
  {
    if(source != 0)
    {
      alDeleteSources(1, &source);
      source = 0;
      {
        ALint error = alutGetError();
        if(error != ALUT_ERROR_NO_ERROR)
        {
          ALint error = alutGetError();
          cerr<<"Error deleting source: '"<<alutGetErrorString(error)<<"'"<<endl;
          return false;
        }
      }
      {
        ALint error = alGetError();
        if(error != AL_NO_ERROR)
        {
          cerr<<"alError deleting source: '"<<alGetString(error)<<"'"<<endl;
        }
      }
    }
  }

  if(buffer != 0)
  {
    alDeleteBuffers(1, &buffer);
    buffer = 0;
    {
      ALint error = alutGetError();
      if(error != ALUT_ERROR_NO_ERROR)
      {
        ALint error = alutGetError();
        cerr<<"Error deleting buffer: '"<<alutGetErrorString(error)<<"'"<<endl;
        return false;
      }
    }
    {
      ALint error = alGetError();
      if(error != AL_NO_ERROR)
      {
        cerr<<"alError deleting buffer: '"<<alGetString(error)<<"'"<<endl;
      }
    }
  }
  return true;
}

bool Sound::play()
{
  using namespace std;
  ALuint next = sources[0];
  for(auto & source : sources)
  {
    ALint status;
    alGetSourcei(source, AL_SOURCE_STATE, &status);
    if(status != AL_PLAYING)
    {
      next = source;
      break;
    }
  }
  // No matter what play something (should be last played...)
  alSourcePlay(next);
  {
    ALint error = alutGetError();
    if(error != ALUT_ERROR_NO_ERROR)
    {
      ALint error = alutGetError();
      cerr<<"Error playing: '"<<alutGetErrorString(error)<<"'"<<endl;
      return false;
    }
  }
  return true;
}


