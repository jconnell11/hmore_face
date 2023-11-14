// jhcFestTTS.h : simple interface to Linux Festival Text-To-Speech
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2023 Etaoin Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
///////////////////////////////////////////////////////////////////////////

// Multiphase programming example (Linux only):
//
//   jhcFestTTS tts;
//   tts.Start();                      // start server (once)
//   tts.split = 1;                    // two phase mode (once)
//   tts.Say("Hello out there!");      // request conversion
//   while (tts.Poised() == 0)         // await phoneme file
//     sleep(0.01);
//   --lip sync--                      // start mouth movement
//   tts.Emit();                       // start waveform playback
//   while (tts.Talking() > 0)         // await sound finish (if desired)
//     sleep(0.01);

/////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdio.h>
#include <pthread.h>

#include <jhcGenTTS.h>


//= Simple interface to Linux Festival Text-To-Speech package.
// interacts solely through files and Command Line Interface
// requires command line libraries:
//   sudo apt-get install festival soundstretch
// Note: code works for LINUX ONLY!

class jhcFestTTS : public jhcGenTTS
{
// PRIVATE MEMBER VARIABLES
private:
  char ph[10];
  FILE *in;
  pthread_t synth, play;
  int hook, prepping, emitting;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  jhcFestTTS ();
  ~jhcFestTTS ();
  int Start (int vol =0, int dev =1);  

  // main functions
  void Say (const char *txt, int split =0);
  int Poised ();
  const char *Phoneme (float& secs);
  void Emit ();
  int Talking ();
  int Working ();


// PRIVATE MEMBER FUNCTIONS
private:
  // creation and initialization
  void shutdown ();
  void make_prolog ();

  // background thread functions
  static void *generate (void *tts);
  static void *speak (void *dummy);

};


//= Construct instance of derived class if using shared library (or DLL).
// program should call delete on returned object at end 

extern "C" jhcGenTTS *new_jhcTTS ()
{
  return new jhcFestTTS;
}


