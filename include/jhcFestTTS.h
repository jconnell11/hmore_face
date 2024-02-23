// jhcFestTTS.h : simple interface to Linux Festival Text-To-Speech
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2023-2024 Etaoin Systems
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
//   tts.Prep("Hello out there!");     // request conversion
//   while (tts.Poised() == 0)         // await phoneme file
//     sleep(0.01);
//   --lip sync--                      // start mouth movement
//   tts.Emit();                       // start waveform playback
//   while (tts.Talking() > 0)         // await sound finish (if desired)
//     sleep(0.01);
//   tts.Done();

/////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdio.h>
#include <pthread.h>
#include <time.h>


//= Simple interface to Linux Festival Text-To-Speech package.
// interacts solely through files and Command Line Interface
// requires command line libraries:
//   sudo apt-get install festival-dev soundstretch
// Note: code works for LINUX ONLY!

class jhcFestTTS
{
// PRIVATE MEMBER VARIABLES
private:
  char ph[10];
  FILE *in;
  pthread_t synth, play;
  int prepping, emitting;


// PUBLIC MEMBER VARIABLES
public:
  int freq;        // voice pitch
  int infl;        // pitch variation
  int shift;       // formant raising
  int slow;        // stretch phrase


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcFestTTS ();
  jhcFestTTS ();
  int Start (int vol =0);  

  // main functions
  void Prep (const char *txt);
  int Poised ();
  const char *Phoneme (float& secs);
  void Emit ();
  int Talking ();
  void Done ();


// PRIVATE MEMBER FUNCTIONS
private:
  // creation and initialization
  void shutdown ();
  void make_prolog ();

  // background thread functions
  void kill_prep ();
  static void *generate (void *tts);
  void kill_emit ();
  static void *speak (void *dummy);

};



