// jhcFestTTS.h : simple interface to Linux Festival Text-To-Speech
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2023-2025 Etaoin Systems
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

#include <pthread.h>
#include <time.h>


//= Simple interface to Linux Festival Text-To-Speech package.
// interacts solely through files and Command Line Interface
// requires command line libraries:
//   sudo apt-get install festival-dev soundstretch
// Note: code requires Festival so works for LINUX ONLY!

class jhcFestTTS
{
// PRIVATE MEMBER VARIABLES
private:
  static const int pmax = 200;         /* Max phonemes per utterance. */

  // background threads
  pthread_t synth, play;
  int ok, prepping, emitting;

  // mouth shape
  timespec t0;

  // prosody
  float fmult, imult, rmult;  
  

// PUBLIC MEMBER VARIABLES
public:
  int freq;        // base voice pitch
  int infl;        // base pitch variation
  int slow;        // base phrase stretch
  int shift;       // formant raising
  int drama;       // emotional modulation
  int loud;        // audio volume

  // phoneme sequence
  char ph[pmax][10];   
  float off[pmax];
  int np;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcFestTTS ();
  jhcFestTTS ();
  bool Active () const {return(ok > 0);}  

  // modulation adjustment
  void Mood (int bits =0x0000);
  void Emotion (int feel =5, int very =0);
  void Prosody (int fpc, int ipc, int rpc);

  // main functions
  int Start (const char *dir =NULL);
  void Prep (const char *txt);
  int Poised ();
  void Emit ();
  int Talking ();
  int Mouth ();
  void Done ();


// PRIVATE MEMBER FUNCTIONS
private:
  // main functions
  void shutdown ();
  int load_voice (const char *base);

  // background thread functions
  void kill_prep ();
  static void *generate (void *tts);
  void make_prolog ();
  void get_phonemes ();
  void kill_emit ();
  static void *speak (void *dummy);

};



