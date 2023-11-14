// jhcGenTTS.h : interface to a variety of Text-to-Speech implementations
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

#pragma once


//= Interface to a variety of Text-to-Speech implementations.

class jhcGenTTS
{
// PUBLIC MEMBER VARIABLES
public:
  int freq;        // voice pitch
  int infl;        // pitch variation
  int shift;       // formant raising
  int slow;        // stretch phrase


// PUBLIC MEMBER FUNCTIONS
public:
  jhcGenTTS () {}     
  virtual ~jhcGenTTS () {}

  //= Configure Text-To-Speech system (blocks).
  // can set volume percentage (0 = no change) for some audio device
  // returns 1 if successful, 0 or negative for problem
  virtual int Start (int vol =0, int dev =0) {return 1;}  

  //= Generate acoustic speech from text input and possibly play it.
  // split 1: poll Poised() then call Emit() then poll Talking() for done
  // split 0: poll Working () for done 
  virtual void Say (const char *txt, int split =0) =0;

  //= Test if acoustic and auxilliary outputs have been generated yet.
  // returns 1 if just finished, 0 if still working, -1 if nothing in progress 
  virtual int Poised () =0;

  //= Enumerate phoneme codes and audio start times in order.
  // reset by call to Say(), returns NULL when no more
  virtual const char *Phoneme (float& time) =0;

  //= Start playing acoustic waveform if in delayed mode.
  // poll Talking() to check when audio output is finished
  virtual void Emit () =0;

  //= Test whether something is currently being said (audio playing).
  // equally valid in single phase and two phase settings (= noisy)
  // returns 1 if yacking, 0 if just finished, -1 if nothing in progress
  virtual int Talking () =0;

  //= Test if single phase Say() is fully complete (synthesis and audio finished).
  // returns 1 if still working, 0 if just finished, -1 if nothing in progress
  virtual int Working () =0;

};


//= Construct instance of derived class if using shared library (or DLL).
// program should call delete on returned object at end 

extern "C" jhcGenTTS *new_jhcTTS ();

