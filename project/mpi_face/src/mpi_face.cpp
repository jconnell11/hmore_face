// mpi_face.cpp : speech output and animated face for MasterPi robot
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2025 Etaoin Systems
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

#include <jhcFaceApp.h>


///////////////////////////////////////////////////////////////////////////
//                          Global Variables                             //
///////////////////////////////////////////////////////////////////////////

//= Text-to-speech audio player and animated face graphics.

static jhcFaceApp face;


///////////////////////////////////////////////////////////////////////////
//                           Main Functions                              //
///////////////////////////////////////////////////////////////////////////

//= Create and configure animated face and Text-to-Speech system.
// "dir" is where to find subdirs "config" and "mesh" with face geometry
// returns 1 if successful, 0 or negative for problem

extern "C" int face_start (const char *dir =NULL)
{
  return face.Start(dir);
}


//= Set expression and prosody for next utterance based on mood bits.
// [ surprised angry scared happy : unhappy bored lonely tired ]
// high-order bytes contain "very" bits for corresponding conditions

extern "C" void face_mood (int bits =0x0000)
{
  face.Mood(bits);
}
 

//= Convert text to audio and start playing (does not block).

extern "C" void face_say (const char *msg)
{
  face.Say(msg);
}
  

//= Turn eyes to the "paying attention" color (1) or normal (0).

extern "C" void face_stare (int doit =1)
{
  face.Stare(doit);
}


//= Progressively angle the head in some direction at some speed.

extern "C" void face_gaze (float pan =0.0, float tilt =0.0, float dps =0.0)
{
  face.Gaze(pan, tilt, dps);
}


//= Report TTS status for "sock puppet" mouth animation.
// returns: -1 = silent, 0 = mouth closed, 1 = mouth open

extern "C" int face_mouth ()
{
  return face.Mouth();
}


//= Cleanly shut down system.

extern "C" void face_done ()
{
  face.Done();
}
