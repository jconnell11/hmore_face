// mpi_spout.cpp : speech output for MasterPi robot
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

#include <time.h>
#include <pthread.h>

#include <jhcFestTTS.h>


///////////////////////////////////////////////////////////////////////////
//                          Global Variables                             //
///////////////////////////////////////////////////////////////////////////

//= Text-to-speech audio player.

static jhcFestTTS tts;


//= Background TTS sequencing thread.
  
static pthread_t seq;


//= Whether background thread is currently active.

static int running = 0;


///////////////////////////////////////////////////////////////////////////
//                           Background Loop                             //
///////////////////////////////////////////////////////////////////////////

//= Play TTS audio file once it is ready.

static void *shovel (void *inst)
{
  timespec ts = {0, 50 * 1000000};     // 20 Hz

  while (tts.Active())                 // set false by tts.Done()
  {
    if (tts.Poised() > 0)              // TTS files have just become available
      tts.Emit();                               
    nanosleep(&ts, NULL); 
  } 
  return NULL;
}


///////////////////////////////////////////////////////////////////////////
//                           Main Functions                              //
///////////////////////////////////////////////////////////////////////////

//= Create and configure Text-to-Speech system.
// "dir" is where to find subdirectory "config" with voice parameters
// returns 1 if successful, 0 or negative for problem

extern "C" int face_start (const char *dir =NULL)
{
  // try starting Festival
  if (tts.Start(dir) <= 0)
    return 0;
  if (running > 0)
    return 1;

  // set up background thread to call tts.Emit() when ready
  pthread_create(&seq, NULL, shovel, NULL); 
  running = 1;
  return 1;
}


//= Set prosody for next utterance based on mood bits.
// [ surprised angry scared happy : unhappy bored lonely tired ]
// high-order bytes contain "very" bits for corresponding conditions

extern "C" void face_mood (int bits =0x0000)
{
  tts.Mood(bits);
}
 

//= Convert text to audio and start playing (does not block).

extern "C" void face_say (const char *msg)
{
  tts.Prep(msg);
}
  

//= Turn eyes to the "paying attention" color (1) or normal (0).
// Note: dummy function for compatibility with mpi_face.so

extern "C" void face_stare (int doit =1)
{
}


//= Progressively angle the head in some direction at some speed.
// Note: dummy function for compatibility with mpi_face.so

extern "C" void face_gaze (float pan =0.0, float tilt =0.0, float dps =0.0)
{
}


//= Report TTS status for "sock puppet" mouth animation.
// returns: -1 = silent, 0 = mouth closed, 1 = mouth open

extern "C" int face_mouth ()
{
  return tts.Mouth();
}


//= Cleanly shut down system.

extern "C" void face_done ()
{
  timespec ts;

  // cleanly shut down Festival
  tts.Done();
  if (running <= 0)
    return;

  // shut down background thread
  clock_gettime(CLOCK_REALTIME, &ts); 
  ts.tv_sec += 1;
  pthread_timedjoin_np(seq, 0, &ts);
  pthread_detach(seq);
  running = 0;
}
