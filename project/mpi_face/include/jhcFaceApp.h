// jhcFaceApp.h : layers jhcAnimHead over backdrop as Qt application
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

#pragma once

#include <QApplication>
#include <QWidget>
#include <pthread.h>
#include <jhcAnimHead.h>
#include <jhcFestTTS.h>


//= Layers jhcAnimHead over backdrop as Qt application.
// known to work with Qt5

class jhcFaceApp
{
// PRIVATE MEMBER VARIABLES
private:
  // overall application
  QApplication *app;
  char base[80];
  int ok;

  // graphics components
  QWidget *fbox, *curtain;
  jhcAnimHead *anim;

  // Text-To-Speech component
  jhcFestTTS tts;

  // background threads
  pthread_t lips, cgi;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcFaceApp ();
  jhcFaceApp ();  

  // main functions
  int Start (const char *dir =NULL);
  void Mood (int bits =0x0000)
    {if (ok > 0) {tts.Mood(bits); anim->SetMood(bits);}}
  void Say (const char *msg)
    {if (ok > 0) tts.Prep(msg);}
  void Stare (int doit =1)
    {if (ok > 0) anim->Stare(doit);}
  void Gaze (float pan =0.0, float tilt =0.0, float dps =0.0)
    {if (ok > 0) anim->SetGaze(pan, tilt, dps);}
  int Mouth ()
    {return((ok > 0) ? tts.Mouth() : -1);}
  void Done ();


// PUBLIC MEMBER FUNCTIONS
public:
  // background thread fcns
  static void *app_loop (void *inst)
    {jhcFaceApp *me = (jhcFaceApp *) inst; me->launch_qt(); return 0;}
  static void *sync_loop (void *inst)
    {jhcFaceApp *me = (jhcFaceApp *) inst; me->coord_lips(); return 0;}

  // background operations
  void launch_qt ();
  void init_graphics ();
  void coord_lips ();

  // configuration parameters
  int load_size (int& wid, int& ht) const;
  void load_face ();

};

