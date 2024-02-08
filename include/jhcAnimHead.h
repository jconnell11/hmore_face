// jhcAnimHead.h : graphics animation routines for talking robot head
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

#pragma once

#include <OgreRoot.h>
#include <OgreRenderWindow.h>
#include <OgreCamera.h>
#include <std_msgs/String.h>
#include <QtWidgets>
#include <OgreWindowEventUtilities.h>

#include <jhcFestTTS.h>


//= Graphics animation routines for talking robot head.
// borrows heavily from TalkingHead in Homer Robot Face from Univ. Koblenz-Landau

class jhcAnimHead : public QWidget, public Ogre::FrameListener, public Ogre::WindowEventListener
{
  Q_OBJECT                   // needed for signals/slots

// PRIVATE CONSTANTS
private:
  // all the various deformations the mouth and eyebrows can take
  enum F_POSE {M_REST, M_WIDE, M_NARROW, M_OPEN, M_CLOSE, 
               M_SMILE, M_SAD, M_DISGUST, M_AFRAID, 
               EB_REST, EB_UP, EB_DOWN, EB_SAD, EB_FROWN, FP_MAX};


// PRIVATE MEMBER VARIABLES
private:
  // graphics components
  Ogre::Root *root;
  Ogre::RenderWindow *win;
  Ogre::Camera *cam;
  int setup;

  // animation and poses
  Ogre::AnimationStateSet *all_anim;
  Ogre::String t_name, m_name, e_name;
  Ogre::AnimationState *t_anim, *m_anim, *e_anim;
  Ogre::VertexAnimationTrack *t_trk, *m_trk, *e_trk;
  Ogre::ColourValue iris_col, alt_col;
  int pose_idx[FP_MAX];

  // main loop timer
  QTimer* timer;

  // appearance and animation state
  float rest, smile, surprised, afraid, sad, angry, disgusted;
  double pan0, tilt0, pan2, tilt2, nsp;
  unsigned long blink2, wiggle2;


// PUBLIC MEMBER VARIABLES
public:
  // face parameters
  std::string model;
  int skin, back, iris, stare, brows, eyes, mouth;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  jhcAnimHead (QWidget *parent =0);
  ~jhcAnimHead ();

  // external hooks
  void SetEmotion (float emag =0.0, float edir =0.0, float secs =0.0);
  void Stare (int doit =1);
  void SetGaze (float pan =0.0, float tilt =0.0, float dps =0.0);
  void LipSync (jhcFestTTS *tts);


public slots:
  // timer callback
  void main_loop ();                 


// PROTECTED MEMBER FUNCTIONS
protected:
  // window events 
  virtual void resizeEvent (QResizeEvent *evt);
  virtual void moveEvent (QMoveEvent *evt);


// PRIVATE MEMBER FUNCTIONS
private:
  // graphics setup
  void init_ogre ();
  void make_root ();
  void bind_window ();
  void init_anim ();  

  // model configuration
  void set_materials ();
  void color_item (std::string item, Ogre::ColourValue col);
  void augment_mesh ();
  int find_submesh (Ogre::MeshPtr mesh, Ogre::String item) const;
  void cache_poses (Ogre::MeshPtr mesh);

  // animation playback
  bool frameRenderingQueued (const Ogre::FrameEvent& evt);
  float rand_rng (float lo, float hi) const;

  // external hooks
  void chg_expression (float mag, float dir);
  void shift_gaze ();
  int viseme_for (const char *ph) const;

  // pose functions
  void mouth_mood (Ogre::VertexPoseKeyFrame *frame) const;
  void eyebrow_mood (Ogre::VertexPoseKeyFrame *frame) const;
  void mixin_pose (Ogre::VertexPoseKeyFrame *frame, int cat, float wt) const;

};


