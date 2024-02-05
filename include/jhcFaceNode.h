// jhcFaceNode.h : ROS wrapper for jhcAnimHead graphics class
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

#include <QApplication>
#include <QWidget>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>

#include <jhcAnimHead.h>
#include <jhcFestTTS.h>


//= ROS wrapper for jhcAnimHead graphics class.

class jhcFaceNode : public QThread
{
// PRIVATE MEMBER VARIABLES
private:
  // primary instance variables
  ros::NodeHandle nh;
  QApplication *app;

  // graphics components
  QWidget *fbox, *curtain;
  jhcAnimHead *anim;

  // Text-To-Speech component
  jhcFestTTS tts;
  int talk;

  // message publishers
  ros::Publisher talk_pub;

  // message subscribers
  ros::Subscriber speak_sub, mood_sub, stare_sub, gaze_sub;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  jhcFaceNode (QApplication *qt);  
  ~jhcFaceNode ();


// PRIVATE MEMBER FUNCTIONS
private:
  // creation and initialization
  void init_graphics ();
  void init_speech ();

  // main polling loop
  void run ();

  // message callbacks
  void callbackSpeak (const std_msgs::String::ConstPtr& msg);
  void callbackMood (const geometry_msgs::Point::ConstPtr& msg);
  void callbackStare (const std_msgs::Bool::ConstPtr& msg);
  void callbackGaze (const geometry_msgs::Point::ConstPtr& msg);

};


