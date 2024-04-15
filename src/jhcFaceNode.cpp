// jhcFaceNode.cpp : ROS wrapper for jhcAnimHead graphics class
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

#include <ros/xmlrpc_manager.h>

#include <jhcFaceNode.h>


// signal-safe flag for whether "rosnode kill" is received

sig_atomic_t volatile hmore_kill = 0;


//= replacement "shutdown" XMLRPC callback (from "rosnode kill")

void killCallback (XmlRpc::XmlRpcValue& p, XmlRpc::XmlRpcValue& res)
{
  if (p.getType() == XmlRpc::XmlRpcValue::TypeArray)
    if (p.size() > 1)
      hmore_kill = 1; 
  res = ros::xmlrpc::responseInt(1, "", 0);
}


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Make ROS node that responds to topics "speak", "mood", and "gaze".

jhcFaceNode::jhcFaceNode (QApplication *qt) 
{
  // save so can be terminated cleanly
  app = qt;

  // override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", killCallback);

  // set up animation and TTS components
  init_graphics();
  init_speech();

  // publishes one message
  talk_pub = nh.advertise<std_msgs::Bool>("talking", 1);

  // install message callbacks
  speak_sub = nh.subscribe<std_msgs::String>("speak", 10, &jhcFaceNode::callbackSpeak, this);
  mood_sub = nh.subscribe<geometry_msgs::Point>("mood", 10, &jhcFaceNode::callbackMood, this);
  stare_sub = nh.subscribe<std_msgs::Bool>("stare", 10, &jhcFaceNode::callbackStare, this);
  gaze_sub = nh.subscribe<geometry_msgs::Point>("gaze", 10, &jhcFaceNode::callbackGaze, this);
}


//= Set up windows and mesh animation for talking head.
// binds member variables "fbox", "curtain", and "anim"

void jhcFaceNode::init_graphics ()
{
  ros::NodeHandle nh2("~");
  QRect scr = QGuiApplication::primaryScreen()->geometry();
  QPalette pal;
  int head_w, head_h;

  // read desired head size (pixels)
  nh2.param("face_width",  head_w, 800);
  nh2.param("face_height", head_h, 600);

  // build centered black area for inserting robot face
  fbox = new QWidget(0, Qt::FramelessWindowHint);
  fbox->setWindowTitle("Face Animation");
  pal = fbox->palette();
  pal.setColor(QPalette::Window, Qt::black);
  fbox->setPalette(pal);
  fbox->setAutoFillBackground(true);
  fbox->setCursor(Qt::BlankCursor);
  fbox->setFixedSize(head_w, head_h);      
  fbox->move((scr.width() - head_w) / 2, (scr.height() - head_h) / 2);

  // create animated face component (stretch face to fit face box)
  anim = new jhcAnimHead(fbox);
  anim->setFixedSize(head_w, head_h); 

  // configure face appearance options
  nh2.getParam("face_model", anim->model);
  nh2.getParam("face_skin",  anim->skin);
  nh2.getParam("face_back",  anim->back);
  nh2.getParam("face_iris",  anim->iris);
  nh2.getParam("face_stare", anim->stare);
  nh2.getParam("face_brows", anim->brows);
  nh2.getParam("face_eyes",  anim->eyes);
  nh2.getParam("face_mouth", anim->mouth);

  // build an independent black backdrop window filling whole screen
  curtain = new QWidget(fbox, Qt::Window | Qt::FramelessWindowHint);
  curtain->setWindowTitle("Face Backdrop");
  pal = curtain->palette();
  pal.setColor(QPalette::Window, QColor(0xFF000000 | anim->back));
  curtain->setPalette(pal);
  curtain->setAutoFillBackground(true);
  curtain->setCursor(Qt::BlankCursor);
  curtain->setFixedSize(scr.width(), scr.height());

  // show backdrop then face on top of it (use ALT+TAB to escape)
  // to hide LXDE taskbar: Panel / Advanced / "Minimize panel when not in use"
  curtain->show();
  fbox->show();
}


//= Configure Festival TTS system.

void jhcFaceNode::init_speech ()
{
  ros::NodeHandle nh2("~");
  int loud;

  // voice characteristics
  nh2.getParam("voice_freq",  tts.freq);
  nh2.getParam("voice_infl",  tts.infl);
  nh2.getParam("voice_shift", tts.shift);
  nh2.getParam("voice_slow",  tts.slow);

  // start background server
  nh2.param("voice_loud", loud, 0);
  tts.Start(loud);
  talk = 0;
}


// cleanup any allocated items

jhcFaceNode::~jhcFaceNode ()
{
  // stop Festival
  tts.Done();

  // graphics components
  delete curtain;
  delete anim;
  delete fbox;
}


///////////////////////////////////////////////////////////////////////////
//                            Main Polling Loop                         //
///////////////////////////////////////////////////////////////////////////

//= Periodically check whether activities started by callbacks have finished yet.
// in ROS node.start() automatically invokes node.run() as main loop

void jhcFaceNode::run ()
{
  std_msgs::Bool yack;       

  // notify other nodes that face is ready by sending "talking=False"
  yack.data = false;             
  talk_pub.publish(yack);

  // run loop at 20 Hz, exit if ROS shuts down
  ros::Rate rate(20);
  while (ros::ok() && (hmore_kill <= 0))
  {
    // see if TTS files have just become available
    if (tts.Poised() > 0)
    {
      anim->LipSync(&tts);             // build then start animation
      tts.Emit();                      // start playing audio file
      talk = 1;
      yack.data = true;                // send message
      talk_pub.publish(yack);          // 250ms silence at start
    }

    // see if TTS audio has just finished playing
    if (talk > 0)
      if (tts.Talking() <= 0)
      {
        talk = 0;
        yack.data = false;             // send message
        talk_pub.publish(yack);        
      }

    // sleep until next cycle then service callbacks
    rate.sleep();
    ros::spinOnce();
  }

  // shutdown Festival and QT framework
  tts.Done();
  app->exit();    
}


///////////////////////////////////////////////////////////////////////////
//                            Message Callbacks                          //
///////////////////////////////////////////////////////////////////////////

//= Request Festival TTS to generate synth.wav and phonemes.txt files.
// takes a string to speak, including punctuation (like "?" at end)

void jhcFaceNode::callbackSpeak (const std_msgs::String::ConstPtr& msg)
{
  tts.Prep(msg->data.c_str());     
}


//= Change the facial expression based to reflect some mood.
// takes a point with x = magnitude, y = angle, z = transition time
// angle is in degrees, magnitude is usually in the 0 to 1 range
// transition time is in seconds, 0 converted to default of 0.25s
// example: point(1.0, 270.0, 0.0) -> angry shouting
// <pre>
//        120 unhappy  surprised 60
//                 \    /
//  180 scared ---- rest ---- happy 0
//                 /    \
//         240 angry   excited 300
// </pre>

void jhcFaceNode::callbackMood (const geometry_msgs::Point::ConstPtr& msg)
{
  anim->SetEmotion(msg->x, msg->y, msg->z);
}


//= Temporarily change eyes to alternate color (or revert them).
// typically used to indicate the robot is listening or hearing speech

void jhcFaceNode::callbackStare (const std_msgs::Bool::ConstPtr& msg)
{
  anim->Stare((msg->data) ? 1 : 0);
}


//= Move the eyes and perhaps head to look in some direction.
// takes a point with x = pan, y = tilt, z = slew rate
// pan and tilt are in degrees, slew rate is in deg/sec
// example: point(-15.0, 20.0, 0.0) -> slightly right and up

void jhcFaceNode::callbackGaze (const geometry_msgs::Point::ConstPtr& msg)
{
  anim->SetGaze(msg->x, msg->y, msg->z); 
}



