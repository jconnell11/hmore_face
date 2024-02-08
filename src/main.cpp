// main.cpp : start up all portions of Hmore RobotFace ROS node
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

#include <QApplication>
#include <QString>
#include <ros/ros.h>

#include <jhcFaceNode.h>


// holds main application and primary ROS node
// for lip sync: rostopic pub -1 speak std_msgs/String "What's new dude?"
// for emotion:  rostopic pub -1 mood geometry_msgs/Point 1.0 0.0 5.0

int main (int argc, char *argv[])
{
  const char *name = "hmore_face";

  // create application object and animated face 
  QApplication app(argc, argv);
  app.setApplicationName(name);

  // initialize ROS then build main node 
  ros::init(argc, argv, name);
  jhcFaceNode face(&app);              // no main loop

  // start node msg loop and widgets
  face.start();                        // invokes run() then returns
  app.exec();                          // blocks until exit()
  return 0;
}


