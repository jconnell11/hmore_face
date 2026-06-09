// jhcFaceApp.cpp : layers jhcAnimHead over backdrop as Qt application
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

#include <jhcFaceApp.h>


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Default destructor does necessary cleanup.

jhcFaceApp::~jhcFaceApp()
{
  Done();
}


//= Default constructor initializes certain values.

jhcFaceApp::jhcFaceApp ()
{
  app = NULL;
  fbox = NULL;
  anim = NULL;
  curtain = NULL;
  ok = 0;                    // ready for single init try 
}


///////////////////////////////////////////////////////////////////////////
//                              Main Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Start up TTS server and face animation subsytem.
// specified "dir" is expected to have subdirs "config" and "mesh"
// returns 1 if successful, 0 or negative for problem

int jhcFaceApp::Start (const char *dir)
{
  int n;

  // sanity check (must be first and only attempt)
  if (ok != 0)
    return ok; 
  ok = -1;              

  // make sure directory is a valid starting point
  strcpy(base, ".");
  if ((dir != NULL) && (strlen(dir) > 0))
    strcpy(base, dir);

  // configure TTS
  if (tts.Start(base) <= 0)
    return ok;

  // start overall Qt app and background lip sync
  pthread_create(&cgi, NULL, app_loop, this);    
  pthread_create(&lips, NULL, sync_loop, this);    

  // wait for graphics to be completely set up (first call to main_loop())
  for (n = 0; n < 5; n++)
  {
    sleep(1);
    if ((anim != NULL) && anim->Ready())
      break;
  }
  if (n < 5)
    ok = 1;              
  return ok;
}


//= Cleanly shut down system.

void jhcFaceApp::Done ()
{
  timespec ts;

  // sanity check
  if (ok <= 0)
    return;
  
  // stop Qt and Festival
  if (app != NULL)
    app->exit();             // unblocks app->exec() in app_loop()
  tts.Done();                // stops coord_lips() with tts.Active()

  // kill background threads
  clock_gettime(CLOCK_REALTIME, &ts); 
  ts.tv_sec += 1;
  pthread_timedjoin_np(cgi, 0, &ts);
  pthread_timedjoin_np(lips, 0, &ts);
  pthread_detach(cgi);
  pthread_detach(lips);
}


///////////////////////////////////////////////////////////////////////////
//                         Background Operations                         //
///////////////////////////////////////////////////////////////////////////

//= Create and launch main Qt application.
// Note: all Qt objects must be created/destroyed in same thread as exec() call!

void jhcFaceApp::launch_qt ()
{
  char name[20] = "hmore_face";
  char *n = name;
  int i = 1;

  // make sure required X11 variables are set (e.g. for services)
  setenv("DISPLAY", ":0", 0);
  setenv("XAUTHORITY", "/home/pi/.Xauthority", 0);

  // create all Qt elements
  app = new QApplication(i, &n);   
  app->setApplicationName(name); 
  init_graphics();                     

  // start message loop (BLOCKS until app->exit() called!)
  app->exec();                         

  // dealloc all Qt elements
  delete curtain;
  delete anim;
  delete fbox;
  delete app;
}


//= Set up windows and mesh animation for talking head.
// binds member variables "fbox", "curtain", and "anim"

void jhcFaceApp::init_graphics ()
{
  QRect scr = QGuiApplication::primaryScreen()->geometry();
  QPalette pal;
  int dx, wid = 800, ht = 600;

  // load desired head size from file
  dx = load_size(wid, ht);

  // build centered black area for inserting robot face
  fbox = new QWidget(0, Qt::FramelessWindowHint);
  fbox->setWindowTitle("Face Animation");
  pal = fbox->palette();
  pal.setColor(QPalette::Window, Qt::black);
  fbox->setPalette(pal);
  fbox->setAutoFillBackground(true);
  fbox->setCursor(Qt::BlankCursor);
  fbox->setFixedSize(wid, ht);      
  fbox->move((scr.width() - wid) / 2 + dx, (scr.height() - ht) / 2);

  // create animated face component (stretch face to fit face box)
  // graphics not fully initialized until first call to main_loop()
  anim = new jhcAnimHead(base, fbox);
  anim->setFixedSize(wid, ht); 
  load_face();                         // needs valid "anim"

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
  curtain->show();
  fbox->show();
}


//= Periodically check whether lips and audio need to be coordinated.

void jhcFaceApp::coord_lips ()
{
  timespec ts = {0, 50 * 1000000};     // 20 Hz

  while (tts.Active())                 // set false by tts.Done()
  {
    if (tts.Poised() > 0)              // TTS files have just become available
    {
      // build animation then start it along with audio track
      if (anim != NULL)
        anim->LipSync(tts.ph, tts.off, tts.np);  
      tts.Emit();                               
    }
    nanosleep(&ts, NULL); 
  } 
}


///////////////////////////////////////////////////////////////////////////
//                       Configuration Parameters                        //
///////////////////////////////////////////////////////////////////////////

//= Get desired head size from YAML file based on machine name.
// assumes "wid" and "ht" already set to appropriate defaults
// returns amount of xshift for de-centering head

int jhcFaceApp::load_size (int& wid, int& ht) const
{
  char txt[200], tag[80];
  FILE *in;
  int n, xsh = 0;

  // sanity check
  if (base == NULL) 
    return xsh;

  // form full configuration file name and try opening
  gethostname(tag, 80);
  sprintf(txt, "%s/config/%s_face.yaml", base, tag); 
  if ((in = fopen(txt, "r")) == NULL)
  { 
    sprintf(txt, "%sconfig/hmore_face.yaml", base);
    if ((in = fopen(txt, "r")) == NULL)
      return xsh;
  }

  // look for proper size parameters
  while (fgets(txt, 200, in) != NULL)
    if (sscanf(txt, "%s : %d", tag, &n) == 2)
    {
      if (strcmp(tag, "face_width") == 0)
        wid = n;
      else if (strcmp(tag, "face_height") == 0)
        ht = n;
      else if (strcmp(tag, "face_xshift") == 0)
        xsh = n;
    } 

  // clean up
  fclose(in);  
  return xsh;              
}


//= Get desired face color, etc. from YAML file based on machine name.
// assumes "anim" exists and already has appropriate defaults set

void jhcFaceApp::load_face () 
{
  char txt[200], tag[80], val[80];
  FILE *in;
  unsigned long n;

  // sanity check
  if ((base == NULL) || (anim == NULL))
    return;

  // form full configuration file name and try opening
  gethostname(tag, 80);
  sprintf(txt, "%s/config/%s_face.yaml", base, tag); 
  if ((in = fopen(txt, "r")) == NULL)
  { 
    sprintf(txt, "%sconfig/hmore_face.yaml", base);
    if ((in = fopen(txt, "r")) == NULL)
      return;
  }

  // look for proper graphics parameters
  while (fgets(txt, 200, in) != NULL)
    if (sscanf(txt, "%s : %s", tag, val) == 2)
    {
      // colors are hex numbers (prefixed with "0x")
      if (strcmp(tag, "face_skin") == 0)
        anim->skin = strtoul(val, NULL, 16);
      else if (strcmp(tag, "face_back") == 0)
        anim->back = strtoul(val, NULL, 16);
      else if (strcmp(tag, "face_iris") == 0)
        anim->iris = strtoul(val, NULL, 16);
      else if (strcmp(tag, "face_stare") == 0)
        anim->stare = strtoul(val, NULL, 16);
      else if (strcmp(tag, "face_brows") == 0)
        anim->brows = strtoul(val, NULL, 16);
      else if (strcmp(tag, "face_eyes") == 0)
        anim->eyes = strtoul(val, NULL, 16);
      else if (strcmp(tag, "face_mouth") == 0)
        anim->mouth = strtoul(val, NULL, 16);
      else if (strcmp(tag, "face_model") == 0)
      {
        // remove quotes from model name ("GiGo")
        val[strlen(val) - 1] = '\0';
        anim->model = val + 1;
      }
    } 

  // clean up
  fclose(in);
}
 

