// jhcAnimHead.cpp : graphics animation routines for talking robot head
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

#include <OgreSceneManager.h>
#include <OgreViewport.h>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <QX11Info>
#include <ros/ros.h>
#include <ros/package.h>
#include <sys/random.h>
#include <math.h>

#include <jhcAnimHead.h>


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Holds all the Ogre calls for morphing face and feature meshes between poses.
// there are 3 autonomic animation loops: breathing, blinking, and rotating
// a special animation track implements lip sync using a phoneme timing file
// extra short animation tracks change mouth and eyebrow positions with emotion

jhcAnimHead::jhcAnimHead (QWidget *parent) : QWidget(parent) 
{
  // mark that Ogre initialization is needed and clear basic items 
  setMinimumSize(100, 100);
  setup = 0;
  root = 0;
  win = 0;
  cam = 0;

  // set names for new tracks and clear other animation items 
  all_anim = 0;
  t_name = "mouth talk";
  t_anim = 0;
  t_trk = 0;
  m_name = "mouth emote";
  m_anim = 0;
  m_trk = 0;
  e_name = "eyebrows emote";
  e_anim = 0;
  e_trk = 0;
 
  // create and configure timer for rendering loop
  timer = new QTimer(this); 
  connect(timer, SIGNAL(timeout()), SLOT(main_loop()));
  timer->start(50);    
  timer->setSingleShot(true);          // in case of stalls elsewhere

  // default face appearance (used by init_ogre call at first frame)
  model = "GiGo";            // no eyelashes
  skin  = 0xFFFFFF;          // white
  back  = 0x000000;          // black
  iris  = 0xFF00FF;          // magenta
  stare = 0x80FF00;          // greenish
  brows = 0x000000;          // black
  eyes  = 0x000000;          // black
  mouth = 0x000000;          // black

  // initialize expression and gaze
  chg_expression(0.0, 0.0);
  pan0 = 0.0;
  tilt0 = 0.0;
  pan2 = 0.0;
  tilt2 = 0.0;
  nsp = 90.0;
}


//= Destructor cleans up allocated structures.

jhcAnimHead::~jhcAnimHead ()
{
  delete timer;
  Ogre::WindowEventUtilities::removeWindowEventListener(win, this);
  windowClosed(win);
  delete root;
}


///////////////////////////////////////////////////////////////////////////
//                               Ogre Setup                              //
///////////////////////////////////////////////////////////////////////////

//= Configure majority of graphics components.
// binds "root", "win" and "cam" member variables

void jhcAnimHead::init_ogre ()
{
  Ogre::SceneManager *scene;
  Ogre::Viewport *view;
  Ogre::Entity *head;
  Ogre::SceneNode *node;
  Ogre::Light *light, *spot;
  Ogre::ColourValue back_col;

  // set up Ogre to render into QWidget window 
  make_root();
  bind_window();
  windowResized(win);
  Ogre::WindowEventUtilities::addWindowEventListener(win, this);
  root->addFrameListener(this);

  // create a new scene and set up environmental lighting
  scene = root->createSceneManager(Ogre::ST_GENERIC); 
  light = scene->createLight("Light");
  light->setPosition(20, 80, 50);
  scene->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

  // add camera (face is flat disk 14cm in diameter)
  cam = scene->createCamera("Camera");
  cam->setPosition(Ogre::Vector3(0, 0, 12));     // 45 deg FOV -> 10cm square
  cam->lookAt(Ogre::Vector3(0, 0, 0));         
  cam->setAspectRatio(Ogre::Real(1));            // for initial face cropping
  cam->setNearClipDistance(1);

  // set up for one viewport covering entire window
  view = win->addViewport(cam);
  back_col.setAsARGB(0xFF000000 | back);
  view->setBackgroundColour(back_col);   

  // create spotlight (makes disk look more like sphere)
  spot = scene->createLight("Spot");
  spot->setType(Ogre::Light::LT_SPOTLIGHT);
  spot->setDirection(0, 0, -1);
  spot->setPosition(Ogre::Vector3(0, 0, 10));
  spot->setSpotlightRange(Ogre::Degree(10), Ogre::Degree(20));

  // load face mesh resources
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(model);
  set_materials();
  augment_mesh();  

  // create local scene
  head = scene->createEntity("Head", model + ".mesh");
  node = scene->getRootSceneNode()->createChildSceneNode("HeadNode", Ogre::Vector3(0, 0, 0));
  node->attachObject(head);

  // get pointers to animations
  all_anim = head->getAllAnimationStates();
  t_anim = head->getAnimationState(t_name);
  m_anim = head->getAnimationState(m_name);
  e_anim = head->getAnimationState(e_name);
  init_anim();
}


//= Configures core Ogre graphics rendering system (binds "root").

void jhcAnimHead::make_root ()
{
  std::string prefix, tools;
  Ogre::RenderSystemList list;
  Ogre::RenderSystem *render = NULL;
  int i, n;

  // CMakeLists.txt defines prefix depending on Ogre version
#ifdef OGRE_PLUGIN_PATH
  prefix = OGRE_PLUGIN_PATH + std::string("/");
#endif

  // find and set up rendering engine
  try
  {
    // basic system shell 
    root = new Ogre::Root();
    root->loadPlugin(prefix + "RenderSystem_GL");

    // look for valid renderer (similar to Gazebo code)
    list = root->getAvailableRenderers();
    n = list.size();
    for (i = 0; i < n; i++)
      if (list[i]->getName() == "OpenGL Rendering Subsystem")
      {
        render = list[i];
        break;
      }
    if (render == NULL)
      throw std::runtime_error("Could not find the OpenGL rendering subsystem!\n");

    // configure rendering properties
    render->setConfigOption("Full Screen", "No");
    render->setConfigOption("FSAA", "8");                  // anti-aliasing ON
    render->setConfigOption("RTT Preferred Mode", "FBO");
    render->setConfigOption("VSync", "Yes");
    render->setConfigOption("sRGB Gamma Conversion", "No");
    root->setRenderSystem(render);
    root->initialise(false);

    // provide path to mesh files
    tools = ros::package::getPath(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(tools + "/mesh", "FileSystem", model);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Failed to initialize Ogre: %s\n", e.what());
    throw;
  }
  root->restoreConfig();
}


//= Attach graphics routines to window from QWidget (binds "win").

void jhcAnimHead::bind_window ()
{
  Ogre::String hwin;
  Ogre::NameValuePairList vcfg;

  // render into pre-existing window from widget
  hwin = Ogre::StringConverter::toString((unsigned long) QX11Info::display()) + ":"
       + Ogre::StringConverter::toString((unsigned int)  QX11Info::appScreen()) + ":"
       + Ogre::StringConverter::toString((unsigned long) nativeParentWidget()->effectiveWinId());
  vcfg["externalWindowHandle"] = hwin;           // percolates moves & resizes
  vcfg["monitorIndex"] = "2";
  vcfg["FSAA"] = "8";                            // anti-aliasing ON                           
  vcfg["vsync"] = "true";
  vcfg["border"] = "none";
  win = root->createRenderWindow("RobotFace", width(), height(), false, &vcfg);
  win->setActive(true);

  // allow widget's window to be used
//  WId id = 0x0;
//  win->getCustomAttribute("WINDOW", &id);
//  QWidget::create(id);                           // obsolete? 
  QWidget::create();
  setAttribute(Qt::WA_PaintOnScreen, true);
  setAttribute(Qt::WA_OpaquePaintEvent);
}


//= Reset all motion sequences (blink, breathe, etc.).
// Note: needs "all_anim" to be bound first

void jhcAnimHead::init_anim ()
{
  Ogre::AnimationStateIterator anim_it = all_anim->getAnimationStateIterator();
  Ogre::AnimationState *a;

  while (anim_it.hasMoreElements())
  {
    a = anim_it.getNext();
    a->setEnabled(true);
    a->setLoop(a->getAnimationName() == "breathe");    // continuous
    a->setTimePosition(0);
  }
  blink2 = 0;
  wiggle2 = 0;
}


///////////////////////////////////////////////////////////////////////////
//                          Model Configuration                          //
///////////////////////////////////////////////////////////////////////////

//= Specify colors used to draw various parts of the face.
// frozen by init_ogre() when drawing first frame

void jhcAnimHead::set_materials ()
{
  Ogre::ColourValue skin_col, brows_col, eyes_col, mouth_col;

  // set basic colors using RGB integer specs
  skin_col.setAsARGB( 0xFF000000 | skin);
  iris_col.setAsARGB( 0xFF000000 | iris);
  alt_col.setAsARGB(  0xFF000000 | stare);
  brows_col.setAsARGB(0xFF000000 | brows);
  eyes_col.setAsARGB( 0xFF000000 | eyes);
  mouth_col.setAsARGB(0xFF000000 | mouth);

  // assign to various features
  color_item("Head",     skin_col);
  color_item("Iris",     iris_col);    // normal
  color_item("EyeBrows", brows_col);
  color_item("Eyes",     eyes_col);
  color_item("Mouth",    mouth_col);
}


//= Set the drawing color for some facial feature.

void jhcAnimHead::color_item (std::string item, Ogre::ColourValue col)
{
  Ogre::MaterialPtr material;

  material = Ogre::MaterialManager::getSingleton().load(item, model).staticCast<Ogre::Material>();
  Ogre::Technique *tech = material->createTechnique();
  Ogre::Pass *pass = tech->createPass();
  pass = material->getTechnique(0)->getPass(0);
  pass->setDiffuse(col);
}


//= Opens a face model to add animation tracks and extract poses. 
// A face mesh file contains sveral named animation sequences:
//   "breathe" = raising of face - automatically looped
//   "blink"   = closing of eyes - run semi-randomly
//   "rotate"  = wiggling of head - run semi-randomly
// An additional three are created here:
//   "mouth talk"     = for shaping mouth based on speech
//   "mouth emoote"   = for shaping mouth based on emotion
//   "eyebrows emote" = for moving eyebrows based on emotion

void jhcAnimHead::augment_mesh ()
{
  Ogre::MeshPtr mesh; 
  Ogre::VertexData *vdata;
  Ogre::VertexDeclaration *vdecl;
  Ogre::Animation *anim;
  int sub, ns;

  // organize and build tracks
  try
  {
    // open mesh file and reorganize submeshes for vertex animation
    mesh = Ogre::MeshManager::getSingleton().load(model + ".mesh", model);
    ns = mesh->getNumSubMeshes();
    for (sub = 1; sub <= ns; sub++)
    {
      vdata = mesh->getVertexDataByTrackHandle(sub); 
      vdecl = vdata->vertexDeclaration->getAutoOrganisedDeclaration(false, true, false);
      vdata->reorganiseBuffers(vdecl);
    }

    // add mouth and eyebrow vertex animation tracks
    if ((sub = find_submesh(mesh, "mouth")) > 0)
    {
      anim = mesh->createAnimation(t_name, 0);
      t_trk = anim->createVertexTrack(sub, Ogre::VAT_POSE);
      anim = mesh->createAnimation(m_name, 0);
      m_trk = anim->createVertexTrack(sub, Ogre::VAT_POSE);
    }   
    if ((sub = find_submesh(mesh, "eyebrows")) > 0)
    {
      anim = mesh->createAnimation(e_name, 0);
      e_trk = anim->createVertexTrack(sub, Ogre::VAT_POSE);
    }

    // associate specific pose indices with enumeration values
    cache_poses(mesh);
  }
  catch (std::runtime_error err)
  {
    // complain about loading given mesh
    std::string error = "OGRE ERROR: ";
    error.append(__FILE__);
    error.append(err.what());
    ROS_WARN(error.c_str());
  }
}


//= Determine which submesh is associated with a certain facial feature.
// poses have names like "eyes-blink2-1"
//   first number (2) is animation step (none = first step)
//   second number (1 +1) is submesh corresponding to colored item like "Iris"
// the "eyes-blink" animation affects submeshes 1+1 (iris) and 4+1 (eye)

int jhcAnimHead::find_submesh (Ogre::MeshPtr mesh, Ogre::String item) const
{
  Ogre::String pname;
  Ogre::PoseList poses = mesh->getPoseList();
  int i, np = mesh->getPoseCount();

  for (i = 0; i < np; i++)
  {
    pname = mesh->getPose(i)->getName();
    if (Ogre::StringUtil::startsWith(pname, item, true))
      return(atoi(Ogre::StringUtil::split(pname, "-", 3).at(2).c_str()) + 1); 
  }
  return 0;                  // not found
}


//= Build lookup table from F_POSE values to equivalent mouth or eyebrow pose index.
// Note: strings must be in same order as F_POSE values

void jhcAnimHead::cache_poses (Ogre::MeshPtr mesh)
{
  char key[FP_MAX][20] = {"mouth-rest", "mouth-wide", "mouth-narrow", "mouth-open", "mouth-close",
                          "mouth-smile", "mouth-sad", "mouth-disgusted", "mouth-afraid",
                          "eyebrows-rest", "eyebrows-up", "eyebrows-down", 
                          "eyebrows-sad", "eyebrows-frown"};
  Ogre::PoseList list = mesh->getPoseList();
  int d, i, np = mesh->getPoseCount();

  for (d = 0; d < FP_MAX; d++)          
  {
    pose_idx[d] = -1;                  // default to invalid
    for (i = 0; i < np; i++)
      if (Ogre::StringUtil::startsWith(list[i]->getName(), key[d], true))
        break;
    if (i < np)
      pose_idx[d] = i;      
  }
}


///////////////////////////////////////////////////////////////////////////
//                           Animation Playback                          //
///////////////////////////////////////////////////////////////////////////

//= Change displayed face image based on animation tracks.
// this slot called when timer elapses and sends "timeout" interrupt signal

void jhcAnimHead::main_loop ()
{
  // set up delay for next loop (20 Hz -> 50 ms)
  timer->start(50); 

  // configure Ogre system if not already done
  if (setup <= 0)
  {
    init_ogre();
    setup = 1;
  }

  // check for various overall termination conditions
  if (!root->renderOneFrame() || win->isClosed())
  {
    root->getRenderSystem()->shutdown();
    ros::shutdown();
  }

  // tell system to render next face frame
  shift_gaze();
  win->update();
  update();
}


//= Advance animation state while GPU is rendering.
// overrides default from Ogre::FrameListener
// Note: could change breathing rate by futzing addTime() call
//       could change random parameters for next blink and rotate
//       could disable behaviors by not reseting their time (e.g. rotate)

bool jhcAnimHead::frameRenderingQueued (const Ogre::FrameEvent& evt)
{
  Ogre::Real elapsed = evt.timeSinceLastFrame;
  unsigned long now = root->getTimer()->getMilliseconds();
  bool rc;

  // pass on to base class then check that rendering is needed
  rc = Ogre::FrameListener::frameRenderingQueued(evt);
  if (win->isClosed() || win->isHidden())
    return false;

  // go through all animation sequences
  Ogre::AnimationStateIterator anim_it = all_anim->getAnimationStateIterator();
  while (anim_it.hasMoreElements())
  {
    // advance the time index in mouth and eyebrow sequences
    Ogre::AnimationState* anim = anim_it.getNext();
    const Ogre::String& seq = anim->getAnimationName();
    if ((seq == t_name) || (seq == m_name) || (seq == e_name))
      anim->addTime(elapsed);

    // breathing animation auto-loops (could use f * elapsed)
    if (seq == "breathe")
      anim->addTime(elapsed);         
  
    // possibly randomly restart for next blink 
    if (seq == "blink")
    {
      anim->addTime(elapsed);     
      if (now > blink2)
      {
        anim->setTimePosition(0);
        blink2 = now + (int)(1000.0 * rand_rng(4.0, 6.0));
      }
    }

    // possibly randomly restart for next wiggle
    if (seq == "rotate")
    {
      anim->addTime(elapsed);
      if (now > wiggle2)
      {
        anim->setTimePosition(0);
        wiggle2 = now + (int)(1000.0 * rand_rng(12.0, 22.0));
      }
    }
  }
  return rc;
}


//= Generate a random value between lo and hi with 15 bit resolution.

float jhcAnimHead::rand_rng (float lo, float hi) const
{
  unsigned short val, top = 0x7FFF;
  double f; 
  int nb; 

  nb = getrandom(&val, 2, 0);
  f = (float)(val & top) / (float) top;
  return(lo + f * (hi - lo));
}


///////////////////////////////////////////////////////////////////////////
//                             Window Events                             //
///////////////////////////////////////////////////////////////////////////

//= Change the position of the talking head panel (generally only at start).
// overrides default from Ogre::WindowEventListener

void jhcAnimHead::moveEvent (QMoveEvent *evt)
{
  // native QWidget handler
  QWidget::moveEvent(evt);

  // corresponding action in Ogre (window is external)
  if (evt->isAccepted() && win)         
  {
    win->windowMovedOrResized();                             
    update();
  }
}


//= Change the size of the talking head panel (generally only at start).
// overrides default from Ogre::WindowEventListener

void jhcAnimHead::resizeEvent (QResizeEvent *evt)
{
  // native QWidget handler
  QWidget::resizeEvent(evt);

  // corresponding action in Ogre (window is external)
  if (evt->isAccepted())
  {
    const QSize& sz2 = evt->size();    
    if (win)
    {
      win->resize(sz2.width(), sz2.height());
      win->windowMovedOrResized();                         
    }
    if (cam)                 // stretch face to fit new box
    {
      Ogre::Real aspect = Ogre::Real(sz2.width()) / Ogre::Real(sz2.height());
      cam->setAspectRatio(aspect);
    }
  }
}


///////////////////////////////////////////////////////////////////////////
//                            External Hooks                             //
///////////////////////////////////////////////////////////////////////////

//= Set expression mixing weights based on polar coordinate emotion vector.
// takes "secs" to change face from current expression to new one over 
// zero or negative value invokes default of 0.5 seconds
// morphing from current expresion to new one handled by Ogre animation

void jhcAnimHead::SetEmotion (float mag, float dir, float secs)
{
  float lag = ((secs > 0.0) ? secs : 0.5);

  // clear eyebrow animation 
  e_anim->setLength(lag);
  e_anim->setTimePosition(0);
  e_trk->removeAllKeyFrames();
  e_trk->getParent()->setLength(lag);

  // clear mouth animation
  m_anim->setLength(lag);  
  m_anim->setTimePosition(0);
  m_trk->removeAllKeyFrames();
  m_trk->getParent()->setLength(lag);   

  // add mouth and eyebrow start frames
  mouth_mood(m_trk->createVertexPoseKeyFrame(0));
  eyebrow_mood(e_trk->createVertexPoseKeyFrame(0));

  // change expression for new emotion
  chg_expression(mag, dir);

  // add mouth and eyebrow finish frames
  mouth_mood(m_trk->createVertexPoseKeyFrame(lag));
  eyebrow_mood(e_trk->createVertexPoseKeyFrame(lag));
}


//= Adjust weights of expression poses based on saved "emag" and "edir".
// <pre>
//        120 unhappy  surprised 60
//                 \    /
//  180 scared ---- rest ---- happy 0
//                 /    \
//         240 angry   excited 300
// </pre>

void jhcAnimHead::chg_expression (float mag, float dir)
{
  int hex = (int)(dir / 60.0);
  float f = dir / 60.0 - hex, sc = ((mag > 0.0) ? mag : 0.0);
  float w0 = sc * (1.0 - f), w1 = sc * f;

  // determine overall degree of emotion
  rest = ((sc > 1.0) ? 0.0 : 1.0 - sc);

  // mix two nearest expressions by angle
  smile     = ((hex == 0) ? w0 : ((hex == 5) ? w1 : 0.0));
  surprised = ((hex == 1) ? w0 : ((hex == 0) ? w1 : 0.0));
  sad       = ((hex == 2) ? w0 : ((hex == 1) ? w1 : 0.0));
  disgusted = ((hex == 3) ? w0 : ((hex == 2) ? w1 : 0.0));     // looks scared
  angry     = ((hex == 4) ? w0 : ((hex == 3) ? w1 : 0.0));
  afraid    = ((hex == 5) ? w0 : ((hex == 4) ? w1 : 0.0));     // big mouth
}


//= Change iris color temporarily to alternate value.
// normal and alternate color frozen at beginning by init_ogre()
// change is instantaneous - no progressive cross fading

void jhcAnimHead::Stare (int doit)
{
  color_item("Iris", ((doit > 0) ? alt_col : iris_col));
}


//= Move pupils and perhaps facial features to look in some direction.
// both pan and tilt are in degrees, pan = 0.0 means straight ahead
// pan aesthetically limited to +/-45 deg and tilt to +/- 30 degs
// dps is transition speed in degrees/second, 0 defaults to 90 deg/sec 
// orientation ramping done by shift_gaze() in main_loop()

void jhcAnimHead::SetGaze (float pan, float tilt, float dps)
{
  float plim = 45.0, tlim = 30.0;

  // limit angles and save target
  pan2  = ((pan  < -plim) ? -plim : ((pan  < plim) ? pan  : plim));
  tilt2 = ((tilt < -tlim) ? -tlim : ((tilt < tlim) ? tilt : tlim));

  // save transition speed
  if (dps <= 0.0)
    nsp = 90.0;
  else
    nsp = dps;
}


//= Bring head gaze closer to target values while respecting rotation speed.
// actually moves camera to simulate head reorientation
  
void jhcAnimHead::shift_gaze ()
{
  float d2r = M_PI / 180.0, dp = pan2 - pan0, dt = tilt2 - tilt0;
  float f, p, t, r, cx, cy, cz;

  // determine angular step to take (at 20Hz)
  if ((fabs(dp) < 0.1) && (fabs(dt) < 0.1))
    return;
  f = nsp / (20.0 * sqrt(dp * dp + dt * dt));
  pan0  = ((f >= 1.0) ? pan2  :  pan0 + f * dp);
  tilt0 = ((f >= 1.0) ? tilt2 : tilt0 + f * dt);

  // figure out new spot for camera (z is out from face disc)
  t = -d2r * tilt0;
  cy = 12.0 * sin(t);
  r  = 12.0 * cos(t);
  p = -d2r * pan0;
  cx = r * sin(p);
  cz = r * cos(p);

  // reposition camera to simulate head reorientation            
  cam->setPosition(Ogre::Vector3(cx, cy, cz));  
  cam->lookAt(Ogre::Vector3(0, 0, 0));                             
}


//= Process input text and phoneme file to create timed lists.
// resulting animation is queued for playback via standard timer loop

void jhcAnimHead::LipSync (jhcFestTTS *tts)
{
  Ogre::VertexPoseKeyFrame *frame;
  const char *ph;
  float secs, early = 0.05;            // start mouth shape before sound
  int cnt = 0;           

  // clear animation and setup neutral mouth frame at time 0
  t_anim->setTimePosition(0);
  t_trk->removeAllKeyFrames();
  frame = t_trk->createVertexPoseKeyFrame(0);
  mixin_pose(frame, M_REST, 1.0);

  // make mouth key frame for phoneme and extend animation to include it
  while ((ph = tts->Phoneme(secs)) != NULL)
  {
    secs -= early;
    t_anim->setLength(secs);        
    t_trk->getParent()->setLength(secs);
    frame = t_trk->createVertexPoseKeyFrame(secs);
    mixin_pose(frame, viseme_for(ph), rand_rng(0.75, 1.0));
    cnt++;
  }

  // sanity check
  if (cnt <= 0)
    ROS_WARN("No phonemes from TTS synthesis");
}


//= Convert acoustic phoneme into visual mouth shape.

int jhcAnimHead::viseme_for (const char *ph) const
{
  char wide[9][5]   = {"ae", "ah", "ay", "eh", "el", "em", "en", "ey", "iy"};
  char narrow[7][5] = {"aa", "ao", "aw", "ow", "oy", "uh", "uw"};
  char close[6][5]  = {"b", "f", "m", "p", "v", "w"};
  char open[25][5]  = {"ax", "axr", "ch", "d", "dh", "dx", "er", "g", "hh", "hv", "ih", "jh", 
                       "k", "l", "n", "nx", "ng", "r", "s", "sh", "t", "th", "y", "z", "zh"};
  int i;

  if (strcmp(ph, "pau") == 0)
    return M_REST;                     
  for (i = 0; i < 9; i++)
    if (strcmp(ph, wide[i]) == 0)
      return M_WIDE;
  for (i = 0; i < 7; i++)
    if (strcmp(ph, narrow[i]) == 0)
      return M_NARROW;
  for (i = 0; i < 6; i++)
    if (strcmp(ph, close[i]) == 0)
      return M_CLOSE;
  for (i = 0; i < 25; i++)
    if (strcmp(ph, open[i]) == 0)
      return M_OPEN;
  ROS_WARN("Unknown phoneme \"%s\"", ph);
  return M_REST;                     
}


///////////////////////////////////////////////////////////////////////////
//                            Pose Functions                             //
///////////////////////////////////////////////////////////////////////////

//= Modify mouth shape based on current emotions.

void jhcAnimHead::mouth_mood (Ogre::VertexPoseKeyFrame *frame) const
{
  if (smile > 0.0) 
    mixin_pose(frame, M_SMILE, 0.85 * smile);
  if (sad > 0.0) 
    mixin_pose(frame, M_SAD, 0.85 * sad);
  if (angry > 0.0) 
    mixin_pose(frame, M_SAD, 0.85 * angry);
  if (disgusted > 0.0) 
    mixin_pose(frame, M_DISGUST, disgusted);
  if (afraid > 0.0)
  {
    mixin_pose(frame, M_OPEN,   0.9 * afraid);
    mixin_pose(frame, M_AFRAID, 1.0 * afraid);
  }
  if (surprised > 0.0)
  {
    mixin_pose(frame, M_NARROW, 0.5 * surprised);
    mixin_pose(frame, M_OPEN,   0.5 * surprised);
  }
  if (rest > 0.0)
    mixin_pose(frame, M_REST, rest);
}


//= Modify eyebrow position based on current emotions.

void jhcAnimHead::eyebrow_mood (Ogre::VertexPoseKeyFrame *frame) const
{
  float wt = 0.9;

  if (smile > 0.0)
    mixin_pose(frame, EB_UP, wt * smile);
  if (sad > 0.0)
    mixin_pose(frame, EB_SAD, wt * sad);
  if (angry > 0.0 )
    mixin_pose(frame, EB_DOWN, wt * angry);
  if (afraid > 0.0)
  {
    mixin_pose(frame, EB_UP,    wt * afraid);
    mixin_pose(frame, EB_FROWN, wt * afraid);
  }
  if (disgusted > 0.0) 
    mixin_pose(frame, EB_FROWN, wt * disgusted);
  if (surprised > 0.0) 
    mixin_pose(frame, EB_UP, wt * surprised);
  if (rest > 0.0) 
    mixin_pose(frame, EB_REST, wt * rest);
}


//= Add contribution from some mouth or eyebrow state to current pose.

void jhcAnimHead::mixin_pose (Ogre::VertexPoseKeyFrame *frame, int cat, float wt) const
{
  int i;

  if ((cat >= 0) && (cat < FP_MAX))
    if ((i = pose_idx[cat]) >= 0)
      frame->addPoseReference(i, wt);
}
