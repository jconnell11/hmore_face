// jhcFestTTS.cpp : simple interface to Linux Festival Text-To-Speech
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2023-2025 Etaoin Systems
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <sys/stat.h>

#include <jhcFestTTS.h>


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Destructor cleans up files and any allocated items.
// remove temporary RAM disk

jhcFestTTS::~jhcFestTTS ()
{
  Done();
}


//= Default constructor initializes certain values.

jhcFestTTS::jhcFestTTS ()
{
  // default voice
  freq  = 105;               // deep male pitch
  infl  = 13;                // moderate singsong
  slow  = 120;               // speak more slowly
  shift = 100;               // original tract length
  drama = 100;               // emotional modulation
  loud  = 0;                 // no volume change

  // neutral emotion
  fmult = 1.0;
  imult = 1.0;
  rmult = 1.0;

  // phonemes and thread status
  np = 0;
  prepping = 0;
  emitting = 0;
  ok = 0;
}


///////////////////////////////////////////////////////////////////////////
//                         Modulation Adjustment                         //
///////////////////////////////////////////////////////////////////////////

//= Set prosody for next utterance based on mood bits.
// [ surprised angry scared happy : unhappy bored lonely tired ]
// high-order bytes contain "very" bits for corresponding conditions

void jhcFestTTS::Mood (int bits)
{
  int feel = 5, very = 0;              // default neutral voice

  // prioritize bits in mood vector
  if ((bits & 0x80) != 0)
    very = 1;                          // excited (surprised)
  else if ((bits & 0x40) != 0)
  {
    feel = 3;                          // angry
    if ((bits & 0x4000) != 0)
      very = 1;
  }
  else if ((bits & 0x20) != 0)
  {
    feel = 4;                          // scared
    if ((bits & 0x2000) != 0)
      very = 1;
  }
  else if ((bits & 0x10) != 0)
  {
    feel = 1;                          // happy  
    if ((bits & 0x1000) != 0)
      very = 1;
  }
  else if ((bits & 0x04) != 0)
  {
    feel = 0;                          // bored
    if ((bits & 0x0400) != 0)
      very = 1;
  }
  else if ((bits & 0x02) != 0)
  { 
    feel = 2;                          // sad (lonely)
    if ((bits & 0x0200) != 0)
      very = 1;
  }
  else if ((bits & 0x01) != 0)
    feel = 0;                          // tired

  // set prosody indirectly
  Emotion(feel, very);
}


//= Set prosody for next utterance based on current coarse emotion.
// feel: 0 bored, 1 happy, 2 sad, 3 angry, 4 scared, 5 excited
// use Emotion(5, 0) for default neutral tone
// cf. Burkhardt + Sendlmeier 2000 "... Acoustical Correlates ..."

void jhcFestTTS::Emotion (int feel, int very)
{
  if ((feel <= 0) && (very > 0))       // bored          
    Prosody(-10, -25, -10);
  else if (feel <= 0)                  // tired         
    Prosody(-5, -12, -5);

  else if ((feel == 1) && (very > 0))  // happy         
    Prosody(20, 50, 0);
  else if (feel == 1)                  // pleased       
    Prosody(10, 50, 0);

  else if ((feel == 2) && (very > 0))  // sad           
    Prosody(-10, -10, -20);
  else if (feel == 2)                  // discontent    
    Prosody(-5, -5, -10);

  else if ((feel == 3) && (very > 0))  // angry         
    Prosody(-10, 50, 15);
  else if (feel == 3)                  // annoyed       
    Prosody(-5, 25, 7);

  else if ((feel == 4) && (very > 0))  // scared        
    Prosody(25, -25, 15);
  else if (feel == 4)                  // wary          
    Prosody(12, -12, 7);

  else if (very > 0)                   // excited       
    Prosody(10, 50, 20);
  else                                 // neutral
    Prosody(0, 0, 0);
}


//= Directly modify prosody parameters for pitch, inflection, and speech rate.
// each value is a percentage increase or decrease in base value
// overall scale factor "drama" (pct) can accentuate/diminish effect

void jhcFestTTS::Prosody (int fpc, int ipc, int rpc)
{
  fmult = 1.0 + 0.0001 * drama * fpc;
  imult = 1.0 + 0.0001 * drama * ipc;
  rmult = 1.0 + 0.0001 * drama * rpc;
}


///////////////////////////////////////////////////////////////////////////
//                              Main Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Configure Text-To-Speech system (blocks).
// "dir" is where to find subdirectory "config" with voice parameters
// returns 1 if successful, 0 or negative for problem

int jhcFestTTS::Start (const char *dir)
{
  char txt[80] = ".";
  struct stat sb;
  int rc;

  // stop Festival and phoneme enumeration if they are running
  if (ok > 0)
    shutdown();
  ok = 0;

  // get voice parameters from file
  if ((dir != NULL) && (strlen(dir) > 0))
    strcpy(txt, dir);
  load_voice(txt);

  // possibly set pulseaudio output device volume
  if (loud > 0)
  {
    sprintf(txt, "pactl set-sink-volume @DEFAULT_SINK@ %d%%", loud);
    rc = system(txt);
  }

  // make RAM disk for temporary TTS files (1M = 30 secs @ 16K mono 16 bit)
  if (stat("/mnt/tts_ram", &sb) != 0)
  {
    if (system("sudo mkdir /mnt/tts_ram") != 0)
      return -3;
    if (system("sudo chmod a+w /mnt/tts_ram") != 0)
      return -2;
    if (system("sudo mount -t tmpfs -o size=1m tmpfs /mnt/tts_ram") != 0)
      return -1; 
  }

  // start Festival TTS server (server eats 42MB of RAM with heap = 1M)
  // Note: Scheme environment can be slow to start (hence sleep)
  if (system("sudo nice -n -20 festival --server --heap 1000000 > /dev/null 2>&1 &") != 0)  
    return 0;
  sleep(1.0);          
  ok = 1;
  return 1;
}


//= Stop any sound in progress then stop server.

void jhcFestTTS::shutdown ()
{
  int rc;

  kill_emit();
  kill_prep();
  rc = system("sudo pkill festival"); 
}


//= Get desired TTS pitch, speed, etc. from YAML file based on machine name.
// returns loudness (percent) for use in setting external audio device

int jhcFestTTS::load_voice (const char *base) 
{
  char txt[200], tag[80];
  FILE *in;
  int n, loud = 0;                     // default = no change 

  // sanity check (NULL argument skips loading any file)
  if (base == NULL)
    return loud;

  // form full configuration file name and try opening
  gethostname(tag, 80);
  sprintf(txt, "%s/config/%s_voice.yaml", base, tag); 
  if ((in = fopen(txt, "r")) == NULL)
  { 
    sprintf(txt, "%s/config/%s_face.yaml", base, tag);
    if ((in = fopen(txt, "r")) == NULL)
      return loud;
  }

  // look for proper TTS parameters
  while (fgets(txt, 200, in) != NULL)
    if (sscanf(txt, "%s : %d", tag, &n) == 2)
    {
      if (strcmp(tag, "voice_freq") == 0)
        freq = n;
      else if (strcmp(tag, "voice_infl") == 0)
        infl = n;
      else if (strcmp(tag, "voice_shift") == 0)
        shift = n;
      else if (strcmp(tag, "voice_slow") == 0)
        slow = n;
      else if (strcmp(tag, "voice_drama") == 0)
        drama = n;
      else if (strcmp(tag, "voice_loud") == 0)
        loud = n;
    }

  // clean up
  fclose(in);
  return loud;
}


//= Generate acoustic speech from text input but don't play it.
// must poll Poised() then call Emit(), done when Talking() is zero
// Note: incurs latency of 300-1400ms depending on text length

void jhcFestTTS::Prep (const char *txt)
{
  FILE *out;

  // terminate any other ongoing request
  kill_prep();

  // move text to speak into a file for the server
  if ((out = fopen("/mnt/tts_ram/quip.txt", "w")) == NULL)
    return;
  fprintf(out, "%s\n", txt);
  fclose(out);

  // start background thread to generate speech files
  prepping = 1;
  pthread_create(&synth, NULL, generate, (void *) this);
}


//= Override any Prep request currently in progress (costs 50ms).

void jhcFestTTS::kill_prep ()
{
  timespec one_sec;
  int rc;
  
  if (Poised() != 0)
    return;
  rc = system("sudo pkill festival-client");
  clock_gettime(CLOCK_REALTIME, &one_sec); 
  one_sec.tv_sec += 1; 
  pthread_timedjoin_np(synth, 0, &one_sec);   
}


//= Have Festival ingest input files to generate output files (blocks).
// needs "quip.txt" from Prep and "config.scm" from Start
// terminates when "speech.wav" is ready for use

void *jhcFestTTS::generate (void *tts)
{
  char cmd[250] = "cd /mnt/tts_ram; festival_client --prolog config.scm --ttw quip.txt --output ";
  char post[80] = "speech.wav";
  jhcFestTTS *me = (jhcFestTTS *) tts; 
  int rc;

  // encode requested emotional variations to prosody
  me->make_prolog();

  // possibly change overall pitch (soundstretch may not be needed)
  if (me->shift != 100)
    sprintf(post, "synth.wav; soundstretch synth.wav speech.wav -pitch=%3.1f > /dev/null 2>&1", 
            12.0 * log(0.01 * me->shift) / log(2.0));
  strcat(cmd, post);  

  // run synthesis command and block until finished (usually less than a second)
  rc = system(cmd);

  // generate cached array of phoneme start times 
  me->get_phonemes();
  return NULL;
}


//= Make preamble file to set speech characteristics and save phonemes.
// Note: CG voices sound better but take 20-30x longer and 10x RAM!

void jhcFestTTS::make_prolog ()
{
  float f0  = fmult * freq / (shift * 0.01);
  float std = imult * (infl * 0.01) * f0;        // depends on fmult also
  float dur = (slow * 0.01) / rmult; 
  FILE *out;
 
  if ((out = fopen("/mnt/tts_ram/config.scm", "w")) == NULL)
    return;
  fprintf(out, "(set! int_lr_params (append");
  fprintf(out, " '((target_f0_mean %d) (target_f0_std %d))", int(f0 + 0.5), int(std + 0.5));
  fprintf(out, " (cddr int_lr_params)))\n");
  fprintf(out, "(Parameter.set 'Duration_Stretch %4.2f)\n", dur);
  fprintf(out, "(set! after_synth_hooks (lambda (utt) (begin");
  fprintf(out, " (utt.wave.rescale utt 2.6) (utt.save.segs utt \"/mnt/tts_ram/phonemes.txt\") )))\n");
  fclose(out);
}


//= Cache phoneme start times for use with mouth shape (and animated face).

void jhcFestTTS::get_phonemes ()
{
  char line[80];
  FILE *in;
  int n;

  // clear old array then try to open phoneme file from Festival
  np = 0;
  if ((in = fopen("/mnt/tts_ram/phonemes.txt", "r")) == NULL)
    return;

  // read next entry with valid format (e.g. "1.429 100 zh") 
  while (fgets(line, 80, in) != NULL)
    if ((sscanf(line, "%f %d %s", off + np, &n, ph[np]) == 3) && (n == 100))
      if (++np >= pmax)
        break;
  fclose(in);
}


//= Test if acoustic and auxilliary outputs have been generated yet.
// returns 1 if just finished, 0 if still working, -1 if nothing in progress 

int jhcFestTTS::Poised ()
{
  if (prepping <= 0)         // nothing started
    return -1;
  if (pthread_tryjoin_np(synth, NULL) == EBUSY) 
    return 0;
  prepping = 0;             
  return 1;                  // just finished - returned only once
}


//= Start playing already prepared acoustic waveform.
// poll Done to check when audio output is finished
// returns start time of audio platback

void jhcFestTTS::Emit ()
{
  kill_emit();
  emitting = 1;
  clock_gettime(CLOCK_BOOTTIME, &t0);
  pthread_create(&play, NULL, speak, NULL);
}


//= Override any Emit request currently in progress (costs 50ms).

void jhcFestTTS::kill_emit ()
{
  timespec one_sec;
  int rc;

  if (Talking() <= 0)
    return;
  rc = system("sudo pkill aplay");
  clock_gettime(CLOCK_REALTIME, &one_sec); 
  one_sec.tv_sec += 1; 
  pthread_timedjoin_np(play, 0, &one_sec);   
}


//= Send precomputed wave file to ALSA sound system (blocks).
// file name change prevents interference with follow-on Prep() call
// audio file usually takes several seconds to finish playing

void *jhcFestTTS::speak (void *dummy)
{
  int rc;

  rc = system("cd /mnt/tts_ram; mv speech.wav output.wav; aplay -q output.wav 2>/dev/null");
  return NULL;
}


//= Test whether something is currently being said (audio playing).
// returns 1 if yacking, 0 if just finished, -1 if nothing in progress

int jhcFestTTS::Talking ()
{
  if (emitting <= 0)         // nothing started
    return -1;
  if (pthread_tryjoin_np(play, NULL) == EBUSY)
    return 1;
  emitting = 0;              
  return 0;                  // just finished - returned only once
}


//= Report TTS status for "sock puppet" mouth animation.
// returns: -1 = silent, 0 = mouth closed, 1 = mouth open

int jhcFestTTS::Mouth () 
{
  timespec now;
  float play;
  int i;

  // determine current play time offset (if audio active)
  if (Talking() <= 0)
    return -1;
  clock_gettime(CLOCK_BOOTTIME, &now);
  play = (now.tv_sec - t0.tv_sec) + 1.0e-9 * (now.tv_nsec - t0.tv_nsec); 

  // find very next start time past current point
  if ((np <= 0) || (play < off[0]))
    return -1;
  for (i = 1; i < np; i++)
    if (play < off[i])
      break;

  // convert previous phoneme into open/closed decision
  if ((strchr("aeiou", ph[i - 1][0]) != NULL) && 
      (strchr("lmn",   ph[i - 1][1]) == NULL))
    return 1;  
  return 0;
}


//= Cleanly stop all parts of the system.

void jhcFestTTS::Done ()
{
  struct stat sb;
  int rc;

  // skip if already cleaned up
  if (ok <= 0)
    return;
  ok = 0;

  // stop current message, phoneme enumeration, and server
  shutdown();

  // remove temporary RAM disk
  if (stat("/mnt/tts_ram", &sb) != 0)
    return;
  rc = system("sudo umount /mnt/tts_ram");
  if (stat("/mnt/tts_ram", &sb) != 0)
    return;
  rc = system("sudo rm -r /mnt/tts_ram");
}
