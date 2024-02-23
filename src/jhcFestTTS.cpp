// jhcFestTTS.cpp : simple interface to Linux Festival Text-To-Speech
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
  // default values
  freq  = 105;               // deep male pitch
  infl  = 13;                // moderate singsong
  shift = 100;               // original tract length
  slow  = 120;               // speak more slowly

  // initialize state variables
  in = NULL;
  prepping = 0;
  emitting = 0;
}


//= Configure Text-To-Speech system (blocks).
// can set volume percentage (0 = no change) of output device
// uses value from set-default-sink in /etc/pulse/default.pa
// returns 1 if successful, 0 or negative for problem

int jhcFestTTS::Start (int vol)
{
  char cmd[80];
  struct stat sb;
  int rc;

  // stop Festival and phoneme enumeration if they are running
  shutdown();

  // possibly set pulseaudio output device volume
  if (vol > 0)
  {
    sprintf(cmd, "pactl set-sink-volume @DEFAULT_SINK@ %d%%", vol);
    rc = system(cmd);
  }

  // make RAM disk for temporary TTS files (1M = 30 secs @ 16K mono 16 bit)
  if (stat("/mnt/tts_ram", &sb) != 0)
  {
    if (system("sudo mkdir /mnt/tts_ram") != 0)
      return -2;
    if (system("sudo mount -t tmpfs -o size=1m tmpfs /mnt/tts_ram") != 0)
      return -1; 
  }

  // start Festival TTS server (server eats 42MB of RAM with heap = 1M)
  // Note: Scheme environment can be slow to start (hence sleep)
  if (system("festival --server --heap 1000000 > /dev/null 2>&1 &") != 0)  
    return 0;
  sleep(1.0);          

  // extra instructions for each client call
  make_prolog();
  return 1;
}


//= Stop any sound in progress then stop server.

void jhcFestTTS::shutdown ()
{
  int rc;
  
  kill_emit();
  kill_prep();
  rc = system("pkill festival");   
}


//= Make preamble file to set speech characteristics and save phonemes.
// Note: CG voices sound better but take 20-30x longer and 10x RAM!

void jhcFestTTS::make_prolog ()
{
  int f0 = (int)(freq * 100.0 / shift + 0.5), std = (int)(0.01 * infl * f0 + 0.5);
  FILE *out;
 
  if ((out = fopen("/mnt/tts_ram/config.scm", "w")) == NULL)
    return;
  fprintf(out, "(set! int_lr_params (append");
  fprintf(out, " '((target_f0_mean %d) (target_f0_std %d))", f0, std);
  fprintf(out, " (cddr int_lr_params)))\n");
  fprintf(out, "(Parameter.set 'Duration_Stretch %4.2f)\n", 0.01 * slow);
  fprintf(out, "(set! after_synth_hooks (lambda (utt) (begin");
  fprintf(out, " (utt.wave.rescale utt 2.6) (utt.save.segs utt \"/mnt/tts_ram/phonemes.txt\") )))\n");
  fclose(out);
}


///////////////////////////////////////////////////////////////////////////
//                              Main Functions                           //
///////////////////////////////////////////////////////////////////////////

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
  int rc;
  
  if (Poised() != 0)
    return;
  rc = system("pkill festival-client");
  pthread_join(synth, NULL);
}


//= Have Festival ingest input files to generate output files (blocks).
// needs "quip.txt" from Prep and "config.scm" from Start
// terminates when "speech.wav" and "phonemes.txt" are ready for use

void *jhcFestTTS::generate (void *tts)
{
  char cmd[250] = "cd /mnt/tts_ram; festival_client --prolog config.scm --ttw quip.txt --output ";
  char post[80] = "speech.wav";
  jhcFestTTS *me = (jhcFestTTS *) tts; 
  int rc;

  // possibly change overall pitch (soundstretch may not be needed)
  if (me->shift != 100)
    sprintf(post, "synth.wav; soundstretch synth.wav speech.wav -pitch=%3.1f > /dev/null 2>&1", 
            12.0 * log(0.01 * me->shift) / log(2.0));
  strcat(cmd, post);  

  // run synthesis command and block until finished (usually less than a second)
  rc = system(cmd);
  return NULL;
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


//= Enumerate phoneme codes and audio start times in order.
// reset by call to Prep(), returns NULL when no more

const char *jhcFestTTS::Phoneme (float& secs)
{
  char line[80];
  int n;

  // make sure phonemes file from Festival is open
  if (in == NULL)
    if ((in = fopen("/mnt/tts_ram/phonemes.txt", "r")) == NULL)
      return NULL;

  // read next entry with valid format (e.g. "1.429 100 zh") 
  while (fgets(line, 80, in) != NULL)
    if ((sscanf(line, "%f %d %s", &secs, &n, ph) == 3) && (n == 100))
      return ph;

  // nothing left
  fclose(in);
  in = NULL;
  return NULL;
}


//= Start playing already prepared acoustic waveform.
// poll Done to check when audio output is finished
// returns start time of audio platback

void jhcFestTTS::Emit ()
{
  kill_emit();
  emitting = 1;
  pthread_create(&play, NULL, speak, NULL);
}


//= Override any Emit request currently in progress (costs 50ms).

void jhcFestTTS::kill_emit ()
{
  int rc;

  if (Talking() <= 0)
    return;
  rc = system("pkill aplay");
  pthread_join(play, NULL);
  if (in != NULL)
    fclose(in);
  in = NULL;
}


//= Send precomputed wave file to ALSA sound system (blocks).
// file name change prevents interference with follow-on Prep() call
// audio file usually takes several seconds to finish playing

void *jhcFestTTS::speak (void *dummy)
{
  int rc;

  rc = system("cd /mnt/tts_ram; mv speech.wav output.wav; aplay -q output.wav");
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


//= Cleanly stop all parts of the system.

void jhcFestTTS::Done ()
{
  struct stat sb;
  int rc;

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