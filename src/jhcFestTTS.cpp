// jhcFestTTS.cpp : simple interface to Linux Festival Text-To-Speech
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

//= Default destructor does necessary cleanup.

jhcFestTTS::~jhcFestTTS ()
{
  Shutdown();
}


//= Default constructor initializes certain values.

jhcFestTTS::jhcFestTTS ()
{
  // default values
  freq  = 105;               // deep male pitch
  infl  = 13;                // moderate singsong
  shift = 100;               // original tract length
  slow  = 120;               // speak more slowly
  split = 0;                 // single phase

  // state variables
  prepping = 0;
  emitting = 0;
}


//= Configure Text-To-Speech system (blocks).
// can optional set audio device volume percentage (0 = no change)
// returns 1 if successful, 0 or negative for problem

int jhcFestTTS::Start (int vol, int dev)
{
  char cmd[80];
  struct stat sb;
  int rc;

  // stop Festival and RAM disk if they already exist
  Shutdown();

  // possibly set pulseaudio output device volume
  if (vol > 0)
  {
    sprintf(cmd, "pactl set-sink-volume %d %d%%", dev, vol);
    rc = system(cmd);
  }

  // make RAM disk for temporary TTS files (1M = 30 secs @ 16K mono 16 bit)
  if (stat("/mnt/tts_ram", &sb) != 0)
    if (system("sudo mkdir /mnt/tts_ram") != 0)
      return 0;
  if (system("sudo mount -t tmpfs -o size=1m tmpfs /mnt/tts_ram") != 0)
    return 0; 

  // start Festival TTS server (server eats 42MB of RAM with heap = 1M)
  if (system("festival --server --heap 1000000 &") != 0)  
    return 0;

  // extra instructions for each client call
  make_prolog();

  // clear state
  prepping = 0;
  emitting = 0;
  return 1;
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


//= Cleanly terminate and deallocate all system pieces.

void jhcFestTTS::Shutdown ()
{
  struct stat sb;
  int rc;

  rc = system("pkill festival");                 // server
  if (stat("/mnt/tts_ram", &sb) != 0)
    return;
  rc = system("sudo umount /mnt/tts_ram");
  if (stat("/mnt/tts_ram", &sb) != 0)
    return;
  rc = system("sudo rm -r /mnt/tts_ram");
}


///////////////////////////////////////////////////////////////////////////
//                              Main Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Generate acoustic speech from text input and possibly play it.
// incurs latency of 300-1400ms depending on text length
// split 1: poll Poised() then call Emit() then poll Talking() for done
// split 0: poll Working () for done

void jhcFestTTS::Say (const char *txt)
{
  FILE *out;
  int rc;

  // override any other Prep request currently in progress (costs 50ms)
  if (Poised() == 0)
  {
    rc = system("pkill festival-client");
    pthread_join(synth, NULL);
  }

  // move text to speak into a file for the server
  if ((out = fopen("/mnt/tts_ram/quip.txt", "w")) == NULL)
    return;
  fprintf(out, "%s\n", txt);
  fclose(out);

  // start background thread to generate speech files
  prepping = 1;
  pthread_create(&synth, NULL, generate, (void *) this);
}


//= Have Festival ingest input files to generate output files (blocks).
// needs "quip.txt" from Prep and "config.scm" from Start
// terminates when "speech.wav" and "phonemes.txt" are ready for use
// can automatically launch Emit() when finished if split <= 0

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
  if (me->split <= 0)        
    me->Emit();
}


//= Test if acoustic and auxilliary files have been generated yet.
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


//= Start playing acoustic waveform if in delayed mode.
// poll Done to check when audio output is finished

void jhcFestTTS::Emit ()
{
  int rc;

  // override any other emit request currently in progress (costs 50ms)
  if (Talking() > 0)
  {
    rc = system("pkill aplay");
    pthread_join(play, NULL);
  } 

  // start background thread to play audio file
  emitting = 1;
  pthread_create(&play, NULL, speak, NULL);
}


//= Send precomputed wave file to ALSA sound system (blocks).
// file name change prevents interference with follow-on Say() call
// audio file usually takes several seconds to finish playing

void *jhcFestTTS::speak (void *dummy)
{
  int rc = system("cd /mnt/tts_ram; mv speech.wav output.wav; aplay -q output.wav");
}


//= Test whether something is currently being said (audio playing).
// equally valid in single phase and two phase settings (= noisy)
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


//= Test if single phase Say() is fully complete (synthesis and audio finished).
// returns 1 if still working, 0 if just finished, -1 if nothing in progress

int jhcFestTTS::Working ()
{
  if (Poised() == 0)         
    return 1;                // still in synthesis phase
  return Talking();
}

