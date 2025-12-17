#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_face.py : Python wrapper for animated face and TTS on MasterPi robot
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2025 Etaoin Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# =========================================================================

import time                            # for testing
from ctypes import CDLL, c_float
  
lib = CDLL('lib/libmpi_face.so')       # TTS + animated face
#lib = CDLL('lib/libmpi_spout.so')      # TTS only option

lib.face_gaze.argtypes = [c_float, c_float, c_float]


# Python wrapper for animated face and TTS on MasterPi robot

class MpiFace:

  # create and configure animated face and Text-to-Speech system
  # "dir" is where to find subdirs "config" and "mesh" with face geometry
  # returns 1 if successful, 0 or negative for problem

  def Start(self, dir):
    return lib.face_start(dir.encode())


  # set expression and prosody for next utterance based on mood bits
  # [ surprised angry scared happy : unhappy bored lonely tired ]
  # high-order bytes contain "very" bits for corresponding conditions

  def Mood(self, bits =0x0000):
    lib.face_mood(bits)


  # convert text to audio and start playing (does not block)

  def Say(self, txt):
    lib.face_say(txt.encode())


  # turn eyes to the "paying attention" color (1) or normal (0)

  def Stare(self, doit):
    return lib.face_stare(doit)


  # progressively angle the head in some direction at some speed

  def Gaze(self, pan, tilt, dps =0.0):
    lib.face_gaze(pan, tilt, dps)


  # report TTS status for "sock puppet" mouth animation.
  # returns: -1 = silent, 0 = mouth closed, 1 = mouth open

  def Mouth(self):
    return lib.face_mouth()


  # cleanly shut down system

  def Done(self):
    lib.face_done()


# =========================================================================

# simple test program for some utterances

if __name__ == "__main__":
  face = MpiFace()
  face.Start("/home/pi/Ganbei")

  # default settings
  print('--- hello there dude ---')
  face.Say("hello there dude")
  time.sleep(2)

  # angled, staring, happy speech
  print("\n--- the blue block is right next to the red thing ---")
  face.Stare(1)
  face.Mood(0x0)
  face.Gaze(-30.0, -20.0)
  face.Say("the blue block is right next to the red thing")
  time.sleep(5)  

  # cleanup
  face.Done()
  print('\n--- audio done ---')
   