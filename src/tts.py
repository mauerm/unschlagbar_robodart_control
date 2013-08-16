#!/usr/bin/env python
import sys
import time	#for delay
import pygst	#for playing mp3 stream
import gst
import os
import sys
import hashlib

def download(fileName, url):
  os.system('wget -q -U Mozilla -O "'+fileName+'" "'+url+'"')

def say(text, offline = False, redownload = False):

  print text
  fileName = hashlib.sha512(text).hexdigest() + ".mp3"

  text = text.split()
  
  tts = '+'.join(text)
  
  music_stream_uri = 'http://translate.google.com/translate_tts?tl=de&q=' + tts

  filePath = 'tts/'
  if not os.path.exists(filePath):
    filePath = 'src/tts/'
    if not os.path.exists(filePath):
      print "Couldn't find tts Folder"
  
  if not offline:
    if redownload or not os.path.exists(filePath + fileName):
      download(filePath + fileName, music_stream_uri)
    
    pl = gst.element_factory_make("playbin", "player")
    pl.set_property('uri','file://'+os.path.abspath(filePath + fileName))
    pl.set_state(gst.STATE_PLAYING)

  else:
    if os.path.exists(filePath + fileName):
      pl = gst.element_factory_make("playbin", "player")
      pl.set_property('uri','file://'+os.path.abspath(filePath + fileName))
      pl.set_state(gst.STATE_PLAYING)
    else:
      print "Couldn't find a preloaded File but running in Offlinemode"

if __name__ == "__main__":
  say("Hallo RTL. mein Name ist Kate")
  time.sleep(3)
