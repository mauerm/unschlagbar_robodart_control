#!/usr/bin/env python
import sys
import time	#for delay
import pygst	#for playing mp3 stream
import gst
import os
import sys
import hashlib
import roslib.packages

TTS_PACKAGE_NAME = 'robodart_control'

pl = None


def download(fileName, url):
  os.system('wget -q -U Mozilla -O "'+fileName+'" "'+url+'"')

def say(text, offline = False, redownload = False):
  global pl
  
  if pl is not None:
    ret = pl.set_state(gst.STATE_READY)
    #print "set state ready", ret
  
  #pl.set_state(gst.STATE_READY)
  
    #print "getState:", gst.Element.get_state()

  print text
  fileName = hashlib.sha512(text).hexdigest() + ".mp3"

  text = text.split()
  
  tts = '+'.join(text)
  
  music_stream_uri = 'http://translate.google.com/translate_tts?tl=de&q=' + tts

  filePath = roslib.packages.get_pkg_dir(TTS_PACKAGE_NAME) + "/tts_mp3/"

  if not os.path.exists(filePath):
    print "Couldn't find tts Folder"
  
  if not offline:
    
    if redownload or not os.path.exists(filePath + fileName):
      download(filePath + fileName, music_stream_uri)
    
    pl = gst.element_factory_make("playbin", "player")
    pl.set_property('uri','file://'+os.path.abspath(filePath + fileName))
    ret = pl.set_state(gst.STATE_PLAYING)
    #print "set state:", ret
  else:
    if os.path.exists(filePath + fileName):
      pl = gst.element_factory_make("playbin", "player")
      pl.set_property('uri','file://'+os.path.abspath(filePath + fileName))
      ret = pl.set_state(gst.STATE_PLAYING)
      #print "set state:", ret
    else:
      print "Couldn't find a preloaded File but running in Offlinemode"

if __name__ == "__main__":
  say("Hallo RTL. mein Name ist Kate")
  time.sleep(3)
