#! /usr/bin/env python

PACKAGE='robodart_gui'

import roslib
roslib.load_manifest(PACKAGE)

import rospy

import sys


from Tkinter import Tk, Button, Frame, OptionMenu, StringVar


def throw_dart(self):
  resp = self.call_service('/robodart_control/throw_dart', Empty)
  return resp


def call_service(self, serviceName, srv):
  rospy.wait_for_service(serviceName)
  try:
    service_method = rospy.ServiceProxy(serviceName, srv)
    resp = service_method()      
    return resp
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
    return None
  
    
def exit():

  sys.exit("Successfully shut down")

def stop():
  print 'Stop all movements'
  #TODO: stop alle movements

  
  
    
if __name__ == '__main__':
  # Initialize the node and name it.
  rospy.init_node('robodart_gui_node')

  try:
    my_robodart_control = Robodart_control()
    
    
  except rospy.ROSInterruptException: pass

  gui = Tk()
  
  mm_btn = Button(gui, command = my_robodart_control.open_gripper, text = 'Open Gripper', height=1, width=15)
  mm_btn.grid(row=0, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.close_gripper, text = 'Close Gripper', height=1, width=15)
  mm_btn.grid(row=1, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.move_to_drop_position, text = 'Drop Position', height=1, width=15)
  mm_btn.grid(row=2, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.move_home, text = 'Home', height=1, width=15)
  mm_btn.grid(row=3, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.get_current_gripper_position, text = 'Get Gripper Position', height=1, width=15)
  mm_btn.grid(row=4, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.pickup_dart, text = 'pickup_dart', height=1, width=15)
  mm_btn.grid(row=5, column=0)

  mm_btn = Button(gui, command = throw_dart, text = 'throw_dart', height=20, width=15)
  mm_btn.grid(row=6, column=0)

  mm_btn = Button(gui, command = my_robodart_control.center_dart_board, text = 'Center Dart Board', height=1, width=15)
  mm_btn.grid(row=7, column=0)

  mm_btn = Button(gui, command = stop, text = 'Stop', height=1, width=15)
  mm_btn.grid(row=8, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.reset_dart_camera_offset, text = 'Reset Dart-Camera-Offset', height=1, width=15)
  mm_btn.grid(row=9, column=0)

  mm_btn = Button(gui, command = exit, text = 'Exit', height=1, width=15)
  mm_btn.grid(row=10, column=0)

  gui.mainloop()
