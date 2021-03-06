#! /usr/bin/env python

PACKAGE='robodart_control'

import roslib
roslib.load_manifest(PACKAGE)

import rospy

import sys
import pickle
import time
import threading

from tts import say

from Tkinter import Tk, Button, Frame, OptionMenu, StringVar

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from math import pi
from math import atan2
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf import TransformListener

import actionlib

from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from geometry_msgs.msg import PoseStamped

from robodart_vision.srv import Point, SetOffset
from std_srvs.srv import Empty

from pr2_plugs_msgs.msg import EmptyAction
from pr2_plugs_msgs.msg._EmptyActionGoal import EmptyActionGoal




class Robodart_control():
  
  # import ipdb; ipdb.set_trace()

  current_dart_number = 0 #The Number of the current Dart to throw (starting with 0)

  ##CONSTANTS
  dart_positions = []
  #dart_positions.append((-0.336,-0.218)) #x and y positions of dart number 0
  #dart_positions.append((-0.338,-0.158)) #x and y positions of dart number 1
  #dart_positions.append((-0.34,-0.098)) #x and y positions of dart number 2
  
  dart_positions.append((-0.336,-0.228)) #x and y positions of dart number 0
  dart_positions.append((-0.338,-0.168)) #x and y positions of dart number 1
  dart_positions.append((-0.34,-0.108)) #x and y positions of dart number 2

  #dart_positions.append((-0.354,0.1645)) #x and y positions of dart number 3
  #dart_positions.append((-0.354,0.223)) #x and y positions of dart number 4
  #dart_positions.append((-0.354,0.287)) #x and y positions of dart number 5
  
  dart_positions.append((-0.344,0.162)) #x and y positions of dart number 3
  #dart_positions.append((-0.345,0.222)) #x and y positions of dart number 4
  dart_positions.append((-0.344,0.221)) #x and y positions of dart number 4
  dart_positions.append((-0.344,0.279)) #x and y positions of dart number 5
  
  REFERENCE_FRAME = 'katana_base_link' #All positions are defined relative to this frame (robot frame)
  GRIPPER_FRAME = 'katana_gripper_tool_frame' #Name of the gripper frame
  
  AIMING_CENTER_POSITION = (0.478,0) #the aiming is done relative to this position, AIMING_CENTER_POSITION is defined in REFERENCE_FRAME
  
  pre_pickup_height = 0.527 #The height in REFERENCE_FRAME above the dart
  lift_offset = 0.067 #The lift height of the dart
  move_away_after_pickup_offset = 0.03 #Distance to move away from dart horizontally after pickup
  
  camera_dart_offset_persistent_filename = 'camera_dart_offset.persistent'
  last_throw_position_persistent_filename = 'last_throw_position.persistent'
  ##CONSTANDS END
  
  package_dir = None
  
  last_throw_position = AIMING_CENTER_POSITION

  last_position = [0,0]
  last_z_position = 0
  last_offset = [0,0]
  
  camera_dart_offset = [0,0]
  last_camera_dart_offset = [0,0]
  
  camera_dart_offset = [0,0] #This tupel describes the offset between the camera and the actual dart drop position.

  client = None
  
  group = None
  
  tf_listener = None
  
  my_robodart_vision = None

  #tk dropdown var
  var = None

  scene = None
  
  log_file = None
  
  lookAroundGoal = None

 
  def __init__(self):
    
    self.package_dir = roslib.packages.get_pkg_dir(PACKAGE) + '/'
    print "Package dir = ", self.package_dir
    
    self.load_camera_dart_offset_from_file()
    self.load_last_throw_position_from_file()
    
    #TODOc comment out
    #return
      


    #init action client
    self.client = actionlib.ActionClient('gripper_action_controller/gripper_command', GripperCommandAction)
    
    print 'Waiting for grippercommand action server'
    self.client.wait_for_server()
    
    
    
    #init move_group
    self.group = MoveGroupCommander('arm')
    self.group.set_pose_reference_frame(self.REFERENCE_FRAME)
    self.group.set_goal_tolerance(0.0001)
    
    #init tf listener
    self.tf_listener = TransformListener()
    
    self.scene = PlanningSceneInterface()

    rospy.loginfo("waiting for PTU servers")
    self.actionClientRight = actionlib.SimpleActionClient('robodart_control/look_at_right_magazin', EmptyAction)
    self.actionClientRight.wait_for_server()
    self.actionClientHome = actionlib.SimpleActionClient('robodart_control/look_at_home', EmptyAction)
    self.actionClientHome.wait_for_server()
    self.actionClientLeft = actionlib.SimpleActionClient('robodart_control/look_at_left_magazin', EmptyAction)
    self.actionClientLeft.wait_for_server()
    self.actionClientAround = actionlib.SimpleActionClient('robodart_control/look_around', EmptyAction)
    self.actionClientAround.wait_for_server()
    self.actionClientDart = actionlib.SimpleActionClient('robodart_control/look_at_dartboard', EmptyAction)
    self.actionClientDart.wait_for_server()
    self.actionClientSad = actionlib.SimpleActionClient('robodart_control/look_sad', EmptyAction)
    self.actionClientSad.wait_for_server()
    self.actionClientSentence = actionlib.SimpleActionClient('robodart_control/look_sentence', EmptyAction)
    self.actionClientSentence.wait_for_server()

    
    
    
    self.move_home()
    say("Hallo ich bin Kate!")
    print "Everything started successfully"

  def throw_dart(self, use_last_offset = True):
    #Center the dartboard according to the previously calibrated dart_center offset
    
    take_last_throw_position = False
    
    if use_last_offset:
      if (self.last_camera_dart_offset[0] == 0 and self.last_camera_dart_offset[1] == 0):
          print "I don't set the new Offset because i think i didn't find an Dart"
          take_last_throw_position = True
      else:
        
        self.camera_dart_offset[0] = self.last_camera_dart_offset[0]
        self.camera_dart_offset[1] = self.last_camera_dart_offset[1]
        self.save_camera_dart_offset_to_file()
        
      print "Control: Adjusted Camera Dart Offset: ",self.camera_dart_offset
    
    else:
      print 'Warning Offset was not adjusted!!!!!'
      

    
    self.look_at_dartboard()
    
    self.move_to_drop_position()

    say("Erfasse Zielscheibe!")
    time.sleep(5)
    self.take_reference_picture()
    
    bullseye_center_offset = self.get_bullseye_center_offset()

    say("Zielscheibe erkannt!")
    time.sleep(2)   
    
    total_offset_to_move = [0,0]
    total_offset_to_move[0] = bullseye_center_offset[0] - self.camera_dart_offset[0]
    total_offset_to_move[1] = bullseye_center_offset[1] - self.camera_dart_offset[1]
    
    if take_last_throw_position or not use_last_offset:
      total_offset_to_move[0] = self.last_throw_position[0]
      total_offset_to_move[1] = self.last_throw_position[1]
    else:
      total_offset_to_move[0] += self.last_throw_position[0]
      total_offset_to_move[1] += self.last_throw_position[1]

    print 'total offset to move',total_offset_to_move
    
    self.pickup_dart()

    self.look_at_dartboard()
    
    self.move_to_drop_position()
    
    self.move_to_position_in_robot_frame(total_offset_to_move)
    #self.move_relative_to_last_position_in_robot_frame(total_offset_to_move)
    
    self.last_throw_position = self.last_position
    self.save_last_throw_position_to_file()

    say("Vorsicht. Abwurf in.")
    time.sleep(2)
    say("Drei")
    time.sleep(1)
    say("zwei")
    time.sleep(1)
    say("eins")
    time.sleep(1)
    say("Abwurf!")

    self.open_gripper()

    time.sleep(1)
    say("Erfasse Pfeil!")
    self.move_to_drop_position()
    time.sleep(5)

    print "Old Camera Dart Offset", self.camera_dart_offset
    
    
    

    self.last_camera_dart_offset = self.get_dart_center_offset()
    
    with open(roslib.packages.get_pkg_dir(PACKAGE) + '/log_file.log', 'a') as log_file:
      
      log_file.write(';Camera_dart_offset:;' + str(self.camera_dart_offset[0]) + ';' + str(self.camera_dart_offset[1]))
    
      log_file.write(';Dart_No:;' + str(self.current_dart_number))

      log_file.write(';total offset to move:;' + str(total_offset_to_move))

      log_file.write(';last_throw_position:;' + str(self.last_throw_position))
      
      import datetime
      timestamp = str(datetime.datetime.now())
      
      log_file.write(';Last Camera Dart Offset: ;' + str(self.last_camera_dart_offset[0]) + ';' + str(self.last_camera_dart_offset[1]))
      print "Control: Last Camera Dart Offset: ",self.last_camera_dart_offset
    
      
      log_file.write(';timestamp;' + timestamp + '\n')
      
    say("Pfeil erkannt.")
       
    self.look_at_home()
       
    self.move_home()
    
  
  
  def throw_dart_with_old_offset(self):
    self.throw_dart(False)

  def throw_dart_from_drop_position(self):
   
    
    self.pickup_dart()

    #TODO: check if this is correct, else save last position in robot frame
    self.move_to_drop_position()

    say("Vorsicht. Abwurf in.")
    time.sleep(2)
    say("Drei")
    time.sleep(1)
    say("zwei")
    time.sleep(1)
    say("eins")
    time.sleep(1)
    say("Abwurf!")

    self.open_gripper()

    self.move_home()


  def reset_dart_camera_offset(self):
    print "reset_dart_camera_offset"
    self.camera_dart_offset[0] = 0.14
    self.camera_dart_offset[1] = 0
    
    self.save_camera_dart_offset_to_file()
    
    self.last_throw_position[0] = self.AIMING_CENTER_POSITION[0]
    self.last_throw_position[1] = self.AIMING_CENTER_POSITION[1]

    self.save_last_throw_position_to_file()
    

  def center_dart_board(self):
    print "Center Dart Board"
    self.move_to_drop_position()
    say("Berechne Ziel position.")
    print "Sleep 4 seconds"
    time.sleep(3) #wait till steady

    self.move_relative_to_last_position_in_robot_frame(self.get_bullseye_center_offset())


  def calibrate(self):
    print 'Calibrating'
    '''
    self.center_dart_board()
    self.open_gripper()
    '''
  def pickup_dart(self):
    say("Hole Pfeil")

    print 'Starting Pickup dart'

    self.open_gripper()

    print 'Move above first dart pickup position of the right or left magazin depending on the number'
    if self.current_dart_number < 3:
      
      #self.move_to_position_in_robot_frame(self.dart_positions[0], self.pre_pickup_height)
      self.look_at_right_magazin()
    else:
      #self.move_to_position_in_robot_frame(self.dart_positions[-1], self.pre_pickup_height)
      self.look_at_left_magazin()


    print 'Move above desired dart pickup position'
    self.move_to_position_in_robot_frame(self.dart_positions[self.current_dart_number], self.pre_pickup_height)
    
    #if darts are available increment current_dart_number
    if self.current_dart_number < len(self.dart_positions) - 1:
      self.current_dart_number += 1

    #if no more darts are available start from the beginning
    else:
      self.current_dart_number = 0

    print 'Move to pickup position'

    self.move_relative_to_last_position_in_robot_frame([0,0],-self.lift_offset)

    self.close_gripper()
 
    print 'Move above pickup position'
    self.move_relative_to_last_position_in_robot_frame([0,0],+self.lift_offset)
    
    self.look_at_home()

    #print 'Move away horizontally in x direction'
    #self.move_relative_to_last_position_in_robot_frame([self.move_away_after_pickup_offset,0])


  def pickup_dart_req(self, req):
    self.pickup_dart()
    return []


  def move_home(self):  
    print 'Move to home position'  
    self.group.set_named_target('home_stable')
    self.group.go()
    
  def move_to_drop_position(self):
    print 'Move to drop position'
    self.move_to_position_in_robot_frame(self.AIMING_CENTER_POSITION)
    
  def open_gripper(self):
    print "Open Gripper"
    goal = GripperCommandGoal()
    #positition open max 0.3
    goal.command.position = 0.1
    self.client.send_goal(goal)
    
  def close_gripper(self):
    print "Close Gripper"
    goal = GripperCommandGoal()
    #position close max -0.44
    goal.command.position = -0.7
    #goal.command.position = -0.23 
    self.client.send_goal(goal)
       
  def move_relative_to_last_position_in_gripper_frame(self, offset_vector = (0,0), z_offset = 0):
    

    """Moves to the position defined by last_position + offset_vector.
    
    Arguments:
    >>offset_vector -- the offset as tupel(x,y) (default (0,0))
    >>z_offset -- the z axis offset (default 0)
    """
    print 'Move relative to last position: ' , self.last_position, 'offset: ', offset_vector
    
    new_position = [self.last_position[0] + offset_vector[0], self.last_position[1] + offset_vector[1]]
    
    self.move_to_position_in_gripper_frame(new_position, self.last_z_position + z_offset)
    
    print 'New Position: ', new_position


  def move_relative_to_last_position_in_robot_frame(self, offset_vector = (0,0), z_offset = 0):
    """Moves to the position defined by last_position + offset_vector.
    
    Arguments:
    >>offset_vector -- the offset as tupel(x,y) (default (0,0))
    >>z_offset -- the z axis offset (default 0)
    """
    print 'Move relative to last position: ' , self.last_position, 'offset: ', offset_vector
    
    new_position = [self.last_position[0] + offset_vector[0], self.last_position[1] + offset_vector[1]]
    
    self.move_to_position_in_robot_frame(new_position, self.last_z_position + z_offset)
    
    print 'New Position: ', new_position

  def move_to_position_in_gripper_frame(self, target_point = [0,0], z_axis = 0.4):
    self.group.set_pose_reference_frame(self.GRIPPER_FRAME)

    self.move_to_position(target_point, z_axis)

  def move_to_position_in_robot_frame(self, target_point = [0,0], z_axis = 0.4):
    self.group.set_pose_reference_frame(self.REFERENCE_FRAME)

    self.move_to_position(target_point, z_axis)
   
    
  def move_to_position(self, target_point = [0,0], z_axis = 0.4):
    """Moves to the position given by target_point in self.REFERENCE_FRAME
    
    Arguments:
    >>target_point -- the offset as vector (default [0,0])
    """
    
    #save position for relative movement
    self.last_position = list(target_point)
    self.last_z_position = z_axis
    
    p = PoseStamped()
    p.header.frame_id = self.REFERENCE_FRAME
    p.pose.position.x = target_point[0]
    p.pose.position.y = target_point[1]
    
    p.pose.position.z = z_axis
    
    #p.pose.orientation.w = 1
    
    q = quaternion_from_euler(-0.032, -0.0376, 0)
    
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    
    
    #this is only needed for 5Dof Katana Robot
    self.replace_yaw_angle_with_reachable_value(p)

    #print "Planning frame: " ,self.group.get_planning_frame()
    #print "Pose reference frame: ",self.group.get_pose_reference_frame()
    
    #group.set_rpy_target([0, 0, 0],"katana_gripper_tool_frame")
    #group.set_position_target([p.pose.position.x,p.pose.position.y,p.pose.position.z], GRIPPER_FRAME)
    
   
    
    self.group.set_pose_target(p, self.GRIPPER_FRAME)
    
    max_trys = 10
    while max_trys > 0:
      
      retVal = self.group.go()
      if retVal is True:
        break
    
      
      max_trys -= 1
        
    if max_trys == 0:
      say('Fehler, Fehler: Ich kann die Zielposition nicht erreichen! ')
 
      time.sleep(10)
    
      raise Exception('Target position not reachable.')
        
        
    

    print "move to position", p.pose.position
    
    #print "Current rpy: " , self.group.get_current_rpy(self.GRIPPER_FRAME)

  def replace_yaw_angle_with_reachable_value(self, pose_stamped):
    #print "Original pose" , pose_stamped
    euler_orientation = list(euler_from_quaternion([pose_stamped.pose.orientation.x,pose_stamped.pose.orientation.y,pose_stamped.pose.orientation.z,pose_stamped.pose.orientation.w]))
    euler_orientation[2] = atan2(pose_stamped.pose.position.y, pose_stamped.pose.position.x)
    
    quaternion = quaternion_from_euler(euler_orientation[0], euler_orientation[1], euler_orientation[2])
    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]
    
    #print "Adjusted yaw: " , pose_stamped
    
  def get_current_gripper_position(self):
        
    try:
      (trans,rot) = self.tf_listener.lookupTransform(self.REFERENCE_FRAME, self.GRIPPER_FRAME, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'Error: No tf found!'
    
    print 'Transformation:' , trans
    print 'Rotation:' , rot
    
    return trans
    

  def save_camera_dart_offset_to_file(self):
    
    pickle.dump(self.camera_dart_offset, open(self.package_dir + self.camera_dart_offset_persistent_filename, 'wb'))
    
  def load_camera_dart_offset_from_file(self):
    try:
      self.camera_dart_offset = pickle.load(open(self.package_dir + self.camera_dart_offset_persistent_filename, 'rb'))
    except (IOError):
      print "Could not read persistent camera dart offset"

    print "Offset loaded from file", self.camera_dart_offset  
    
    
  def save_last_throw_position_to_file(self):
    
    pickle.dump(self.last_throw_position, open(self.package_dir + self.last_throw_position_persistent_filename, 'wb'))
    
  def load_last_throw_position_from_file(self):
    try:
      self.last_throw_position = pickle.load(open(self.package_dir + self.last_throw_position_persistent_filename, 'rb'))
    except (IOError):
      print "Could not read persistent camera dart offset"

    print "Throw Position loaded from file", self.last_throw_position
    
  '''
  def save_current_gripper_position(self):
    
    sf = 'value is %s' % self.var.get()
    
    self.saved_positions[name] = self.get_current_gripper_position()
    pickle.dump(self.saved_positions, open('positions.p', 'wb'))
  '''

  '''
  def get_gripper_position_by_name(self, name):
    self.saved_positions.update(pickle.load(open('positions.p', 'rb')))  
    return self.saved_positions[name]
  '''

  def take_reference_picture(self):
    resp = self.call_service('/robodart_vision/take_reference_picture', Empty)
    return resp

  def get_bullseye_center_offset(self):
    resp = self.call_service('/robodart_vision/get_bullseye_center_offset', Point)
    if resp is None:
      print "No bullseye center offset received!, no circles detected?"
      return [0,0]
    return [resp.x, resp.y]

  def get_dart_center_offset(self):
    resp = self.call_service('/robodart_vision/get_dart_center_offset', Point)
    if resp is None:
      print "No dart center offset received!, no arrows detected?"
      return [0,0]
    return [resp.x, resp.y]


  def look_at_right_magazin(self):
    print 'look_at_right_magazin'

    goal = EmptyActionGoal()
    self.actionClientRight.send_goal(goal)

  def look_at_left_magazin(self):
    print 'look_at_left_magazin'

    goal = EmptyActionGoal()
    self.actionClientLeft.send_goal(goal)


  def look_at_home(self):
    print 'look_at_home'
    goal = EmptyActionGoal()
    self.actionClientHome.send_goal(goal)
    
  
  def look_at_dartboard(self):
    print 'look_at_dartboard'
    goal = EmptyActionGoal()
    self.actionClientDart.send_goal(goal)

  def look_busy(self):
    print 'look_busy'

  def look_weird(self):
    print 'look_weird'
  
  def look_sad(self):
    rospy.loginfo('look_at_home')
    goal = EmptyActionGoal()
    self.actionClientSad.send_goal(goal)

  def look_sentence(self):
    rospy.loginfo('look_at_home')
    goal = EmptyActionGoal()
    self.actionClientSentence.send_goal(goal)

  def call_service(self, serviceName, srv):
    rospy.wait_for_service(serviceName)
    try:
      service_method = rospy.ServiceProxy(serviceName, srv)
      resp = service_method()  
      return resp
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return None
    
  def throw_dart_callack(self, req):   
    self.throw_dart()
    return []
  
  def throw_dart_with_old_offset_callback(self, req):
    self.throw_dart_with_old_offset()
    return []
  
  def throw_dart_from_drop_position_callback(self, req):
    self.throw_dart_from_drop_position()
    return []
  
  def open_gripper_callback(self, req):
    self.open_gripper()
    return []
  
  def close_gripper_callback(self, req):
    self.close_gripper()
    return []
  
  def move_to_drop_position_callback(self, req):
    self.move_to_drop_position()
    return []
  
  def move_home_callback(self, req):
    self.move_home()
    return []
  
  def reset_dart_camera_offset_callback(self, req):
    self.reset_dart_camera_offset()
    return []
  
  def say_sentence_1(self, req):
    say("Ich bitte Dich! Ich bin ein Roboter, ich mache keine Fehler. Ich werde hier locker gewinnen!")
    return []
  
  def say_sentence_2(self, req):
    self.look_sentence()
    time.sleep(1)
    say("Ach Du Scheisse.  Echt jetzt?  Was mach ich denn dann hier?")
    return []
  
  
  
  def say_achherje_callback(self, req):
    self.look_sad()
    time.sleep(1)
    say("Ach herr je")
    return []
  
  def start_looking_around(self, req):
    if self.lookAroundGoal is None:
      self.lookAroundGoal = EmptyActionGoal()
      self.actionClientAround.send_goal(self.lookAroundGoal)
    else:
      rospy.logwarn("Looking around already started!")
    return []
  
  def stop_looking_around(self, req):
    if self.lookAroundGoal is not None:
      self.actionClientAround.cancel_all_goals()
      self.lookAroundGoal = None
    else:
      rospy.loginfo("Not looking around.")
    return []
  
  def reset_to_arrow_1(self, req):
    self.current_dart_number = 0
    return []
  
def exit():

  sys.exit("Successfully shut down")

def stop():
  print 'Stop all movements'
  #TODO: stop alle movements

  
  
    
if __name__ == '__main__':
  # Initialize the node and name it.
  rospy.init_node('robodart_control_node')

  try:
    my_robodart_control = Robodart_control()
    
    rospy.Service('robodart_control/throw_dart', Empty, my_robodart_control.throw_dart_callack)
    rospy.Service('robodart_control/throw_dart_with_old_offset', Empty, my_robodart_control.throw_dart_with_old_offset_callback)
    rospy.Service('robodart_control/throw_dart_from_drop_position', Empty, my_robodart_control.throw_dart_from_drop_position_callback)
    rospy.Service('robodart_control/open_gripper', Empty, my_robodart_control.open_gripper_callback)
    rospy.Service('robodart_control/close_gripper', Empty, my_robodart_control.close_gripper_callback)
    rospy.Service('robodart_control/move_to_drop_position', Empty, my_robodart_control.move_to_drop_position_callback)
    rospy.Service('robodart_control/move_home', Empty, my_robodart_control.move_home_callback)
    rospy.Service('robodart_control/reset_dart_camera_offset', Empty, my_robodart_control.reset_dart_camera_offset_callback)
    rospy.Service('robodart_control/say_sentence_1', Empty, my_robodart_control.say_sentence_1)
    rospy.Service('robodart_control/say_sentence_2', Empty, my_robodart_control.say_sentence_2)
    rospy.Service('robodart_control/start_looking_around', Empty, my_robodart_control.start_looking_around)
    rospy.Service('robodart_control/stop_looking_around', Empty, my_robodart_control.stop_looking_around)
    rospy.Service('robodart_control/reset_to_arrow_1', Empty, my_robodart_control.reset_to_arrow_1)
    rospy.Service('robodart_control/say_achherje', Empty, my_robodart_control.say_achherje_callback)
    rospy.Service('robodart_control/pickup_dart', Empty, my_robodart_control.pickup_dart_req)
    
    
    
  except rospy.ROSInterruptException: pass
  
  print "All Services ready"
  
  rospy.spin()
