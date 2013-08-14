#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
import tf
import numpy as np

x = []
y = []
z = []
w = []

stop = 1000
counter = stop + 1
eulers = []
quats = []

def normalize(quat):
 ret = np.array(quat)
 return ret / np.sqrt(np.dot(ret, ret))

def middleIt(data):
  global counter
  global x, y, z, w
  counter = counter + 1
  if counter < stop:
    x.append(data.orientation.x)
    y.append(data.orientation.y)
    z.append(data.orientation.z)
    w.append(data.orientation.w)
    #quat1 = normalize([data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z])
    #print tf.transformations.euler_from_quaternion(quat1)
  if counter == stop:
    xMiddle = sum(x) / len(x)
    yMiddle = sum(y) / len(y)
    zMiddle = sum(z) / len(z)
    wMiddle = sum(w) / len(w)
    x = []
    y = []
    z = []
    w = []
    quat1 = normalize([wMiddle, xMiddle, yMiddle, zMiddle])
    

    '''
    eul = tf.transformations.euler_from_quaternion(quat1)
    quat1 = tf.transformations.quaternion_from_euler(0, eul[1], eul[2])
    '''    
  
    eul = tf.transformations.euler_from_quaternion(quat1)
    eulers.append(eul)
  
    quats.append(quat1)

    print "Yaw: ", eul[0], " | Pitch: ", eul[1], " | Rol: ", eul[2]
    
    if len(eulers) > 1:
            
      quat2 = quats[-2]
      dot = np.dot(quat1, quat2)
      redian = np.arccos(dot)
      print "Angle in RADIAN is: ", redian, "which equals ", redian * 180 / np.pi, "DEGREE"
      
      #eul2 = eulers[-2]
      


def restart(data):
  global counter
  counter = 0
  return []

if __name__ == '__main__':
  rospy.init_node("middle")
  sub = rospy.Subscriber("/imu/data", Imu, middleIt)
  restart = rospy.Service('/middle/restart', Empty, restart)

  try:
    print "Everything ready"
    rospy.spin()

  except rospy.ROSInterruptException:
    pass

