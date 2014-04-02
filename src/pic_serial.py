#!/usr/bin/env python

import roslib; roslib.load_manifest('quad_track')
import rospy
import serial
from geometry_msgs.msg import PointStamped
from quad_track.msg import IMUDataStamped
from sensor_msgs.msg import Joy
import tf

pub = rospy.Publisher('dataout', IMUDataStamped)
broadcaster = tf.TransformBroadcaster()
ser = serial.Serial("/dev/ttyUSB0", baudrate=38400, bytesize=8, stopbits=1, timeout=0.06, writeTimeout=0.1)

PI = 3.14159265359

# initilize joystick global vars
axis = [0.0,0.0,0.0]
buttons=[0,0,0,0]


def update(position):

    x = int(round(position.point.x))
    y = int(buttons[0]*100)
    z = int((100*(axis[2]+1)))

    dataout = [x,y,z,x,y,z]     #must be list of unsigned shorts

    # send out commands
    #bytesout = [0]*(2*len(dataout)+1)
    #bytesout[0] = "K"
    #for i in range(0,len(dataout)):
    #    bytesout[2*i+1]=dataout[i]>>8
    #    bytesout[2*i+2]=dataout[i]-(bytesout[2*i+1]<<8)
    #ser.flushInput()
    #ser.flushOutput()   
    #ser.write(bytearray(bytesout))
    ser.write(bytearray(["K",int((100*(axis[2]+1))),int((100*(axis[0]+1))),int((100*((-axis[1])+1)))]))

    # receive quadrotor pose + rates + start character
    bytesin = ser.read(7)

    if len(bytesin) == 7 and ord(bytesin[0]) == 75:

        datain = [0]*3
        for i in range(0,len(datain)):
            datain[i]=(ord(bytesin[2*i+1])<<8)+ord(bytesin[2*i+2])

        quad_pose = IMUDataStamped()
        quad_pose.header.stamp = rospy.Time.now()
        quad_pose.roll = (datain[0]-32768)/350.0
        quad_pose.pitch = (datain[1]-32768)/350.0
        quad_pose.yaw = datain[2]/180.0

        pub.publish(quad_pose)

        # send transform data to rviz
        broadcaster.sendTransform((position.point.z,position.point.y,position.point.x), \
            tf.transformations.quaternion_from_euler(quad_pose.roll*PI/180, quad_pose.pitch*PI/180, quad_pose.yaw*PI/180), \
            rospy.Time.now(),"quad","camera_depth_optical_frame")

    else:
        rospy.logwarn("Bad data received")
        ser.flushInput()

# Joystick callback - retrieve current joystick and button data
def joycall(joydata):
    global axis
    global buttons

    axis = joydata.axes
    buttons = joydata.buttons[0:4]
    axis = [joydata.axes[0],joydata.axes[1],joydata.axes[3]]


def PicSerial():
    rospy.init_node('Pic_Serial')
  
    if ser.isOpen() is False:
        ser.open()
    rospy.loginfo("Serial port opened: %s", ser.isOpen())
    rospy.Subscriber("quad_position", PointStamped, update)
    rospy.Subscriber("joy",Joy,joycall)

    rospy.spin()
    ser.close()


if __name__ == '__main__':
    PicSerial()
