#!/usr/bin/env python

import roslib; roslib.load_manifest('quad_track')
import rospy
import serial
from geometry_msgs.msg import PointStamped
from quad_track.msg import IMUDataStamped
import tf

pub = rospy.Publisher('dataout', IMUDataStamped)
broadcaster = tf.TransformBroadcaster()
ser = serial.Serial("/dev/ttyUSB0", baudrate=38400, bytesize=8, stopbits=1, timeout=0.06, writeTimeout=0.1)

PI = 3.14159265359


def update(position):

    x = int(round(position.point.x))
    y = int(round(position.point.y))
    z = int(round(position.point.z))

    dataout = [x,y,z,x,y,z]     #must be list of unsigned shorts

    # send out commands
    bytesout = [0]*(2*len(dataout)+1)
    bytesout[0] = "K"
    for i in range(0,len(dataout)):
        bytesout[2*i+1]=dataout[i]>>8
        bytesout[2*i+2]=dataout[i]-(bytesout[2*i+1]<<8)
    ser.flushInput()
    ser.flushOutput()   
    ser.write(bytearray(bytesout))

    # receive quadrotor pose + rates + start character
    bytesin = ser.read(13) 

    if len(bytesin) == 13 and ord(bytesin[0]) == 75:

        datain = [0]*6
        for i in range(0,len(datain)):
            datain[i]=(ord(bytesin[2*i+1])<<8)+ord(bytesin[2*i+2])
        
        quad_pose = IMUDataStamped()
        quad_pose.header.stamp = rospy.Time.now()
        quad_pose.droll = (datain[0]-32768)/32.0
        quad_pose.dpitch = (datain[1]-32768)/32.0
        quad_pose.dyaw = (datain[2]-32768)/32.0
        quad_pose.roll = (datain[3]-32768)/350.0
        quad_pose.pitch = (datain[4]-32768)/350.0
        quad_pose.yaw = datain[5]/180.0

        pub.publish(quad_pose)

        # send transform data to rviz
        broadcaster.sendTransform((position.point.z,position.point.y,position.point.x), \
            tf.transformations.quaternion_from_euler(quad_pose.roll*PI/180, quad_pose.pitch*PI/180, quad_pose.yaw*PI/180), \
            rospy.Time.now(),"quad","camera_depth_optical_frame")

        
    else:
        #rospy.logwarn("Bad data received")
        ser.flushInput()


def PicSerial():
    rospy.init_node('Pic_Serial')
  
    if ser.isOpen() is False:
        ser.open()
    rospy.loginfo("Serial port opened: %s", ser.isOpen())
    rospy.Subscriber("quad_position", PointStamped, update)

    rospy.spin()
    ser.close()


if __name__ == '__main__':
    PicSerial()
