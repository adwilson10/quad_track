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
ser = serial.Serial("/dev/ttyUSB1", baudrate=38400, bytesize=8, stopbits=1, timeout=0.06, writeTimeout=0.1)

PI = 3.14159265359

# initilize joystick global vars
axis = [0.0,0.0,0.0]
buttons=[0,0,0,0]

prev_pos = [0,0,0]
des_pos = [-0.1,0.75,1.75]
acc_err = [0,0,0]

kinect_height = 0.75

equil = 104

#flags: [start,auto]
flags = [0,0]

D = [80,5,80]
P = [50,12,50]
I = [0.5,0.15,0.5]


def update(position):

    if axis[2] != 0 or flags[0]==1:
        global flags
        global des_pos
        global acc_err
        global prev_pos

        flags[0]=1  

        ypos = -position.point.y + kinect_height
        xpos = position.point.x
        zpos = position.point.z

        des_pos[1] = 0.75
        des_pos[0] = -0.1
        des_pos[2] = 1.75

        if flags[1]==1:

            #if (ypos-des_pos[1])>0:
            #    Iy = I[1]/3
            #else:
            Iy=I[1]
            thrust = (equil) - int(round(P[1]*(ypos-des_pos[1]) + D[1]*(ypos-prev_pos[1]) + Iy*acc_err[1]))
            #xcontrol = int((100*(axis[0]+1)))
            #ycontrol = int((100*((-axis[1])+1)))
            xcontrol = 120+int(round(P[0]*(xpos-des_pos[0]) + D[0]*(xpos-prev_pos[0]) + I[0]*acc_err[0]))
            ycontrol = 90+int(round(P[2]*(zpos-des_pos[2]) + D[2]*(zpos-prev_pos[2]) + I[2]*acc_err[2]))

        else:
            thrust = int((100*(axis[2]+1)))
            xcontrol = int((100*(axis[0]+1)))
            ycontrol = int((100*((-axis[1])+1)))

        prev_pos = [xpos,ypos,zpos]
        acc_err = [acc_err[0]+(xpos-des_pos[0]), acc_err[1]+(ypos-des_pos[1]), acc_err[2]+(zpos-des_pos[2])]
        
        if thrust>255:
            thrust=255
        if thrust<0:
            thrust=0
        if xcontrol>255:
            xcontrol=255
        if xcontrol<0:
            xcontrol=0
        if ycontrol>255:
            ycontrol=255
        if ycontrol<0:
            ycontrol=0

        # send out commands
        ser.write(bytearray(["K",thrust,xcontrol,ycontrol]))

        # receive quadrotor pose + rates + start character
        bytesin = ser.read(8)

        if len(bytesin) == 8 and ord(bytesin[0]) == 75:

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
            broadcaster.sendTransform((position.point.x,position.point.y,position.point.z), \
                tf.transformations.quaternion_from_euler(0,0,-PI/2), \
                rospy.Time.now(),"quadz","camera_depth_optical_frame")

            if ord(bytesin[7])==0:
                rospy.logwarn("Low Battery")

        else:
            rospy.logwarn("Bad data received")
            ser.flushInput()

# Joystick callback - retrieve current joystick and button data
def joycall(joydata):
    global axis
    global buttons
    global flags
    global acc_err

    axis = joydata.axes
    buttons = joydata.buttons[0:4]
    axis = [joydata.axes[0],joydata.axes[1],joydata.axes[3]]

    if buttons[1]==1:
        flags[1]=1
        acc_err=[0,0,0]
    if buttons[0]==1:
        flags[1]=0
        acc_err=[0,0,0]


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
