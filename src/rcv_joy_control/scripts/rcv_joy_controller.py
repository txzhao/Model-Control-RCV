#!/usr/bin/env python

import rospy
from rcv_common_msgs.msg import control_command
from sensor_msgs.msg import Joy


STEERING_AXIS = 0
THROTTLE_AXIS = 4

MAX_TORQUE_FWD   = 30
MAX_TORQUE_BCKWD = -30

MAX_KAPPA_RIGHT = -1.0/8.0
MAX_KAPPA_LEFT  =  1.0/8.0

def scale_unit(v,mn,mx):
    ''' Scale -1,1 to [mn,mx]'''
    return (mx-mn)*(v--1.0)/2+mn


class Translator:
    def __init__(self):
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher("/control_command", control_command, queue_size=1)
        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)

        self.park_state = False
        self.last_park_change_time = rospy.get_rostime()

    def timer_callback(self, event):
        if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
            self.callback(self.last_published)

    def callback(self, message):
        rospy.logdebug("joy_translater received axes %s",message.axes)
        command = control_command()
        command.header = message.header
        torque_avg = scale_unit(message.axes[THROTTLE_AXIS],MAX_TORQUE_BCKWD,MAX_TORQUE_FWD)

        command.fl_torque = command.fr_torque = command.rl_torque = command.rr_torque = torque_avg

        if message.buttons[3]:
            #print " self.last_park_change_time+rospy.Duration(0.2) = ", self.last_park_change_time+rospy.Duration(0.2)," rospy.get_rostime() = ",rospy.get_rostime()
            if self.last_park_change_time+rospy.Duration(0.5) < rospy.get_rostime():
                self.park_state = not self.park_state
                self.last_park_change_time = rospy.get_rostime()

        command.park_engage = self.park_state


        kappa_ref = scale_unit(message.axes[STEERING_AXIS],MAX_KAPPA_RIGHT,MAX_KAPPA_LEFT)
        command.kappa=kappa_ref
        command.beta =0.0

        self.last_published = message
        self.pub.publish(command)



if __name__ == '__main__':
    rospy.init_node('joy_translator')
    rospy.loginfo("joy_translater starting...")
    t = Translator()
    rospy.spin()
