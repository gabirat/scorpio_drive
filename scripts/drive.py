#!/usr/bin/env python

import rospy
import canopen #https://github.com/christiansandberg/canopen Check for example code
from sensor_msgs.msg import Joy

class DriveModule:

    def __init__(self):

        #ROS Node setup
        rospy.init_node('drive', anonymous=True)

        #CANOpen setup
        self.network = canopen.Network()
        can_interface = rospy.get_param('~can_interface')
        micontrol_eds = rospy.get_param('~micontrol_eds')
        self.network.connect(channel=can_interface, bustype='socketcan')
        self.wheels = {
            "fl": self.network.add_node(1, micontrol_eds),
            "fr": self.network.add_node(2, micontrol_eds),
            "rl": self.network.add_node(3, micontrol_eds),
            "rr": self.network.add_node(4, micontrol_eds)
        }
        #network.nmt.state = 'OPERATIONAL'

        #ROS Node setup
        rospy.Subscriber('joy', Joy, self.controller_data)
        self.wheels['rl'].sdo['Power enable'].raw = 1



    def controller_data(self, data):
        print 'Controler data'
        for wheel_key in self.wheels:
            print self.wheels[wheel_key].sdo['Current actual value'].raw
            self.wheels[wheel_key].sdo['Velocity - desired value'].raw = data.axes[4] * 500 # 0 - 500 RPM



if __name__ == '__main__':
    drive = DriveModule()
    rospy.spin()
    drive.network.disconnect()
