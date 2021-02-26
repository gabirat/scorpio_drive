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
        self.enabled = 0
        self.wheels = {
            "fl": self.network.add_node(1, micontrol_eds),
            "fr": self.network.add_node(2, micontrol_eds),
            "rl": self.network.add_node(3, micontrol_eds),
            "rr": self.network.add_node(4, micontrol_eds)
        }
        #network.nmt.state = 'OPERATIONAL'

        #ROS Node setup
        rospy.Subscriber('joy', Joy, self.controller_data)
        self.wheels['rl'].sdo['Device command']['Device command - execute on change'].raw = 0x12 #Mode Vel
        



    def controller_data(self, data):
        if data.buttons[7]:
            self.enabled = 1 if self.enabled == 0 else 0
            print('Power toogled: ', self.enabled)
            self.wheels['rl'].sdo['Power enable'].raw = self.enabled

        print('Position actual value:', self.wheels[wheel_key].sdo['Position actual value'].raw)
        print('Position actual value [count]:', self.wheels[wheel_key].sdo['Position actual value [count]'].raw)
        print('Torque actual value:', self.wheels[wheel_key].sdo['Torque actual value'].raw)
        print('Current actual value:', self.wheels[wheel_key].sdo['Current actual value'].raw)
        print('Current - actual value:', self.wheels[wheel_key].sdo['Current - actual value'].raw)
        print('Unfiltered velocity - actual value:', self.wheels[wheel_key].sdo['Unfiltered velocity - actual value'].raw)
        print('Velocity - desired value:', self.wheels[wheel_key].sdo['Velocity - desired value'].raw)
        trigger_norm = 1.0 - ((data.axes[4] + 1.0) / 2) #0.0 - 1.0
        self.wheels[wheel_key].sdo['Velocity - desired value'].raw = int(trigger_norm * 3000) # 0 - 500 RPM


if __name__ == '__main__':
    drive = DriveModule()
    rospy.spin()
    drive.network.disconnect()
