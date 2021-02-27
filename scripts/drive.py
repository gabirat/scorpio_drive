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

        self.setup_pdo(self.wheels['rl'])
        print("PDO is set up")
        self.network.nmt.state = 'OPERATIONAL'
        #ROS Node setup
        rospy.Subscriber('joy', Joy, self.controller_data)
        print("Subscribed to controller")
        self.wheels['rl'].sdo['Device command']['Device command - execute on change'].raw = 0x15 #Mode SubVel
        print("Mode Sub Velocity")
        


    def setup_pdo(self, node):
        # Read current PDO configuration
        node.tpdo.read()
        node.rpdo.read()

        node.rpdo[4].clear()
        node.sdo['Device command']['Device command - data 1'].raw = 0
        node.rpdo[4].add_variable('Device command', 'Device command - data 0')
        node.rpdo[4].add_variable('Device command', 'Device command - execute on change')
        node.rpdo[4].enabled = True

        # Save new configuration (node must be in pre-operational)
        node.nmt.state = 'PRE-OPERATIONAL'
        node.rpdo.save()

        # Start RPDO4 with an interval of 100 ms
        node.rpdo[4]['Device command.Device command - data 0'].raw = 0
        node.rpdo[4]['Device command.Device command - execute on change'].raw = 0x32
        node.rpdo[4].start(0.01)

    def controller_data(self, data):
        if data.buttons[7]:
            self.enabled = 1 if self.enabled == 0 else 0
            print('Power toogled: ', self.enabled)
            self.wheels['rl'].sdo['Power enable'].raw = self.enabled

        triggerR_norm = 1.0 - ((data.axes[4] + 1.0) / 2) #0.0 - 1.0
        triggerL_norm =       ((data.axes[5] - 1.0) / 2) #-1.0 - 0.0
        SPEED_CONSTANT = 740
        speed = int ((triggerR_norm + triggerL_norm) * SPEED_CONSTANT)

        print('Speed: ', speed)
        print('\n')

        self.wheels['rl'].rpdo[4]['Device command.Device command - data 0'].raw = speed
        self.wheels['rl'].rpdo[4]['Device command.Device command - execute on change'].raw = 0x32

if __name__ == '__main__':
    drive = DriveModule()
    rospy.spin()
    drive.network.disconnect()
