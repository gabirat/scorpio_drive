#!/usr/bin/env python

import rospy
import canopen  # https://github.com/christiansandberg/canopen Check for example code
from geometry_msgs.msg import twist

# CONFIG
WHEELS_TO_ID = {
    "fl": 1, "rl": 2, "rr": 3, "fr": 4
}
SPEED_CONSTANT = 740



logger = print

class Wheel:
    def __init__(self, wheel_id, network, micontrol_eds):
        self.id = wheel_id
        self.node = network.add_node(wheel_id, micontrol_eds)
        self.node.sdo['Device command']['Device command - execute on change'].raw = 0x15  # Mode SubVel
        logger("Mode Sub Velocity")
        self._setup_pdo()

    def set_speed(self, speed):
        self.node.rpdo[4]['Device command.Device command - data 0'].raw = speed * SPEED_CONSTANT
        self.node.rpdo[4]['Device command.Device command - execute on change'].raw = 0x32
        pass

    def _setup_pdo(self):
        # Read current PDO configuration
        self.node.tpdo.read()
        self.node.rpdo.read()

        self.node.rpdo[4].clear()
        self.node.sdo['Device command']['Device command - data 1'].raw = 0
        self.node.rpdo[4].add_variable('Device command', 'Device command - data 0')
        self.node.rpdo[4].add_variable('Device command', 'Device command - execute on change')
        self.node.rpdo[4].enabled = True

        # Save new configuration (node must be in pre-operational)
        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.rpdo.save()

        # Start RPDO4 with an interval of 100 ms
        self.node.rpdo[4]['Device command.Device command - data 0'].raw = 0
        self.node.rpdo[4]['Device command.Device command - execute on change'].raw = 0x32
        self.node.rpdo[4].start(0.01)
        logger("PDO is set up")


class DriveModule:

    def __init__(self):

        # ROS Node setup
        rospy.init_node('wheel_controller', anonymous=True) #TODO check node name change works

        # CANOpen setup
        self.network = canopen.Network()
        can_interface = rospy.get_param('~can_interface')
        micontrol_eds = rospy.get_param('~micontrol_eds')
        self.network.connect(channel=can_interface, bustype='socketcan')
        self.enabled = 0

        self.wheels = {
            wheel_name: Wheel(id_, self.network, micontrol_eds) for wheel_name, id_ in WHEELS_TO_ID.items()
        }

        self.wheel_sets = {
            "right_set": (self.wheels["fr"], self.wheels["rr"]),
            "left_set": (self.wheels["fl"], self.wheels["rl"]),
        }

        self.network.nmt.state = 'OPERATIONAL'
        # ROS Node setup
        rospy.Subscriber('cmd_vel', twist, self.controller_data)
        logger("Subscribed to controller")


    def controller_data(self, data):
        if data.angular.z > 0:
            langular = 1 # [0;1]
            rangular = 1-abs(data.angular.z) # [0;1]
        else:
            rangular = 1 # [0;1]
            langular = 1-abs(data.angular.z) # [0;1]

        for wheel_set_key, wheel_set_value in self.wheel_sets.items():
            if wheel_set_key == "left_set":
                for wheel in wheel_set_value:
                    wheel.set_speed(langular * data.linear.x)
            if wheel_set_key == "right_set":
                for wheel in wheel_set_value:
                    wheel.set_speed(rangular * data.linear.x)

        #logger('Speed: ', speed)
        logger('\n')


if __name__ == '__main__':
    drive = DriveModule()
    rospy.spin()
    drive.network.disconnect()
