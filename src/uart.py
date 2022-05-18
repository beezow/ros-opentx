#!/usr/bin/env python
import serial
import rospy
import sys
import numpy as np
from sensor_msgs.msg import Imu

class opentx_uart:
    def __init__(self, device):
        rospy.init_node("OpentxUART")

        self.serial = serial.Serial(device, 9600, timeout=1)
        if not self.serial.is_open:
            rospy.logfatal('Failed to connect to opentx')

        self.imu_publisher = rospy.Publisher("opentx_imu", Imu, queue_size=1)

        #rpy
        self.acc = np.asarray([0,0,0])
        #xyz
        self.gyro = np.asarray([0,0,0])
        
        self.serial.reset_input_buffer()

    def update_state(self):
        line = self.serial.readline()
        line = line.strip().split()
        if len(line) != 6:
            rospy.logerr('UART Timedout: %s', line)
            return False 
#        print(line)
        line = np.asarray([float(val) for val in line])
        self.acc = line[:3]
        self.gyro = line[3:]
        return True
    
    def publish_state(self):
        msg = Imu()
        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]
        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]
        self.imu_publisher.publish(msg)       

    def loop(self):
        while not rospy.is_shutdown():
            if self.update_state():
                self.publish_state()
#            print("gyro: ", self.gyro)
#            print("acc: ", self.acc)

if __name__ == "__main__":
    port = sys.argv[1]
    #print(port)
    rospy.loginfo('Connecting to OpenTX on port %s', port)
    telem = opentx_uart(port)
    telem.loop()
    
