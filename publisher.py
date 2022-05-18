#! /usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray
import time

class publish:
    def __init__(self):
        rospy.init_node("controlpublisher")
        self.P = rospy.Publisher("/convalue", Int16MultiArray, queue_size=1)

    def run(self):
        convalue= Int16MultiArray()
        convalue.data=[]
        speed = int(input())
        steer = int(input())
        sign = int(input())
        convalue.data.append(speed)
        convalue.data.append(steer)
        convalue.data.append(0)
        convalue.data.append(0)
        convalue.data.append(sign)
        self.P.publish(convalue)

if __name__ == "__main__":
    PP = publish()
    while not rospy.is_shutdown():
        PP.run()
        time.sleep(0.01)