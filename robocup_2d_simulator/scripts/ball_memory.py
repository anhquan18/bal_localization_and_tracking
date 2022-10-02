#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool


class Memory(object):
    def __init__(self):
        self.data = []
        self.past_status = True 
        self.simu_sub = rospy.Subscriber("robot_ball_information", PoseArray, self.callbackk_ball_information)
        self.simu_sub = rospy.Subscriber("compution_status", Bool, self.callbackk_compution_status)
        self.memo_pub = rospy.Publisher("ball_memories", PoseArray, queue_size=2)

    def callbackk_ball_information(self, msgs):
        #if len(msgs.poses) == 4 and (msgs.poses[3].position.x!=0.0 or msgs.poses[3].position.y!=0.0):
        print(msgs.poses)
        print(len(msgs.poses))
        if len(msgs.poses) == 4:
            self.data.append(msgs)
            #print("memory:", self.data[0].poses[3].position.x, self.data[0].poses[3].position.y, "time", rospy.get_time())

    def callbackk_compution_status(self, msg):
        if msg.data and len(self.data):
            self.past_status = msg.data
            self.memo_pub.publish(self.data.pop(0))
            time.sleep(0.001)

if __name__ == "__main__":
    rospy.init_node("ball_memory")
    b_memo = Memory()
    rospy.spin()
