#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from ros_game.msg import user_msg

class ResultNode:

    def __init__(self):
        rospy.init_node('result_node', anonymous=True)
        rospy.loginfo("Initializing result node...")

        self.username = None
        self.score = None
        self.result_received = False

        rospy.Subscriber("user_information", user_msg, self.user_callback)
        rospy.Subscriber("result_information", Int64, self.result_callback)

        self.rate = rospy.Rate(1)

    def user_callback(self, data):
        self.username = data.username
        rospy.loginfo("Username received")

    def result_callback(self, data):
        self.score = data.data
        self.result_received = True
        self.display_final_result()
    
    def display_final_result(self):

        if self.username and self.score is not None:
            rospy.loginfo("="*50)
            rospy.loginfo("GAME OVER - FINAL RESULTS")
            rospy.loginfo("="*50)
            rospy.loginfo(f"Player: {self.username}")
            rospy.loginfo(f"Final Score: {self.score}")
            rospy.loginfo("="*50)
            rospy.signal_shutdown("Game finished")

    def run(self):
        rospy.loginfo("Running result node...")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        result_node = ResultNode()
        result_node.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt exception")
