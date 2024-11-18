#!/usr/bin/env python3
import rospy
from ros_game.msg import user_msg

class InfoUser:
    def __init__(self):
        # Inicializar el nodo
        rospy.init_node('info_user')
        # Crear el publisher
        self.user_pub = rospy.Publisher('user_information', user_msg, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz

        
    def run(self):
        name = input("Enter your name: ")
        username = input("Enter your username: ")
        age = int(input("Enter your age: "))
        
        user_message = user_msg()
        user_message.name = name
        user_message.username = username
        user_message.age = age

        rospy.loginfo("Publishing user information...")
        while not rospy.is_shutdown():
            self.publisher.publish(user_message)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = InfoUser()
        node.run()
    except rospy.ROSInterruptException:
        pass