#!/usr/bin/env python3
import rospy
from ros_game.msg import user_msg

class InfoUser:
    def __init__(self):
        rospy.loginfo("Initializing info_user node...")
        rospy.init_node('info_user')
        rospy.loginfo("Creating publisher for user_information topic...")
        self.user_pub = rospy.Publisher('user_information', user_msg, queue_size=10)
        self.rate = rospy.Rate(1)
    
    def get_valid_input(self, prompt, input_type="string"):
        """Helper method to get and validate user input"""
        while True:
            value = input(prompt).strip()
            if input_type == "string":
                if value:  # Check if not empty
                    return value
                print("Input cannot be empty. Please try again.")
            elif input_type == "int":
                try:
                    num_value = int(value)
                    if num_value > 0:
                        return num_value
                    print("Please enter a positive number.")
                except ValueError:
                    print("Please enter a valid number.")

    def run(self):
        rospy.loginfo("Starting user information collection...")
        
        # Get and validate user information
        name = self.get_valid_input("Enter your name: ")
        username = self.get_valid_input("Enter your username: ")
        age = self.get_valid_input("Enter your age: ", "int")

        # Create and fill the message
        user_message = user_msg()
        user_message.name = name
        user_message.username = username
        user_message.age = age

        # Log collected information
        rospy.loginfo("User information collected successfully:")
        rospy.loginfo(f"Name: {name}")
        rospy.loginfo(f"Username: {username}")
        rospy.loginfo(f"Age: {age}")

        # Publish information once
        rospy.loginfo("Publishing user information...")
        self.user_pub.publish(user_message)
        
        # Give time for the message to be sent
        rospy.sleep(1)
        
        rospy.loginfo("User information sent successfully. Shutting down node...")
        rospy.signal_shutdown("User information sent")

if __name__ == '__main__':
    try:
        node = InfoUser()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Node stopped by user")
    except rospy.ROSInterruptException:
        rospy.loginfo("Node stopped")
    finally:
        rospy.loginfo("Node shutdown complete")