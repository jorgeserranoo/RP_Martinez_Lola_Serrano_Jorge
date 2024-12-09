#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import pygame

class ControlNodePygame:
    def __init__(self):
        rospy.init_node('control_node_pygame')
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        self.rate = rospy.Rate(60)  # 60Hz para coincidir con el juego
        
        # Inicializar Pygame
        pygame.init()
        # Ventana pequeña para capturar eventos
        self.screen = pygame.display.set_mode((200, 100))
        pygame.display.set_caption("Control Window")

    def run(self):
        rospy.loginfo("Control node (Pygame) is running...")
        
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Control window closed")
                    return
            
            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                self.control_pub.publish("LEFT")
            elif keys[pygame.K_RIGHT]:
                self.control_pub.publish("RIGHT")
            elif keys[pygame.K_SPACE]:
                self.control_pub.publish("SHOOT")
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ControlNodePygame()
        controller.run()
    except rospy.ROSInterruptException:
        pass

    pygame.quit()