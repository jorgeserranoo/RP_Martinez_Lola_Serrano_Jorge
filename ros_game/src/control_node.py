#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import termios
import tty
import threading
import time

class ControlNode:
    def __init__(self):
        rospy.init_node('control_node')
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        self.rate = rospy.Rate(60)  # 60Hz para coincidir con el juego
        self.running = True
        self.current_key = None
        self.key_pressed = False

    def get_key_nonblocking(self):
        """Captura una tecla sin bloquear"""
        if sys.stdin.isatty():
            try:
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1) if sys.stdin.readable() else None
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch
            except:
                return None
        return None

    def key_reader(self):
        """Thread que lee constantemente las teclas presionadas"""
        rospy.loginfo("Control node is running. Use arrow keys to move and spacebar to shoot.")
        rospy.loginfo("Press 'q' to quit")
        
        while self.running and not rospy.is_shutdown():
            key = self.get_key_nonblocking()
            
            if key == '\x1b':  # Código escape para teclas especiales
                arrows = self.get_key_nonblocking() + self.get_key_nonblocking()
                if arrows == '[D':  # Flecha izquierda
                    self.current_key = "LEFT"
                    self.key_pressed = True
                elif arrows == '[C':  # Flecha derecha
                    self.current_key = "RIGHT"
                    self.key_pressed = True
            elif key == ' ':  # Espacio
                self.current_key = "SHOOT"
                self.key_pressed = True
            elif key == 'q':  # Tecla para salir
                self.running = False
                rospy.signal_shutdown("User requested quit")
            elif key is None and self.key_pressed:  # Tecla soltada
                self.key_pressed = False
                self.current_key = None
            
            time.sleep(0.01)  # Pequeña pausa para no saturar el CPU

    def run(self):
        # Iniciar thread para leer teclas
        key_thread = threading.Thread(target=self.key_reader)
        key_thread.daemon = True
        key_thread.start()

        # Loop principal
        while self.running and not rospy.is_shutdown():
            if self.key_pressed and self.current_key:
                self.control_pub.publish(self.current_key)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ControlNode()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Restaurar configuración de terminal
        if sys.stdin.isatty():
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, 
                            termios.tcgetattr(sys.stdin.fileno()))