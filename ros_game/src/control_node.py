#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import termios
import tty
import threading

class ControlNode:
    def __init__(self):
        rospy.init_node('control_node')
        self.control_pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        self.rate = rospy.Rate(60)  # 60Hz para coincidir con el juego
        self.running = True

    def get_key(self):
        """Captura una tecla sin necesidad de Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def key_reader(self):
        """Thread que lee constantemente las teclas presionadas"""
        rospy.loginfo("Control node is running. Use arrow keys to move and spacebar to shoot.")
        rospy.loginfo("Press 'q' to quit")
        
        while self.running and not rospy.is_shutdown():
            key = self.get_key()
            
            if key == '\x1b':  # Código escape para teclas especiales
                # Lee los siguientes dos caracteres para determinar la tecla
                key = sys.stdin.read(2)
                if key == '[D':  # Flecha izquierda
                    self.control_pub.publish("LEFT")
                elif key == '[C':  # Flecha derecha
                    self.control_pub.publish("RIGHT")
            elif key == ' ':  # Espacio
                self.control_pub.publish("SHOOT")
            elif key == 'q':  # Tecla para salir
                self.running = False
                rospy.signal_shutdown("User requested quit")
            else:
                self.control_pub.publish("STOP")  # Añadido para detener el movimiento

    def run(self):
        # Iniciar thread para leer teclas
        key_thread = threading.Thread(target=self.key_reader)
        key_thread.daemon = True
        key_thread.start()

        # Loop principal
        while self.running and not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ControlNode()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Restaurar configuración de terminal
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)