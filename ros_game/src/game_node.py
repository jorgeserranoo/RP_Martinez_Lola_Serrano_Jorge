#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int64
from ros_game.msg import user_msg
import pygame
import time

class GameNode:
    def __init__(self):
        rospy.init_node('game_node')
        
        # Subscribers
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.control_sub = rospy.Subscriber('keyboard_control', String, self.control_callback)
        
        # Publisher para resultados
        self.result_pub = rospy.Publisher('result_information', Int64, queue_size=10)
        
        # Variables de estado
        self.phase = "WELCOME"  # WELCOME, GAME, FINAL
        self.user_info = None
        self.score = 0
        self.current_control = None
        
        # Inicializar Pygame para el juego
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("Galaga")
        self.clock = pygame.time.Clock()
        
        # Cargar recursos del juego
        self.player_img = pygame.image.load("nave_galaga.png")
        self.player_img = pygame.transform.scale(self.player_img, (50, 50))
        self.logo_img = pygame.image.load("logo_galaga.png")
        self.logo_img = pygame.transform.scale(self.logo_img, (400, 200))
        self.retro_font = pygame.font.Font("nombre_de_tu_archivo.ttf", 36)
        self.player_x = 375  # Centrado
        
        # Configuración del juego
        self.running = True

    def user_callback(self, data):
        if self.phase == "WELCOME":
            self.user_info = data
            rospy.loginfo(f"Welcome {data.name}!")
            time.sleep(2)  # Dar tiempo para leer el mensaje
            self.phase = "GAME"

    def control_callback(self, data):
        if self.phase == "GAME":
            self.current_control = data.data
            self.handle_movement()

    def handle_movement(self):
        if self.current_control == "LEFT" and self.player_x > 0:
            self.player_x -= 5
        elif self.current_control == "RIGHT" and self.player_x < 750:  # 800 - 50 (ancho jugador)
            self.player_x += 5
        elif self.current_control == "SHOOT":
            # Aquí iría la lógica de disparo
            pass

    def run(self):
        while not rospy.is_shutdown() and self.running:
            # Manejo de eventos de Pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            # Renderizado según la fase
            if self.phase == "WELCOME":
                self.render_welcome()
            elif self.phase == "GAME":
                self.render_game()
            elif self.phase == "FINAL":
                self.render_final()

            pygame.display.flip()
            self.clock.tick(60)

    def render_welcome(self):
        self.screen.fill((0, 0, 0))
        # Mostrar logo
        logo_rect = self.logo_img.get_rect(center=(400, 150))
        self.screen.blit(self.logo_img, logo_rect)
        
        if self.user_info:
            text = self.retro_font.render(f"Welcome {self.user_info.name}!", True, (255, 255, 255))
            text_rect = text.get_rect(center=(400, 400))
            self.screen.blit(text, text_rect)

    def render_game(self):
        self.screen.fill((0, 0, 0))
        self.screen.blit(self.player_img, (self.player_x, 500))
        # Aquí irían los enemigos y disparos

    def render_final(self):
        self.screen.fill((0, 0, 0))
        font = pygame.font.Font(None, 36)
        text = font.render(f"Final Score: {self.score}", True, (255, 255, 255))
        self.screen.blit(text, (300, 250))
        # Publicar score final
        self.result_pub.publish(self.score)

if __name__ == '__main__':
    try:
        game = GameNode()
        game.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()