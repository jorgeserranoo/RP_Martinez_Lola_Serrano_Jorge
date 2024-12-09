#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int64
from ros_game.msg import user_msg
import pygame
import random
import time
import rospkg
# At the top of game_node.py, add these imports
from RP_Martinez_Lola_Serrano_Jorge.srv import GetUserScore, GetUserScoreResponse
from RP_Martinez_Lola_Serrano_Jorge.srv import SetGameDifficulty, SetGameDifficultyResponse

class GameNode:
    def __init__(self):
        rospy.init_node('game_node')
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('ros_game')

        # ROS setup
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.control_sub = rospy.Subscriber('keyboard_control', String, self.control_callback)
        self.result_pub = rospy.Publisher('result_information', Int64, queue_size=10)
        
        self.user_name = rospy.get_param("user_name", "default")
        self.change_player_color = rospy.get_param("change_player_color", 1)  # Default: red
        self.screen_param = rospy.get_param("screen_param", "phase1")
        
        self.score_service = rospy.Service('user_score', GetUserScore, self.handle_get_user_score)
        self.difficulty_service = rospy.Service('difficulty', SetGameDifficulty, self.handle_set_difficulty)
        
        self.difficylty = rospy.get_param("~difficulty", 'easy')
        
        
        # Pygame setup
        pygame.init()
        self.ANCHO = 800
        self.ALTO = 600
        self.screen = pygame.display.set_mode((self.ANCHO, self.ALTO))
        pygame.display.set_caption("Galaga")
        
        # Cargar recursos
        self.logo_img = pygame.image.load(f"{package_path}/src/RP_Martinez_Lola_Serrano_Jorge/resources/logo_galaga.png")
        self.logo_img = pygame.transform.scale(self.logo_img, (400, 200))
        self.player_img = pygame.image.load(f"{package_path}/src/RP_Martinez_Lola_Serrano_Jorge/resources/nave_galaga.png")
        self.player_img = pygame.transform.scale(self.player_img, (50, 50))
        
        # Fuentes
        self.font_normal = pygame.font.Font(None, 36)
        self.font_retro = pygame.font.Font(None, 36)
        
        # Variables del juego
        self.phase = "WELCOME"
        self.user_info = None
        self.score = 0
        self.nivel_actual = 1
        self.enemigos_por_nivel = 3
        
        # Jugador
        self.player_x = self.ANCHO // 2 - 25
        self.player_y = self.ALTO - 60
        self.player_speed = 5
        
        # Elementos del juego
        self.enemigos_estaticos = []
        self.enemigos_moviles = []
        self.enemigos_caida = []
        self.disparos = []
        self.disparos_enemigos = []
        
        # Configuración
        self.enemigo_ancho = 40
        self.enemigo_alto = 40
        self.enemigo_velocidad = 2
        self.disparo_ancho = 5
        self.disparo_alto = 15
        self.disparo_velocidad = 7
        
        self.clock = pygame.time.Clock()
        self.running = True

        self.can_shoot = True
        self.shoot_cooldown = 0.5
        self.last_shoot_time = 0


    def handle_get_user_score(self, req):
        """
        Service handler for GetUserScore
        req.username: The username to check score for
        Returns: Score as a percentage
        """
        if req.username == self.user_name:
            # Convert current score to percentage
            # Since we don't have a max score defined, let's use a base value
            # You can adjust this calculation based on your game's scoring system
            max_score = 1000  # Example maximum score
            score_percentage = (self.score / max_score) * 100
            return GetUserScoreResponse(int(score_percentage))
        return GetUserScoreResponse(0)  # Return 0 if username doesn't match
    
    def handle_set_difficulty(self, req):
        """
        Service handler for SetGameDifficulty
        req.change_difficulty: Requested difficulty level ("easy", "medium", "hard")
        Returns: True if change successful, False otherwise
        """
        # Can only change difficulty in WELCOME phase (phase1)
        if self.phase != "WELCOME":
            return SetGameDifficultyResponse(False)

        # Validate requested difficulty
        if req.change_difficulty.lower() in ["easy", "medium", "hard"]:
            self.difficulty = req.change_difficulty.lower()

            # Adjust game parameters based on difficulty
            if self.difficulty == "easy":
                self.enemigos_por_nivel = 2  # Fewer enemies per level
                self.enemigo_velocidad = 1   # Slower enemies
            elif self.difficulty == "medium":
                self.enemigos_por_nivel = 3  # Default number of enemies
                self.enemigo_velocidad = 2   # Default speed
            else:  # hard
                self.enemigos_por_nivel = 4  # More enemies
                self.enemigo_velocidad = 3   # Faster enemies

            return SetGameDifficultyResponse(True)
        return SetGameDifficultyResponse(False)
    
    
    
    def user_callback(self, data):
        if self.phase == "WELCOME":
            self.user_info = data
            rospy.loginfo(f"Welcome {data.name}!")

    def control_callback(self, data):
        if self.phase == "GAME":
            if data.data == "LEFT" and self.player_x > 0:
                self.player_x -= self.player_speed
            elif data.data == "RIGHT" and self.player_x < self.ANCHO - 50:
                self.player_x += self.player_speed
            elif data.data == "SHOOT":
                current_time = time.time()
                if current_time - self.last_shoot_time > self.shoot_cooldown:
                    self.disparos.append([self.player_x + 25 - self.disparo_ancho//2, self.player_y])
                    self.last_shoot_time = current_time
    
    def manejar_disparos_enemigos(self):

        for enemigo in self.enemigos_estaticos:
            enemigo['tiempo_disparo'] = enemigo.get('tiempo_disparo', 0) + 1
            if enemigo['tiempo_disparo'] >= 180:
                if random.random() < 0.5:
                    self.disparos_enemigos.append([enemigo['x'] + self.enemigo_ancho//2, enemigo['y'] + self.enemigo_alto])
                    enemigo['tiempo_disparo'] = 0

        for enemigo in self.enemigos_caida:
            if enemigo['estado'] == 'disparando':
                enemigo['tiempo_disparo'] = enemigo.get('tiempo_disparo', 0) + 1
                if enemigo['tiempo_disparo'] >= 180:
                    if random.random() < 0.5:
                        self.disparos_enemigos.append([enemigo['x'] + self.enemigo_ancho//2, enemigo['y'] + self.enemigo_alto])
                        enemigo['tiempo_disparo'] = 0

    def crear_enemigos_para_nivel(self):
        self.enemigos_estaticos.clear()
        self.enemigos_moviles.clear()
        self.enemigos_caida.clear()
        
        total_enemigos = self.enemigos_por_nivel * self.nivel_actual
        todos_enemigos = []
        
        for _ in range(total_enemigos):
            tipo = random.randint(1, 3)
            if tipo == 1 and self.nivel_actual > 2:
                enemigo = self.crear_enemigo_estatico(todos_enemigos)
                if enemigo:
                    self.enemigos_estaticos.append(enemigo)
                    todos_enemigos.append(enemigo)
            elif tipo == 2:
                enemigo = self.crear_enemigo_movil(todos_enemigos)
                if enemigo:
                    self.enemigos_moviles.append(enemigo)
                    todos_enemigos.append(enemigo)
            else:
                enemigo = self.crear_enemigo_caida(todos_enemigos)
                if enemigo:
                    self.enemigos_caida.append(enemigo)
                    todos_enemigos.append(enemigo)

    def posicion_valida(self, x, y, lista_enemigos):
        for enemigo in lista_enemigos:
            if abs(enemigo['x'] - x) < self.enemigo_ancho and abs(enemigo['y'] - y) < self.enemigo_alto:
                return False
        return True

    def crear_enemigo_estatico(self, lista_enemigos):
        for _ in range(100):
            x = random.randint(0, self.ANCHO - self.enemigo_ancho)
            y = random.randint(50, self.ALTO // 3)
            if self.posicion_valida(x, y, lista_enemigos):
                return {'x': x, 'y': y, 'tiempo_disparo': 0}
        return None

    def crear_enemigo_movil(self, lista_enemigos):
        for _ in range(100):
            y = random.randint(50, self.ALTO // 2)
            x = random.choice([0, self.ANCHO - self.enemigo_ancho])
            if self.posicion_valida(x, y, lista_enemigos):
                return {'x': x, 'y': y, 'direccion': 1 if x == 0 else -1}
        return None

    def crear_enemigo_caida(self, lista_enemigos):
        for _ in range(100):
            x = random.randint(0, self.ANCHO - self.enemigo_ancho)
            if self.posicion_valida(x, 0, lista_enemigos):
                return {'x': x, 'y': 0, 'estado': 'cayendo', 'tiempo_disparo': 0}
        return None

    def mover_elementos(self):
        # Mover enemigos móviles
        for enemigo in self.enemigos_moviles:
            enemigo['x'] += self.enemigo_velocidad * enemigo['direccion']
            if enemigo['x'] <= 0 or enemigo['x'] >= self.ANCHO - self.enemigo_ancho:
                enemigo['direccion'] *= -1

        # Mover enemigos en caída
        for enemigo in self.enemigos_caida:
            if enemigo['estado'] == 'cayendo':
                enemigo['y'] += self.enemigo_velocidad
                if enemigo['y'] >= self.ALTO // 2 - self.enemigo_alto:
                    enemigo['y'] = self.ALTO // 2 - self.enemigo_alto
                    enemigo['estado'] = 'disparando'

        # Mover disparos
        for disparo in self.disparos[:]:
            disparo[1] -= self.disparo_velocidad
            if disparo[1] < 0:
                self.disparos.remove(disparo)

        # Mover disparos enemigos
        for disparo in self.disparos_enemigos[:]:
            disparo[1] += self.disparo_velocidad
            if disparo[1] > self.ALTO:
                self.disparos_enemigos.remove(disparo)

    def check_colisiones(self):
        # Colisiones de disparos con enemigos
        for disparo in self.disparos[:]:
            for enemigo_lista in [self.enemigos_estaticos, self.enemigos_moviles, self.enemigos_caida]:
                for enemigo in enemigo_lista[:]:
                    if (disparo[0] < enemigo['x'] + self.enemigo_ancho and
                        disparo[0] + self.disparo_ancho > enemigo['x'] and
                        disparo[1] < enemigo['y'] + self.enemigo_alto and
                        disparo[1] + self.disparo_alto > enemigo['y']):
                        if disparo in self.disparos:
                            self.disparos.remove(disparo)
                        enemigo_lista.remove(enemigo)
                        self.score += 10

        # Colisiones con el jugador
        for disparo in self.disparos_enemigos:
            if (disparo[0] < self.player_x + 50 and
                disparo[0] + self.disparo_ancho > self.player_x and
                disparo[1] < self.player_y + 50 and
                disparo[1] + self.disparo_alto > self.player_y):
                self.phase = "FINAL"

        for enemigo_lista in [self.enemigos_estaticos, self.enemigos_moviles, self.enemigos_caida]:
            for enemigo in enemigo_lista:
                if (self.player_x < enemigo['x'] + self.enemigo_ancho and
                    self.player_x + 50 > enemigo['x'] and
                    self.player_y < enemigo['y'] + self.enemigo_alto and
                    self.player_y + 50 > enemigo['y']):
                    self.phase = "FINAL"

    def render(self):
    # Fill background with black
        self.screen.fill((0, 0, 0))

        if self.phase == "WELCOME":
            # Update screen parameter to show we're in welcome phase
            rospy.set_param("screen_param", "phase1")

            # Display logo in center of screen
            logo_rect = self.logo_img.get_rect(center=(self.ANCHO//2, self.ALTO//3))
            self.screen.blit(self.logo_img, logo_rect)

            # Show welcome message if user info is available
            if self.user_info:
                text = self.font_retro.render(f"Welcome {self.user_info.name}!", True, (255, 255, 255))
                text_rect = text.get_rect(center=(self.ANCHO//2, self.ALTO*2//3))
                self.screen.blit(text, text_rect)

                # Transition to game phase after 3 seconds
                if pygame.time.get_ticks() > 3000:
                    self.phase = "GAME"
                    self.crear_enemigos_para_nivel()

        elif self.phase == "GAME":
            # Update screen parameter to show we're in game phase
            rospy.set_param("screen_param", "phase2")

            # Draw player
            self.screen.blit(self.player_img, (self.player_x, self.player_y))

            # Get current enemy color scheme based on parameter
            enemy_colors = self.get_enemy_colors()

            # Draw all static enemies with their color
            for enemigo in self.enemigos_estaticos:
                pygame.draw.rect(self.screen, enemy_colors[0], 
                               (enemigo['x'], enemigo['y'], 
                                self.enemigo_ancho, self.enemigo_alto))

            # Draw all mobile enemies with their color
            for enemigo in self.enemigos_moviles:
                pygame.draw.rect(self.screen, enemy_colors[1], 
                               (enemigo['x'], enemigo['y'], 
                                self.enemigo_ancho, self.enemigo_alto))

            # Draw all falling enemies with their color
            for enemigo in self.enemigos_caida:
                pygame.draw.rect(self.screen, enemy_colors[2], 
                               (enemigo['x'], enemigo['y'], 
                                self.enemigo_ancho, self.enemigo_alto))

            # Draw player shots
            for disparo in self.disparos:
                pygame.draw.rect(self.screen, (255, 255, 255),  # White shots
                               (disparo[0], disparo[1], 
                                self.disparo_ancho, self.disparo_alto))

            # Draw enemy shots
            for disparo in self.disparos_enemigos:
                pygame.draw.rect(self.screen, (255, 255, 255),  # White shots
                               (disparo[0], disparo[1], 
                                self.disparo_ancho, self.disparo_alto))

            # Display score and level
            score_text = self.font_normal.render(f"Score: {self.score}", True, (255, 255, 255))
            level_text = self.font_normal.render(f"Level: {self.nivel_actual}", True, (255, 255, 255))
            self.screen.blit(score_text, (10, 10))
            self.screen.blit(level_text, (self.ANCHO - 100, 10))

            # Update game elements
            self.mover_elementos()
            self.manejar_disparos_enemigos()
            self.check_colisiones()

            # Check if level is complete
            if not self.enemigos_estaticos and not self.enemigos_moviles and not self.enemigos_caida:
                self.nivel_actual += 1
                self.crear_enemigos_para_nivel()

        elif self.phase == "FINAL":
            # Update screen parameter to show we're in final phase
            rospy.set_param("screen_param", "phase3")

            # Display game over message and final score
            text = self.font_retro.render(f"Game Over - Final Score: {self.score}", True, (255, 255, 255))
            text_rect = text.get_rect(center=(self.ANCHO//2, self.ALTO//2))
            self.screen.blit(text, text_rect)

            # Publish final score and shutdown
            self.result_pub.publish(self.score)
            rospy.signal_shutdown("Game Over")

        # Update display
        pygame.display.flip()
        
    def get_enemy_colors(self):
        """
        Returns a tuple of three RGB colors based on the change_player_color parameter
        Each tuple contains three RGB color values, one for each type of enemy
        """
        # Color scheme 1: Yellow, Green, Pink
        if self.change_player_color == 1:
            return [
                (255, 0, 0),    # Red for static enemies
                (255, 0, 255),      # Purple for mobile enemies
                (0, 0, 255)   # Blue for falling enemies
            ]

        # Color scheme 2: Orange, Red, Blue
        elif self.change_player_color == 2:
            return [
                (255, 165, 0),    # Orange for static enemies
                (255, 0, 0),      # Red for mobile enemies
                (0, 0, 255)       # Blue for falling enemies
            ]

        # Color scheme 3: White, Brown, Gray
        else:  # self.change_player_color == 3
            return [
                (255, 255, 255),  # White for static enemies
                (139, 69, 19),    # Brown for mobile enemies
                (128, 128, 128)   # Gray for falling enemies
            ]

    def run(self):
        while not rospy.is_shutdown() and self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            self.render()
            self.clock.tick(60)

if __name__ == '__main__':
    try:
        game = GameNode()
        game.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()