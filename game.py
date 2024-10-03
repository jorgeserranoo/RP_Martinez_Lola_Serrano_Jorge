import pygame
import random
import math

# Inicializar Pygame
pygame.init()

# Configuración de la pantalla
ANCHO = 800
ALTO = 600
pantalla = pygame.display.set_mode((ANCHO, ALTO))
pygame.display.set_caption("Galaga")

# Colores
NEGRO = (0, 0, 0)
BLANCO = (255, 255, 255)
MORADO = (128, 0, 128)
ROJO = (255, 0, 0)

# Cargar el logo de Galaga
logo_galaga = pygame.image.load("logo_galaga.png")
logo_galaga = pygame.transform.scale(logo_galaga, (400, 200))  # Ajusta el tamaño según sea necesario

# Fuentes
fuente_normal = pygame.font.Font(None, 36)
fuente_retro = pygame.font.Font(None, 36)  # Puedes usar una fuente pixelada si la tienes

# Función para mostrar texto normal
def mostrar_texto(texto, x, y):
    superficie = fuente_normal.render(texto, True, BLANCO)
    pantalla.blit(superficie, (x, y))

# Función para texto retro
def texto_retro(texto, fuente, color, x, y):
    superficie = fuente.render(texto, True, color)
    rect = superficie.get_rect()
    rect.center = (x, y)
    return superficie, rect

def pantalla_bienvenida():
    pantalla.fill(NEGRO)
    
    # Mostrar el logo de Galaga
    logo_rect = logo_galaga.get_rect(center=(ANCHO // 2, ALTO // 3))
    pantalla.blit(logo_galaga, logo_rect)
    
    # Texto parpadeante
    texto, texto_rect = texto_retro("Pulsa cualquier tecla para comenzar", fuente_retro, BLANCO, ANCHO // 2, ALTO * 3 // 4)
    
    esperando = True
    reloj = pygame.time.Clock()
    tiempo_inicio = pygame.time.get_ticks()
    
    while esperando:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                return False
            if evento.type == pygame.KEYUP:
                return True
        
        # Efecto de parpadeo
        tiempo_actual = pygame.time.get_ticks()
        if (tiempo_actual - tiempo_inicio) % 1000 < 500:
            pantalla.blit(texto, texto_rect)
        
        # Efecto de scanline
        for y in range(0, ALTO, 4):
            pygame.draw.line(pantalla, (20, 20, 20), (0, y), (ANCHO, y))
        
        pygame.display.flip()
        reloj.tick(60)
    
    return True

# Jugador
jugador_imagen = pygame.image.load("nave_galaga.png")
jugador_ancho = 50
jugador_alto = 50
jugador_imagen = pygame.transform.scale(jugador_imagen, (jugador_ancho, jugador_alto))
jugador_x = ANCHO // 2 - jugador_ancho // 2
jugador_y = ALTO - jugador_alto - 10
jugador_velocidad = 5

# Enemigos
enemigos_estaticos = []
enemigos_moviles = []
enemigos_caida = []
enemigo_ancho = 40
enemigo_alto = 40
enemigo_velocidad = 2

# Disparos
disparos = []
disparo_ancho = 5
disparo_alto = 15
disparo_velocidad = 7

# Disparos enemigos
disparos_enemigos = []
disparo_enemigo_velocidad = 5

# Puntuación
puntuacion = 0
fuente = pygame.font.Font(None, 36)

# Reloj para controlar FPS
reloj = pygame.time.Clock()

def crear_enemigo_estatico():
    x = random.randint(0, ANCHO - enemigo_ancho)
    y = random.randint(50, ALTO // 3)
    return {'x': x, 'y': y, 'tiempo_disparo': 0}

def crear_enemigo_movil():
    y = random.randint(50, ALTO // 2)
    return {'x': 0, 'y': y, 'direccion': 1}

def crear_enemigo_caida():
    x = random.randint(0, ANCHO - enemigo_ancho)
    return {'x': x, 'y': 0}

def mover_enemigos():
    # Mover enemigos móviles
    for enemigo in enemigos_moviles:
        enemigo['x'] += enemigo_velocidad * enemigo['direccion']
        if enemigo['x'] <= 0 or enemigo['x'] >= ANCHO - enemigo_ancho:
            enemigo['direccion'] *= -1

    # Mover enemigos en caída
    for enemigo in enemigos_caida:
        enemigo['y'] += enemigo_velocidad
        if enemigo['y'] > ALTO:
            enemigos_caida.remove(enemigo)

def disparar_enemigos():
    for enemigo in enemigos_estaticos:
        enemigo['tiempo_disparo'] += 1
        if enemigo['tiempo_disparo'] >= 60:  # Dispara cada 60 frames (aprox. 1 segundo)
            disparos_enemigos.append([enemigo['x'] + enemigo_ancho // 2, enemigo['y'] + enemigo_alto])
            enemigo['tiempo_disparo'] = 0

def mover_disparos_enemigos():
    for disparo in disparos_enemigos:
        disparo[1] += disparo_enemigo_velocidad
        if disparo[1] > ALTO:
            disparos_enemigos.remove(disparo)

def dibujar_enemigos():
    for enemigo in enemigos_estaticos:
        pygame.draw.rect(pantalla, (255, 0, 0), (enemigo['x'], enemigo['y'], enemigo_ancho, enemigo_alto))
    for enemigo in enemigos_moviles:
        pygame.draw.rect(pantalla, (0, 255, 0), (enemigo['x'], enemigo['y'], enemigo_ancho, enemigo_alto))
    for enemigo in enemigos_caida:
        pygame.draw.rect(pantalla, (0, 0, 255), (enemigo['x'], enemigo['y'], enemigo_ancho, enemigo_alto))
    for disparo in disparos_enemigos:
        pygame.draw.rect(pantalla, (255, 255, 0), (disparo[0], disparo[1], disparo_ancho, disparo_alto))

def mover_disparos():
    for disparo in disparos:
        disparo[1] -= disparo_velocidad
        if disparo[1] < 0:
            disparos.remove(disparo)

# Modificar la función juego()
def juego():
    global jugador_x, jugador_y, puntuacion
    
    jugando = True
    while jugando:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                return False
            if evento.type == pygame.KEYDOWN:
                if evento.key == pygame.K_SPACE:
                    disparos.append([jugador_x + jugador_ancho // 2 - disparo_ancho // 2, jugador_y])

        # Movimiento del jugador
        teclas = pygame.key.get_pressed()
        if teclas[pygame.K_LEFT] and jugador_x > 0:
            jugador_x -= jugador_velocidad
        if teclas[pygame.K_RIGHT] and jugador_x < ANCHO - jugador_ancho:
            jugador_x += jugador_velocidad

        # Generar enemigos
        if random.randint(1, 200) == 1:
            enemigos_estaticos.append(crear_enemigo_estatico())
        if random.randint(1, 150) == 1:
            enemigos_moviles.append(crear_enemigo_movil())
        if random.randint(1, 100) == 1:
            enemigos_caida.append(crear_enemigo_caida())

        # Mover enemigos y disparos
        mover_enemigos()
        mover_disparos()
        disparar_enemigos()
        mover_disparos_enemigos()

        # Detectar colisiones
        for disparo in disparos:
            for enemigo_lista in [enemigos_estaticos, enemigos_moviles, enemigos_caida]:
                for enemigo in enemigo_lista:
                    if (disparo[0] < enemigo['x'] + enemigo_ancho and
                        disparo[0] + disparo_ancho > enemigo['x'] and
                        disparo[1] < enemigo['y'] + enemigo_alto and
                        disparo[1] + disparo_alto > enemigo['y']):
                        disparos.remove(disparo)
                        enemigo_lista.remove(enemigo)
                        puntuacion += 10

        for disparo in disparos_enemigos:
            if (disparo[0] < jugador_x + jugador_ancho and
                disparo[0] + disparo_ancho > jugador_x and
                disparo[1] < jugador_y + jugador_alto and
                disparo[1] + disparo_alto > jugador_y):
                return False

        for enemigo_lista in [enemigos_estaticos, enemigos_moviles, enemigos_caida]:
            for enemigo in enemigo_lista:
                if (jugador_x < enemigo['x'] + enemigo_ancho and
                    jugador_x + jugador_ancho > enemigo['x'] and
                    jugador_y < enemigo['y'] + enemigo_alto and
                    jugador_y + jugador_alto > enemigo['y']):
                    return False

        # Dibujar todo
        pantalla.fill(NEGRO)
        pantalla.blit(jugador_imagen, (jugador_x, jugador_y))
        dibujar_enemigos()
        for disparo in disparos:
            pygame.draw.rect(pantalla, BLANCO, (disparo[0], disparo[1], disparo_ancho, disparo_alto))
        
        mostrar_texto(f"Puntuación: {puntuacion}", 10, 10)
        
        pygame.display.flip()
        reloj.tick(60)  # 60 FPS
    
    return True

# Pantalla de fin de juego
def pantalla_fin_juego():
    pantalla.fill(NEGRO)
    mostrar_texto(f"Juego terminado. Puntuación final: {puntuacion}", ANCHO // 2 - 200, ALTO // 2 - 50)
    mostrar_texto("Presiona R para reiniciar o Q para salir", ANCHO // 2 - 200, ALTO // 2 + 50)
    pygame.display.flip()
    esperando = True
    while esperando:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                return False
            if evento.type == pygame.KEYUP:
                if evento.key == pygame.K_r:
                    return True
                elif evento.key == pygame.K_q:
                    return False
    return False

# Bucle principal
def main():
    global puntuacion
    jugando = True
    while jugando:
        if not pantalla_bienvenida():
            break
        puntuacion = 0
        if not juego():
            if not pantalla_fin_juego():
                break
        else:
            break

    pygame.quit()

if __name__ == "__main__":
    main()