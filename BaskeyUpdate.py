# By Daniel Peña                                
# Estudiante Ingeniería Civil Informática       
# Universidad Católica de Temuco                

# Importación de módulos necesarios
from vpython import canvas, box, cylinder, sphere, vector, color, rate, ring, label  # VPython para gráficos 3D\ nimport math                                     # Funciones matemáticas (cos, sin, radians, sqrt)
import random as ra                             # Generación de valores aleatorios
import ctypes as ct                             # Definición de estructuras de estilo C en Python
import math
# Parámetros globales
nMAX_ROBOTS = 10                                # Cantidad de robots y pelotas en la simulación
current_hit_label = None                        # Etiqueta del último acierto (actualizable)
'''
================================================================================
'''
# Definición de estructuras de datos
class eRobot(ct.Structure):                      # Estructura para datos de un robot
    _fields_ = [
        ('nBoId', ct.c_ubyte),   # ID del robot (0-255)
        ('nAngu', ct.c_int),     # Ángulo de dirección en XZ (grados)
        ('nVelo', ct.c_float),   # Velocidad de desplazamiento
        ('nStep', ct.c_int),     # Pasos restantes antes de recalcular
        ('nPryX', ct.c_float),   # Componente X de la velocidad
        ('nPryZ', ct.c_float),   # Componente Z de la velocidad
        ('nPosX', ct.c_float),   # Posición X actual
        ('nPosZ', ct.c_float),   # Posición Z actual
    ]

class ePelo(ct.Structure):                       # Estructura para datos de una pelota
    _fields_ = [
        ('nAccG', ct.c_float),   # Aceleración de la gravedad
        ('nVelo', ct.c_int),     # Velocidad inicial del disparo
        ('nAngE', ct.c_uint),    # Ángulo de elevación (grados)
        ('nAngD', ct.c_int),     # Ángulo horizontal (grados)
        ('nPryY', ct.c_float),   # Componente Y de la velocidad
        ('nVe_X', ct.c_float),   # Velocidad en X
        ('nVe_Y', ct.c_float),   # Velocidad en Y
        ('nVe_Z', ct.c_float),   # Velocidad en Z
        ('nPosX', ct.c_float),   # Posición X de la pelota
        ('nPosY', ct.c_float),   # Posición Y de la pelota
        ('nPosZ', ct.c_float),   # Posición Z de la pelota
        ('nSamp', ct.c_float),   # Tiempo de vuelo transcurrido
        ('nDi_T', ct.c_float),   # Intervalo de muestreo
        ('lSw_T', ct.c_bool)     # Flag de vuelo activo (True=volando)
    ]
'''
================================================================================
'''
# Funciones auxiliares aleatorias

def Get_Angulo_R():                             # Ángulo para robots: múltiplos de 15°
    return ra.choice(range(0, 360, 15))

def Get_Velocidad_R():                          # Velocidades válidas para robots
    return ra.choice([0.0, 1.0, 2.0, 4.0, 5.0, 6.0])

def Get_Angulo_P():                             # Ángulo de elevación para pelotas
    return ra.choice(range(5, 85, 60))

def Get_Angulo_D():                             # Ángulo horizontal para pelotas
    return ra.choice(range(-50, 55, 5))

def Get_Velocidad_P():                          # Velocidad inicial de disparo de pelotas
    return ra.choice(range(10, 140, 10))
'''
================================================================================
'''
# Inicialización de datos globales

aRobots = [eRobot() for _ in range(nMAX_ROBOTS)]  # Array de estructuras de robots
aPelota = [ePelo() for _ in range(nMAX_ROBOTS)]  # Array de estructuras de pelotas
'''
================================================================================
'''
# Inicializa un robot con parámetros aleatorios
def Robot_Init(i):
    r = aRobots[i]                          # Obtener estructura del robot i
    r.nBoId = i                             # Asignar ID
    r.nAngu = Get_Angulo_R()                # Ángulo de movimiento
    r.nVelo = Get_Velocidad_R()             # Velocidad de movimiento
    r.nStep = ra.randint(10, 200)           # Pasos antes de cambiar de dirección
    # Cálculo de componentes de velocidad
    r.nPryX = r.nVelo * math.cos(math.radians(r.nAngu))
    r.nPryZ = r.nVelo * math.sin(math.radians(r.nAngu))
    # Posición inicial igual a la primera proyección
    r.nPosX, r.nPosZ = r.nPryX, r.nPryZ
    r.nTime = ra.randint(5000, 10000)       # Tiempo aleatorio (no usado en bucle actual)
'''
================================================================================
'''
# Inicializa parámetros físicos de todas las pelotas
def Pelotas_Init():
    for p in aPelota:
        p.nAccG = 9.8                      # Gravedad (m/s²)
        p.nVelo = Get_Velocidad_P()        # Velocidad de disparo
        p.nAngE = Get_Angulo_P()           # Ángulo de elevación
        p.nAngD = Get_Angulo_D()           # Ángulo horizontal
        # Componente Y de la proyección inicial
        p.nPryY = p.nVelo * math.cos(math.radians(p.nAngE))
        # Descomposición de velocidad en X, Y, Z
        p.nVe_X = p.nPryY * math.cos(math.radians(p.nAngD))
        p.nVe_Y = p.nVelo * math.sin(math.radians(p.nAngE))
        p.nVe_Z = p.nPryY * math.sin(math.radians(p.nAngD))
        p.nDi_T = 0.1                      # Intervalo de tiempo (segundos)
        p.nSamp = 0.0                      # Tiempo de vuelo inicial
        p.lSw_T = False                    # No está volando
        # Posición tras primer intervalo
        p.nPosX = p.nVe_X * p.nDi_T
        p.nPosY = p.nVe_Y * p.nDi_T - 0.5 * p.nAccG * (p.nDi_T ** 2)
        p.nPosZ = p.nVe_Z * p.nDi_T
'''
================================================================================
'''
# Configuración de la escena 3D y cancha
scene = canvas(
    title='Proyecto #1 Robotica - By Daniel Peña',  # Título ventana
    width=1700, height=700,                         # Dimensiones
    center=vector(0, 100, 100),                     # Centro de cámara
    background=color.black                          # Color de fondo
)
# Suelo y líneas de la cancha
box(pos=vector(0, 0, 0), size=vector(1800, 3, 1000), color=color.red)
box(pos=vector(0, 2, 0), size=vector(1780, 5, 980), color=color.gray(0.5))
box(pos=vector(0, 4, 0), size=vector(3, 3, 1000), color=color.white)
# Canastas y aros
# Derecha (equipo rojo)
box(pos=vector(900, 150, 0), size=vector(4, 70, 140), color=color.red)
box(pos=vector(900, 60, 0), size=vector(3, 120, 6), color=color.white)
ring(pos=vector(874, 130, 0), axis=vector(0, 4, 0), radius=25, thickness=2, color=color.white)
# Izquierda (equipo azul)
box(pos=vector(-900, 150, 0), size=vector(4, 70, 140), color=color.blue)
box(pos=vector(-900, 60, 0), size=vector(3, 120, 6), color=color.white)
ring(pos=vector(-874, 130, 0), axis=vector(0, 4, 0), radius=25, thickness=2, color=color.white)
# Columnas centrales
cylinder(pos=vector(0, 0, 0), radius=50, axis=vector(0, 6, 0), color=color.red)
cylinder(pos=vector(0, 0, 0), radius=20, axis=vector(0, 8, 0), color=color.white)

# Posiciones iniciales para colocar robots
positions = [
    vector(-800, 1, -400), vector(-400, 1, 0), vector(-800, 1, 400),
    vector(-800, 1, 200), vector(-400, 1, 200),
    vector(800, 1, -400), vector(400, 1, 0), vector(800, 1, 400),
    vector(800, 1, -200), vector(800, 1, 200)
]
'''
================================================================================
'''
# Construcción gráfica de robots y etiquetas
global robot_bodies, robot_tops, number_robot
robot_bodies, robot_tops, number_robot = [], [], []
for i, pos in enumerate(positions):
    # Colores según equipo
    body_color = color.blue if i < 5 else color.red
    top_color = color.yellow if i < 5 else color.green
    number_color = color.blue if i < 5 else color.red
    # Crea cuerpo
    body = cylinder(pos=pos, axis=vector(0, 15, 0), radius=15, color=body_color)
    # Crea cabeza
    top = cylinder(pos=pos + vector(0, 13, 0), axis=vector(0, 10, 0), radius=8, color=top_color)
    # Crea etiqueta con número
    number = label(
        pos=pos + vector(0, 45, 0), text=str(i), height=10,
        box=True, color=number_color, billboard=True
    )
    robot_bodies.append(body)
    robot_tops.append(top)
    number_robot.append(number)

# Nombre flotante en cancha
label(pos=vector(0, 100, -600), text='By Daniel Peña - Basket Bot', color=color.green, height=30, box=True)
'''
================================================================================
'''
# Esferas para pelotas (invisibles inicialmente)
ball_spheres = []
for _ in range(nMAX_ROBOTS):
    s = sphere(pos=vector(0, 0, 0), radius=6, color=color.orange)
    s.visible = False
    ball_spheres.append(s)
'''
================================================================================
'''
# Función para mover robots azules (índices 0-4)
def Robot_Blue_Move():
    for i in range(5):
        r = aRobots[i]                      # Estructura del robot azul i
        r.nStep -= 1                        # Decrementa pasos restantes
        if r.nStep <= 0:
            Robot_Init(i)                   # Recalcula trayectoria cuando pasos llegan a 0
        dx, dz = r.nPosX, r.nPosZ          # Desplazamientos X y Z
        # Actualiza posición gráfica
        robot_bodies[i].pos.x += dx
        robot_tops[i].pos.x += dx
        robot_bodies[i].pos.z += dz
        robot_tops[i].pos.z += dz
        # Limita movimiento dentro de la zona blue
        if robot_bodies[i].pos.x <= -870: r.nStep = 0; robot_bodies[i].pos.x = -870; robot_tops[i].pos.x = -870
        if robot_bodies[i].pos.x >= -50:  r.nStep = 0; robot_bodies[i].pos.x = -50;  robot_tops[i].pos.x = -50
        if robot_bodies[i].pos.z <= -470: r.nStep = 0; robot_bodies[i].pos.z = -470; robot_tops[i].pos.z = -470
        if robot_bodies[i].pos.z >= 470:  r.nStep = 0; robot_bodies[i].pos.z = 470;  robot_tops[i].pos.z = 470
        # Mueve la etiqueta
        number_robot[i].pos = robot_bodies[i].pos + vector(0, 45, 0)
'''
================================================================================
'''
# Función para mover robots rojos (índices 5-9)
def Robot_Red_Move():
    for i in range(5, 10):
        r = aRobots[i]
        r.nStep -= 1
        if r.nStep <= 0:
            Robot_Init(i)
        dx, dz = r.nPosX, r.nPosZ
        robot_bodies[i].pos.x += dx
        robot_tops[i].pos.x += dx
        robot_bodies[i].pos.z += dz
        robot_tops[i].pos.z += dz
        # Limita movimiento dentro de la zona red
        if robot_bodies[i].pos.x <= 50:  r.nStep = 0; robot_bodies[i].pos.x = 50;  robot_tops[i].pos.x = 50
        if robot_bodies[i].pos.x >= 870: r.nStep = 0; robot_bodies[i].pos.x = 870; robot_tops[i].pos.x = 870
        if robot_bodies[i].pos.z <= -470:r.nStep = 0; robot_bodies[i].pos.z = -470;robot_tops[i].pos.z = -470
        if robot_bodies[i].pos.z >= 470: r.nStep = 0; robot_bodies[i].pos.z = 470; robot_tops[i].pos.z = 470
        number_robot[i].pos = robot_bodies[i].pos + vector(0, 45, 0)
'''
================================================================================
'''

# Variables para control de disparos
last_shot_time = 0           # Tiempo del último disparo
next_shooter = 0            # Índice del siguiente robot que disparará

# Función para gestionar disparos periódicos de pelotas
def Shot_Ball():
    global last_shot_time, next_shooter
    current_time = t                          # Tiempo simulado actual
    if current_time - last_shot_time < 5.0:   # Espera mínimo de 5 s entre disparos
        return
    # Busca siguiente robot sin pelota en vuelo
    for _ in range(nMAX_ROBOTS):
        i = next_shooter
        next_shooter = (next_shooter + 1) % nMAX_ROBOTS
        if not aPelota[i].lSw_T:              # Si pelota no está volando
            Calcular_Disparo(i)               # Configura y lanza
            last_shot_time = current_time
            break

'''
================================================================================
'''
# Función para calcular y disparar la pelota del robot i
def Calcular_Disparo(i):
    r = robot_bodies[i]                      # Objeto gráfico del robot
    p = aPelota[i]                           # Estructura de la pelota
    origen = r.pos + vector(0, 15, 0)        # Punto de disparo (boca del robot)
    p.origen = origen                        # Guarda origen para actualizar posición
    ball_spheres[i].pos = origen             # Coloca esfera inicial
    # Define objetivo en canasta opuesta
    aro = vector(874, 130, 0) if origen.x < 0 else vector(-874, 130, 0)
    d = aro - origen                         # Vector distancia a canasta
    dx = math.hypot(d.x, d.z)                # Distancia horizontal
    dy = d.y                                # Diferencia vertical
    g = p.nAccG                             # Gravedad
    theta = math.radians(45)                # Ángulo elevación inicial 45°
    denom = dx * math.tan(theta) - dy
    if denom <= 0:                          # Si 45° no alcanza, usar 60°
        theta = math.radians(60)
        denom = dx * math.tan(theta) - dy
    # Velocidad inicial necesaria v0
    v0 = math.sqrt((g * dx ** 2) / (2 * (math.cos(theta) ** 2) * denom))
    # Ángulo horizontal en plano XZ
    phi = math.atan2(d.z, d.x)
    # Descompone v0 en componentes X, Y, Z
    p.nVe_X = v0 * math.cos(theta) * math.cos(phi)
    p.nVe_Y = v0 * math.sin(theta)
    p.nVe_Z = v0 * math.cos(theta) * math.sin(phi)
    ball_spheres[i].visible = True           # Muestra la esfera
    p.nSamp = 0.0                            # Reinicia tiempo de vuelo
    p.lSw_T = True                          # Activa flag de vuelo
'''
================================================================================
'''
# Reinicia y oculta la pelota i tras caer al suelo o anotar
def Reset_Pelota(i):
    p = aPelota[i]
    p.lSw_T = False                         # Desactiva vuelo
    p.nSamp = 0.0                           # Reinicia tiempo
    ball_spheres[i].visible = False         # Oculta esfera
'''
================================================================================
'''
# Actualiza posición de todas las pelotas en vuelo
def UpDate_Pelotas():
    for i, p in enumerate(aPelota):
        if not p.lSw_T:                     # Solo si está en vuelo
            continue
        p.nSamp += p.nDi_T                  # Incrementa tiempo de vuelo
        x = p.nVe_X * p.nSamp               # Posición X
        y = p.nVe_Y * p.nSamp - 0.5 * p.nAccG * p.nSamp ** 2  # Posición Y
        z = p.nVe_Z * p.nSamp               # Posición Z
        ball_spheres[i].pos = p.origen + vector(x, y, z)  # Mueve esfera
        if ball_spheres[i].pos.y < 0:       # Si toca suelo
            Reset_Pelota(i)                # Reinicia pelota
'''
================================================================================
'''
# Verifica acierto al pasar por el aro del robot i
def Verificar_Acierto(i):
    global current_hit_label
    p = aPelota[i]
    if not p.lSw_T:                         # Solo pelotas en vuelo
        return
    pos = ball_spheres[i].pos
    tol_x, tol_y, tol_z = 25, 5, 25         # Tolerancias de acierto
    aro = vector(874, 130, 0) if pos.x > 0 else vector(-874, 130, 0)
    if (abs(pos.x - aro.x) < tol_x and
        abs(pos.y - aro.y) < tol_y and
        abs(pos.z - aro.z) < tol_z):
        print(f"¡Acierto robot {i}!")       # Mensaje en consola
        Reset_Pelota(i)                    # Oculta pelota
        if current_hit_label:              # Borra etiqueta previa
            current_hit_label.visible = False
        color_lbl = color.blue if i < 5 else color.red
        current_hit_label = label(
            pos=vector(0, 300, -600),
            text=f'¡Acierto Robot {i}!',
            color=color_lbl,
            height=30,
            box=True,
            billboard=True
        )
'''
================================================================================
'''
# Inicialización de estados y bucle principal
def initialize():
    for i in range(nMAX_ROBOTS):
        Robot_Init(i)                     # Inicializa robots
    Pelotas_Init()                        # Inicializa pelotas

initialize()                              # Llamada inicial

t = 0                                     # Tiempo simulado
dt = 0.01                                 # Paso de tiempo por frame
'''
================================================================================
'''
while True:
    rate(100)                              # Controla FPS (~100)
    t += dt                                # Incrementa tiempo
    Robot_Blue_Move()                      # Actualiza robots azules
    Robot_Red_Move()                       # Actualiza robots rojos
    UpDate_Pelotas()                       # Actualiza posiciones de pelotas
    for i in range(nMAX_ROBOTS):
        Verificar_Acierto(i)               # Comprueba si anota
    Shot_Ball()                            # Disparos periódicos
