from controller import Robot, Motor, LED, Camera
import math
import sys

NUMBER_OF_LEDS = 8
NUMBER_OF_JOINTS = 12
NUMBER_OF_CAMERAS = 5

robot = Robot()
time_step = int(robot.getBasicTimeStep())

# Nombres de dispositivos
motor_names = [
    "front left shoulder abduction motor", "front left shoulder rotation motor", "front left elbow motor",
    "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
    "rear left shoulder abduction motor", "rear left shoulder rotation motor", "rear left elbow motor",
    "rear right shoulder abduction motor", "rear right shoulder rotation motor", "rear right elbow motor"
]

camera_names = [
    "left head camera", "right head camera",
    "left flank camera", "right flank camera",
    "rear camera"
]

led_names = [
    "left top led", "left middle up led", "left middle down led", "left bottom led",
    "right top led", "right middle up led", "right middle down led", "right bottom led"
]

# Inicialización
motors = [robot.getDevice(name) for name in motor_names]
cameras = [robot.getDevice(name) for name in camera_names]
leds = [robot.getDevice(name) for name in led_names]

# Activar cámaras frontales
cameras[0].enable(2 * time_step)
cameras[1].enable(2 * time_step)

# Encender LEDs
for led in leds:
    led.set(1)

# Función step
def step():
    if robot.step(time_step) == -1:
        sys.exit(0)

# Movimiento suave
def movement_decomposition(target, duration):
    n_steps = int(duration * 1000 / time_step)

    current_position = [m.getTargetPosition() for m in motors]
    step_diff = [(target[i] - current_position[i]) / n_steps for i in range(NUMBER_OF_JOINTS)]

    for _ in range(n_steps):
        for j in range(NUMBER_OF_JOINTS):
            current_position[j] += step_diff[j]
            motors[j].setPosition(current_position[j])
        step()

# Posturas
def lie_down(duration):
    target = [
        -0.40, -0.99, 1.59,
         0.40, -0.99, 1.59,
        -0.40, -0.99, 1.59,
         0.40, -0.99, 1.59
    ]
    movement_decomposition(target, duration)

def stand_up(duration):
    target = [
        -0.1, 0.0, 0.0,
         0.1, 0.0, 0.0,
        -0.1, 0.0, 0.0,
         0.1, 0.0, 0.0
    ]
    movement_decomposition(target, duration)

def sit_down(duration):
    target = [
        -0.20, -0.40, -0.19,
         0.20, -0.40, -0.19,
        -0.40, -0.90, 1.18,
         0.40, -0.90, 1.18
    ]
    movement_decomposition(target, duration)

def give_paw():
    # estabilizar
    target1 = [
        -0.20, -0.30, 0.05,
         0.20, -0.40, -0.19,
        -0.40, -0.90, 1.18,
         0.49, -0.90, 0.80
    ]
    movement_decomposition(target1, 4)

    start_time = robot.getTime()

    while robot.getTime() - start_time < 8:
        motors[4].setPosition(0.2 * math.sin(2 * robot.getTime()) + 0.6)
        motors[5].setPosition(0.4 * math.sin(2 * robot.getTime()))
        step()

    # regresar a sentado
    target2 = [
        -0.20, -0.40, -0.19,
         0.20, -0.40, -0.19,
        -0.40, -0.90, 1.18,
         0.40, -0.90, 1.18
    ]
    movement_decomposition(target2, 4)

# Loop principal
while True:
    lie_down(4.0)
    stand_up(4.0)
    sit_down(4.0)
    give_paw()
    stand_up(4.0)
    lie_down(3.0)
    stand_up(3.0)
    lie_down(2.0)
    stand_up(2.0)
    lie_down(1.0)
    stand_up(1.0)
    lie_down(0.75)
    stand_up(0.75)
    lie_down(0.5)
    stand_up(0.5)
