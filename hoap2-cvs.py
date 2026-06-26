
import math
import sys
import os
from controller import Robot, Motor, PositionSensor, TouchSensor, Emitter

# Mapeo de articulaciones (Reemplazo del enum de C)
(
    body_joint_1,
    lleg_joint_1, lleg_joint_3, lleg_joint_2, lleg_joint_4, lleg_joint_5, lleg_joint_6,
    rleg_joint_1, rleg_joint_3, rleg_joint_2, rleg_joint_4, rleg_joint_5, rleg_joint_6,
    larm_joint_1, larm_joint_2, larm_joint_3, larm_joint_4, larm_joint_5,
    rarm_joint_1, rarm_joint_2, rarm_joint_3, rarm_joint_4, rarm_joint_5,
    head_joint_2, head_joint_1
) = range(25)

# Array que mapea el ID numérico con el nombre del string de la articulación
JOINT_NAMES = [
    "body_joint_1",
    "lleg_joint_1", "lleg_joint_3", "lleg_joint_2", "lleg_joint_4", "lleg_joint_5", "lleg_joint_6",
    "rleg_joint_1", "rleg_joint_3", "rleg_joint_2", "rleg_joint_4", "rleg_joint_5", "rleg_joint_6",
    "larm_joint_1", "larm_joint_2", "larm_joint_3", "larm_joint_4", "larm_joint_5",
    "rarm_joint_1", "rarm_joint_2", "rarm_joint_3", "rarm_joint_4", "rarm_joint_5",
    "head_joint_2", "head_joint_1"
]

def main():
    # Inicializar Webots
    robot = Robot()

    # Obtener los argumentos pasados al controlador (sys.argv)
    # Nota: En Webots Python, sys.argv[0] es la ruta del script. Los argumentos reales empiezan en sys.argv[1]
    if len(sys.argv) > 1 and sys.argv[1] == "sumo":
        filename = "sumo.csv"
        control_step = 64
    else:
        filename = "walk.csv"
        control_step = 50

    # Inicializar listas para motores y sensores
    joint_motors = [None] * 25
    joint_sensors = [None] * 25

    for i in range(25):
        joint_motors[i] = robot.getDevice(JOINT_NAMES[i])
        joint_sensors[i] = joint_motors[i].getPositionSensor()
        joint_sensors[i].enable(control_step)

    # Dispositivos adicionales
    left_touch = robot.getDevice("left touch")
    right_touch = robot.getDevice("right touch")
    left_touch.enable(control_step)
    right_touch.enable(control_step)
    
    gps = robot.getDevice("gps")
    emitter = robot.getDevice("emitter")

    # Intentar abrir el archivo CSV
    try:
        file = open(filename, "r")
    except FileNotFoundError:
        print(f"unable to locate the {filename} file")
        return 1

    # Estructuras de datos equivalentes a los arrays en C
    pulse = [
        209,  209, 209,  209, -209, -209, -209, 209,  -209, 209, 209, 209, -209,
       -209, 209, -209, 209, 0,    209,  209,  -209, -209, 0,   0,   0
    ]
    
    motor_position = [0.0] * 25
    next_position = [0.0] * 25
    temp_motor = [0] * 25

    # Los dedos no se usan inicialmente
    temp_motor[larm_joint_5] = 0
    temp_motor[rarm_joint_5] = 0
    next_position[larm_joint_5] = 0.0
    next_position[rarm_joint_5] = 0.0

    # Leer la primera línea
    first_line = file.readline()
    if not first_line:
        print(f"Error while reading the {filename} file", file=sys.stderr)
        file.close()
        return 1

    # Procesar la primera línea (Simulación del sscanf en C)
    # El archivo original tiene este orden en los datos:
    # sampling, (ignora una columna), rleg1..6, rarm1..4, lleg1..6, larm1..4, body1
    tokens = [t.strip() for t in first_line.split(',') if t.strip()]
    
    sampling = int(tokens[0])
    temp_motor[rleg_joint_1] = int(tokens[2])
    temp_motor[rleg_joint_2] = int(tokens[3])
    temp_motor[rleg_joint_3] = int(tokens[4])
    temp_motor[rleg_joint_4] = int(tokens[5])
    temp_motor[rleg_joint_5] = int(tokens[6])
    temp_motor[rleg_joint_6] = int(tokens[7])
    temp_motor[rarm_joint_1] = int(tokens[8])
    temp_motor[rarm_joint_2] = int(tokens[9])
    temp_motor[rarm_joint_3] = int(tokens[10])
    temp_motor[rarm_joint_4] = int(tokens[11])
    temp_motor[lleg_joint_1] = int(tokens[12])
    temp_motor[lleg_joint_2] = int(tokens[13])
    temp_motor[lleg_joint_3] = int(tokens[14])
    temp_motor[lleg_joint_4] = int(tokens[15])
    temp_motor[lleg_joint_5] = int(tokens[16])
    temp_motor[lleg_joint_6] = int(tokens[17])
    temp_motor[larm_joint_1] = int(tokens[18])
    temp_motor[larm_joint_2] = int(tokens[19])
    temp_motor[larm_joint_3] = int(tokens[20])
    temp_motor[larm_joint_4] = int(tokens[21])
    temp_motor[body_joint_1] = int(tokens[22])

    # Convertir a radianes
    for i in range(23):
        if pulse[i] != 0:
            motor_position[i] = temp_motor[i] * (math.pi / 180.0) / pulse[i]
        else:
            motor_position[i] = 0.0

    motor_position[larm_joint_5] = 0.0
    motor_position[rarm_joint_5] = 0.0
    com_interval = int(control_step / sampling)

    # Esperar un poco antes de empezar
    robot.step(50 * control_step)

    for i in range(25):
        next_position[i] = joint_sensors[i].getValue()

    file_ended = False

    # Bucle principal infinito del robot
    while True:
        if not file_ended:
            for i in range(com_interval):
                if not file_ended:
                    line = file.readline()
                    if not line:
                        file.close()
                        file_ended = True

            if not file_ended:
                # Leer datos de la línea actual
                tokens = [t.strip() for t in line.split(',') if t.strip()]
                
                # Mapeo idéntico al sscanf del bucle en C
                pos_from_csv = [0] * 25
                pos_from_csv[rleg_joint_1] = int(tokens[2])
                pos_from_csv[rleg_joint_2] = int(tokens[3])
                pos_from_csv[rleg_joint_3] = int(tokens[4])
                pos_from_csv[rleg_joint_4] = int(tokens[5])
                pos_from_csv[rleg_joint_5] = int(tokens[6])
                pos_from_csv[rleg_joint_6] = int(tokens[7])
                pos_from_csv[rarm_joint_1] = int(tokens[8])
                pos_from_csv[rarm_joint_2] = int(tokens[9])
                pos_from_csv[rarm_joint_3] = int(tokens[10])
                pos_from_csv[rarm_joint_4] = int(tokens[11])
                pos_from_csv[lleg_joint_1] = int(tokens[12])
                pos_from_csv[lleg_joint_2] = int(tokens[13])
                pos_from_csv[lleg_joint_3] = int(tokens[14])
                pos_from_csv[lleg_joint_4] = int(tokens[15])
                pos_from_csv[lleg_joint_5] = int(tokens[16])
                pos_from_csv[lleg_joint_6] = int(tokens[17])
                pos_from_csv[larm_joint_1] = int(tokens[18])
                pos_from_csv[larm_joint_2] = int(tokens[19])
                pos_from_csv[larm_joint_3] = int(tokens[20])
                pos_from_csv[larm_joint_4] = int(tokens[21])
                pos_from_csv[body_joint_1] = int(tokens[22])

                # Convertir a radianes
                for i in range(22):
                    if pulse[i] != 0:
                        next_position[i] = pos_from_csv[i] * (math.pi / 180.0) / pulse[i]

            # Asignar los nuevos objetivos de posición a los motores mapeados
            joint_motors[body_joint_1].setPosition(next_position[body_joint_1])
            joint_motors[lleg_joint_1].setPosition(next_position[lleg_joint_1])
            joint_motors[lleg_joint_2].setPosition(next_position[lleg_joint_2])
            joint_motors[lleg_joint_3].setPosition(next_position[lleg_joint_3])
            joint_motors[lleg_joint_4].setPosition(next_position[lleg_joint_4])
            joint_motors[lleg_joint_5].setPosition(next_position[lleg_joint_5])
            joint_motors[lleg_joint_6].setPosition(next_position[lleg_joint_6])
            joint_motors[rleg_joint_1].setPosition(next_position[rleg_joint_1])
            joint_motors[rleg_joint_2].setPosition(next_position[rleg_joint_2])
            joint_motors[rleg_joint_3].setPosition(next_position[rleg_joint_3])
            joint_motors[rleg_joint_4].setPosition(next_position[rleg_joint_4])
            joint_motors[rleg_joint_5].setPosition(next_position[rleg_joint_5])
            joint_motors[rleg_joint_6].setPosition(next_position[rleg_joint_6])
            joint_motors[larm_joint_1].setPosition(next_position[larm_joint_1])
            joint_motors[larm_joint_2].setPosition(next_position[larm_joint_2])
            joint_motors[larm_joint_3].setPosition(next_position[larm_joint_3])
            joint_motors[larm_joint_4].setPosition(next_position[larm_joint_4])
            joint_motors[rarm_joint_1].setPosition(next_position[rarm_joint_1])
            joint_motors[rarm_joint_2].setPosition(next_position[rarm_joint_2])
            joint_motors[rarm_joint_3].setPosition(next_position[rarm_joint_3])
            joint_motors[rarm_joint_4].setPosition(next_position[rarm_joint_4])
        else:
            for i in range(25):
                next_position[i] = joint_sensors[i].getValue()

        # Avanzar simulación un paso
        if robot.step(control_step) == -1:
            break

        # Sensores de fuerza / contacto
        left_force = left_touch.getValue() / 10.0
        right_force = right_touch.getValue() / 10.0
        sum_force = left_force + right_force

        print(f"Touch sensors: left force: {left_force:4.1f} N right force: {right_force:4.1f} N -> sum: {sum_force:4.1f} N")

if __name__ == "__main__":
    main()
