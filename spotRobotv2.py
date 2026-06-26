# Copyright 1996-2024 Cyberbotics Ltd.
# Licensed under the Apache License 2.0

from controller import Robot
import math
import sys

# ================= CONFIG =================
NUMBER_OF_JOINTS = 12

robot = Robot()
time_step = int(robot.getBasicTimeStep())

motor_names = [
    "front left shoulder abduction motor", "front left shoulder rotation motor", "front left elbow motor",
    "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
    "rear left shoulder abduction motor", "rear left shoulder rotation motor", "rear left elbow motor",
    "rear right shoulder abduction motor", "rear right shoulder rotation motor", "rear right elbow motor"
]

motors = [robot.getDevice(name) for name in motor_names]

# ================= PARÁMETROS =================
L1 = 0.18
L2 = 0.18

z_base = -0.35  # 🔥 altura corregida

ABDUCTION_ANGLE = 0.2

# ================= LIMITES =================
def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

# ================= CINEMÁTICA INVERSA =================
def inverse_kinematics(x, z):
    r = abs(x)
    h = -z
    d = math.hypot(r, h)

    # limitar alcance geométrico
    d = clamp(d, abs(L1 - L2) + 1e-6, L1 + L2 - 1e-6)

    cos_q3 = (d*d - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_q3 = clamp(cos_q3, -1.0, 1.0)

    q3 = math.acos(cos_q3)

    #  limite real del robot
    q3 = clamp(q3, -0.1, 1.6)

    beta = math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
    gamma = math.atan2(x, h)
    q2 = gamma - beta

    return q2, q3

# ================= ESTADO =================
current_angles = [0.0] * NUMBER_OF_JOINTS

# ================= MOVIMIENTO SUAVE =================
def set_pose_smooth(target_angles, duration=0.3):
    global current_angles

    steps = int(duration * 1000 / time_step)
    if steps < 1:
        steps = 1

    for i in range(steps):
        for j in range(NUMBER_OF_JOINTS):

            #  LIMITES GLOBALES
            if (j % 3) == 0:  # abduction
                target_angles[j] = clamp(target_angles[j], -0.5, 0.5)

            elif (j % 3) == 1:  # shoulder
                target_angles[j] = clamp(target_angles[j], -1.5, 1.5)

            elif (j % 3) == 2:  # elbow
                target_angles[j] = clamp(target_angles[j], -0.1, 1.6)

            current_angles[j] += (target_angles[j] - current_angles[j]) / (steps - i)
            motors[j].setPosition(current_angles[j])

        if robot.step(time_step) == -1:
            sys.exit(0)

# ================= POSICIÓN DE PIE =================
def stand():
    target = []

    for i in range(4):

        if i in [0, 2]:  # izquierda
            q1 = ABDUCTION_ANGLE
        else:
            q1 = -ABDUCTION_ANGLE

        q2, q3 = inverse_kinematics(0.0, z_base)

        target += [q1, q2, q3]

    set_pose_smooth(target, 1.5)

# ================= MOVER UNA PATA =================
def move_leg(leg_index, dx, dz):
    angles = current_angles.copy()
    base = leg_index * 3

    if leg_index in [0, 2]:
        q1 = ABDUCTION_ANGLE
    else:
        q1 = -ABDUCTION_ANGLE

    q2, q3 = inverse_kinematics(dx, z_base + dz)

    angles[base] = q1
    angles[base + 1] = q2
    angles[base + 2] = q3

    return angles

# ================= CRAWL GAIT =================
def crawl_walk(duration=10.0):
    start_time = robot.getTime()

    step_length = 0.01
    lift_height = 0.008

    sequence = [0, 3, 1, 2]  # FL, RR, FR, RL

    while robot.getTime() - start_time < duration:
        for leg in sequence:

            # levantar + avanzar
            for t in range(20):
                frac = t / 20.0

                dx = step_length * (frac - 0.5)
                dz = lift_height * math.sin(math.pi * frac)

                target = move_leg(leg, dx, dz)
                set_pose_smooth(target, 0.02)

            # apoyar
            target = move_leg(leg, 0.0, 0.0)
            set_pose_smooth(target, 0.1)

# ================= MAIN =================
if __name__ == "__main__":

    print("Inicializando robot...")

    stand()

    for _ in range(50):
        robot.step(time_step)

    print("Caminando...")

    crawl_walk(15.0)

    print("Regresando a posición de pie...")

    stand()

    print("Listo ✅")
