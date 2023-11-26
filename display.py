import pygame
import numpy as np
from math import *
from datetime import date, datetime
import asyncio
from bleak import BleakScanner, BleakClient
import numpy as np
import time
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
import math




def rad2deg(rad):
    return rad / np.pi * 180

def deg2rad(deg):
    return deg / 180 * np.pi

def getRotMat(q):
    c00 = q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2
    c01 = 2 * (q[1] * q[2] - q[0] * q[3])
    c02 = 2 * (q[1] * q[3] + q[0] * q[2])
    c10 = 2 * (q[1] * q[2] + q[0] * q[3])
    c11 = q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2
    c12 = 2 * (q[2] * q[3] - q[0] * q[1])
    c20 = 2 * (q[1] * q[3] - q[0] * q[2])
    c21 = 2 * (q[2] * q[3] + q[0] * q[1])
    c22 = q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2

    rotMat = np.array([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]])
    return rotMat

def getEulerAngles(q):
    m = getRotMat(q)
    test = -m[2, 0]
    if test > 0.99999:
        yaw = 0
        pitch = np.pi / 2
        roll = np.arctan2(m[0, 1], m[0, 2])
    elif test < -0.99999:
        yaw = 0
        pitch = -np.pi / 2
        roll = np.arctan2(-m[0, 1], -m[0, 2])
    else:
        yaw = np.arctan2(m[1, 0], m[0, 0])
        pitch = np.arcsin(-m[2, 0])
        roll = np.arctan2(m[2, 1], m[2, 2])

    yaw = rad2deg(yaw)
    pitch = rad2deg(pitch)
    roll = rad2deg(roll)

    return yaw, pitch, roll





def connect_points(i, j, points, screen, BLACK):
    pygame.draw.line(screen, BLACK, (points[i][0], points[i][1]), (points[j][0], points[j][1]))
    

def get_rotation_x(angle):
    return np.matrix([[1, 0, 0],[0, cos(angle), -sin(angle)],[0, sin(angle), cos(angle)],])


def get_rotation_y(angle):
    return np.matrix([[cos(angle), -sin(angle), 0],[sin(angle), cos(angle), 0],[0, 0, 1],])


def get_rotation_z(angle):
    return np.matrix([[cos(angle), 0, sin(angle)],[0, 1, 0],[-sin(angle), 0, cos(angle)],])

char_uuid = '19b10001-e8f2-537e-4f6c-d104768a1214'

async def main():
    # WHITE = (255, 255, 255)
    # RED = (255, 0, 0)
    # BLACK = (0, 0, 0)
    # WIDTH, HEIGHT = 800, 600
    # pygame.display.set_caption("9DOF IMU")
    # screen = pygame.display.set_mode((WIDTH, HEIGHT))

    # scale = 100

    # circle_pos = [WIDTH/2, HEIGHT/2]  # x, y

    # angle = 0

    # points = []

    # # all the cube vertices
    # points.append(np.matrix([-1, -1, 1]))
    # points.append(np.matrix([1, -1, 1]))
    # points.append(np.matrix([1,  1, 1]))
    # points.append(np.matrix([-1, 1, 1]))
    # points.append(np.matrix([-1, -1, -1]))
    # points.append(np.matrix([1, -1, -1]))
    # points.append(np.matrix([1, 1, -1]))
    # points.append(np.matrix([-1, 1, -1]))


    # projection_matrix = np.matrix([
    #     [1, 0, 0],
    #     [0, 1, 0]
    # ])


    # projected_points = [
    #     [n, n] for n in range(len(points))
    # ]
    devices = await BleakScanner.discover()
    peripheral = None
    print([d.name for d in devices])
    for d in devices:
        if d.name == "Seeed":
            print(f"found {d}")
            peripheral = d
    if peripheral:
        async with BleakClient(peripheral.address) as client:
            previous_q = 0
            previous_time = 0
            while client:
                current_time = time.time()
                val = await client.read_gatt_char(char_uuid)
                current = [float(i) for i in val.decode("utf-8").split(",")]
                current_q = np.array(current[:4])
                if type(previous_q) == int:
                    previous_q = current_q
                    previous_time = current_time
                    continue
                j = abs(np.dot(current_q,previous_q))
                angle = 2*math.degrees(math.acos(j))
                print(angle, current_q)
                # print(angle, current_time-previous_time, (angle*60/(current_time-previous_time))/360)
                # euler = getEulerAngles(current[:4])
                # print(euler)
                # rotation_x = get_rotation_x(euler[0])
                # rotation_y = get_rotation_y(euler[1])
                # rotation_z = get_rotation_z(euler[2])
                # screen.fill(WHITE)
                # for i, point in enumerate(points):
                #     rotated2d = np.dot(rotation_z, point.reshape((3, 1)))
                #     rotated2d = np.dot(rotation_y, rotated2d)
                #     rotated2d = np.dot(rotation_x, rotated2d)

                #     projected2d = np.dot(projection_matrix, rotated2d)

                #     x = int(projected2d[0][0] * scale) + circle_pos[0]
                #     y = int(projected2d[1][0] * scale) + circle_pos[1]

                #     projected_points[i] = [x, y]
                #     pygame.draw.circle(screen, RED, (x, y), 5)
                
                # for p in range(4):
                #     connect_points(p, (p+1) % 4, projected_points, screen, BLACK)
                #     connect_points(p+4, ((p+1) % 4) + 4, projected_points, screen, BLACK)
                #     connect_points(p, (p+4), projected_points, screen, BLACK)
                # pygame.display.update()

                previous_q = current_q
                previous_time = current_time

asyncio.run(main())

# clock = pygame.time.Clock()
# while True:

#     clock.tick(60)

#     # update stuff
#     get_rotation_x(angleX)
#     get_rotation_y(angleY)
#     get_rotation_Z(angleZ)

#     screen.fill(WHITE)
#     # drawining stuff

#     i = 0
#     for point in points:
#         rotated2d = np.dot(rotation_z, point.reshape((3, 1)))
#         rotated2d = np.dot(rotation_y, rotated2d)
#         rotated2d = np.dot(rotation_x, rotated2d)

#         projected2d = np.dot(projection_matrix, rotated2d)

#         x = int(projected2d[0][0] * scale) + circle_pos[0]
#         y = int(projected2d[1][0] * scale) + circle_pos[1]

#         projected_points[i] = [x, y]
#         pygame.draw.circle(screen, RED, (x, y), 5)
#         i += 1

#     for p in range(4):
#         connect_points(p, (p+1) % 4, projected_points)
#         connect_points(p+4, ((p+1) % 4) + 4, projected_points)
#         connect_points(p, (p+4), projected_points)

#     pygame.display.update()