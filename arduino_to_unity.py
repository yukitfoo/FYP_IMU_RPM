import socket
import time
import asyncio
from bleak import BleakScanner, BleakClient
import numpy as np
import time
import math

host, port = "127.0.0.1", 25005
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
char_uuid = '19b10001-e8f2-537e-4f6c-d104768a1214'

# range (-180, 180)
# Y
def calculate_tangent(opposite, adjacent):
    if adjacent == 0:
        if opposite == 0:
            return 0
        if opposite < 0:
            return -90
        else:
            return 90
    if opposite == 0:
        if adjacent < 0:
            return 180
        else:
            return 0
    # at +opposite +adjacent
    theta = math.degrees(math.atan(abs(opposite)/abs(adjacent)))
    if opposite < 0:
        #  at -opposite -adjacent
        if adjacent < 0:
            theta = -180 + theta
        # at -opposite +adjacent
        else:
            theta = -theta
    else:
        # +opposite -adjacent
        if adjacent < 0:
            theta = 180 - theta
    return theta


def calculate_euclidean_distance(l):
    return (sum([i**2 for i in l]))**0.5
    
# X
def calculate_y_tangent(vector):
    d = calculate_euclidean_distance([vector[0], vector[2]])
    y = vector[1]
    theta = math.degrees(math.atan(abs(y)/abs(d)))
    if y == 0:
        theta = 0
    elif y > 0:
        theta = -theta
    return theta



async def con(sock):
    peripheral = False
    devices = await BleakScanner.discover()
    for d in devices:
        
        if d.name == "Seeed":
            print(f"found {d}")
            peripheral = d
    if peripheral:
        async with BleakClient(peripheral.address) as client:
            prev = 0
            while client:
                val = await client.read_gatt_char(char_uuid)
                current = [float(i) for i in val.decode("utf-8").split(",")]
                # euler angles index 0,1,2
                current_euler = current[0:3]
                # acceleration unit vector index 3,4,5 
                current_accel = current[3:6]
                normalizing_accel = calculate_euclidean_distance(current_accel)
                current_accel = [i/normalizing_accel for i in current_accel]
                # gravity unit vector index 6,7,8
                current_gravity = current[6:9]
                normalizing_gravity = calculate_euclidean_distance(current_gravity)
                current_gravity = [i/normalizing_accel for i in current_gravity]
                # RPM index 9
                current_RPM = current[-1]

                to_send = f"{','.join(current_euler)};{','.join(current_accel)};{','.join(current_gravity)};{current_RPM}"
                sock.sendall(to_send.encode("utf-8"))
                response = sock.recv(1024).decode("utf-8")
                print(response)
                time.sleep(0.01)

                
    # x = 10
    # y = 20
    # z = 30
    # while True:
    #     data = f"{x},{y},{z}"
    #     sock.sendall(data.encode("utf-8"))
    #     response = sock.recv(1024).decode("utf-8")
    #     print(response)
    #     x += 1
    #     y += 1
    #     z += 1
    #     if x == 360:
    #         break
    #     time.sleep(0.01)
            

# Connect to a port
sock.connect((host, port))
asyncio.run(con(sock))
sock.close()
print("sock closed")

