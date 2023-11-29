import socket
import time
import asyncio
from bleak import BleakScanner, BleakClient
import numpy as np
import time
import math

host, port = "127.0.0.1", 25010
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
char_uuid = '19b10001-e8f2-537e-4f6c-d104768a1214'
y = [0,1,0]
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


def calculate_cross_product(a,b):
    c = [0]*3
    c[0] = a[1]*b[2] - b[1]*a[2]
    c[1] = -(a[0]*b[2] - b[0]*a[2])
    c[2] = a[0]*b[1] - b[0]*a[1]
    return c

# deg will always be within 90 since absolute values are used for opposite and hypothenuse
def calculate_cos(n1, n2):
    adotb = sum([elt*n2[i] for i, elt in enumerate(n1)])
    prod_of_mag = calculate_euclidean_distance(n1)*calculate_euclidean_distance(n2)
    if prod_of_mag == 0:
        return 0
    return math.degrees(math.acos(abs(adotb)/abs(prod_of_mag)))


def transform_degrees(a, b, deg):
    [aX, aY] = a
    [bX, bY] = b
    if aX == bX and aY == bY:
        return deg
    if aX == bX:
        # for completely same unit vectors and its absolute equal 
        if aY == bY-180:
            return deg
        else:
            # chack if Y values are equal, larger or smaller than
            if aY > bY:
                return deg + 180
            # aY==bY case covered above
            else:
                return 180 - deg
    elif aX < bX:
        if aY <= bY:
            return deg
        else:
            return -deg
    else:
        if aY == bY:
            return 180
        elif aY < bY:
            return 180 - deg
        else:
            return -180 + deg
    
# f = open("C:\\Users\\Yu Kit\\Desktop\\FYP\\Data\\data.csv", "w")

def filter_accel(l):
    d = []
    for i in l:
        if abs(i) > 3:
            d.append(int(i))
        else:
            d.append(3)
    return d


async def con(sock):
    accum = ""
    peripheral = False
    devices = await BleakScanner.discover()
    for d in devices:
        
        if d.name == "Seeed":
            print(f"found {d}")
            peripheral = d
    if peripheral:
        async with BleakClient(peripheral.address) as client:
            prev = 0
            # i = 0
            while client:
                val = await client.read_gatt_char(char_uuid)
                current = [float(i) for i in val.decode("utf-8").split(",")]
                # euler angles index 0,1,2
                current_euler = current[0:3]
                # acceleration unit vector index 3,4,5 
                current_accel = filter_accel(current[3:6])
                normalizing_accel = calculate_euclidean_distance(current_accel)
                current_accel = [i/normalizing_accel for i in current_accel]
                # gravity unit vector index 6,7,8
                current_gravity = current[6:9]
                normalizing_gravity = calculate_euclidean_distance(current_gravity)
                current_gravity = [i/normalizing_accel for i in current_gravity]
                # calculate spherical coordinates
                aX, aY = calculate_y_tangent(current_accel), calculate_tangent(current_accel[0],current_accel[2])
                bX, bY = calculate_y_tangent(current_gravity), calculate_tangent(current_gravity[0],current_gravity[2])
                AY = calculate_cross_product(current_accel, y)
                AB = calculate_cross_product(current_accel, current_gravity)
                deg = calculate_cos(AY, AB)
                deg = transform_degrees([aX, aY],[bX, bY], deg)
                accel_spherical = [aX, aY, 0]
                gravity_spherical = [bX, bY, 0]
                camera_rotation = [aX, aY, deg]
                # RPM index 9
                current_RPM = current[-1]
                print(current_euler, accel_spherical, gravity_spherical, camera_rotation, current_RPM)
                to_send = f"{','.join(map(str, current_euler))};{','.join(map(str,accel_spherical))};{','.join(map(str, gravity_spherical))};{','.join(map(str,camera_rotation))};{str(current_RPM)}"
                accum += to_send+"\n"
                sock.sendall(to_send.encode("utf-8"))
                response = sock.recv(1024).decode("utf-8")
                print(response)
                time.sleep(0.01)
                # i+=1
    return accum

                
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
a = asyncio.run(con(sock))
sock.close()
# f.write(a)
# f.close()
print("sock closed")

