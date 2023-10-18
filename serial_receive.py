import os
from datetime import date, datetime
import asyncio
from bleak import BleakScanner, BleakClient
import numpy as np
import socket

host, port = "127.0.0.1", 49200
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))


# today = date.today()
# start_time = datetime.now().strftime("%H:%M:%S")
# root = "C:/Users/Yu Kit/Desktop/FYP/Data"
# current = os.path.join(root, str(today))
# if str(today) not in os.listdir(root):
#     os.mkdir(os.path.join(root, str(today)))


#     store_folder = f"{root}/Data/{today}/{start_time}.csv"

char_uuid = '19b10001-e8f2-537e-4f6c-d104768a1214'

def euler_to_quaternions(comms):
    roll, yaw, pitch = comms.split(",")
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    sum_squared = (qx**2+qy**2+qz**2+qw**2)**0.5
    return np.divide(np.array([qx, qy, qz, qw]), sum_squared)


def calculate_rpm(q1, q2, rate):
    pass

async def main():
    devices = await BleakScanner.discover()
    peripheral = None
    for d in devices:
        if d.name == "Seeed":
            print(f"found {d}")
            peripheral = d
    if peripheral:
        async with BleakClient(peripheral.address) as client:
            while client:
                print(f'Connected to {peripheral.address}')
                val = await client.read_gatt_char(char_uuid)    
                print(val)
                # convert to quaternion
                # calculate rpm
                # send to socket
                # perform kalman filter
                


asyncio.run(main())