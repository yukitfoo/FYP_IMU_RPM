import socket
import time
from datetime import date, datetime
import asyncio
from bleak import BleakScanner, BleakClient
import numpy as np
import time
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
import math

host, port = "127.0.0.1", 25005
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
char_uuid = '19b10001-e8f2-537e-4f6c-d104768a1214'


def con():
    sock.connect((host, port))
    # devices = await BleakScanner.discover()
    # for d in devices:
        
    #     if d.name == "Seeed":
    #         print(f"found {d}")
    #         peripheral = d
    # if peripheral:
    #     async with BleakClient(peripheral.address) as client:
    #         prev = 0
    #         while client:
    #             val = await client.read_gatt_char(char_uuid)
    #             current = [float(i) for i in val.decode("utf-8").split(",")]
    #             current_euler = current[0:3]
    #             current_quat = current[3:7]
    #             current_gravity = current[7:]
    #             if prev == 0:
    #                 prev =
                
    x = 10
    y = 20
    z = 30
    while True:
        data = f"{x},{y},{z}"
        sock.sendall(data.encode("utf-8"))
        response = sock.recv(1024).decode("utf-8")
        print(response)
        x += 1
        y += 1
        z += 1
        if x == 360:
            break
        time.sleep(0.01)
            
    sock.close()
    print("sock closed")



con()



