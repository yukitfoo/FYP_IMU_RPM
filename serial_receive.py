from datetime import date, datetime
import asyncio
from bleak import BleakScanner, BleakClient
import numpy as np
import time
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
import math


char_uuid = '19b10001-e8f2-537e-4f6c-d104768a1214'


async def main():
    ekf = EKF(magnetic_ref=69.0, frequency=f)
    q = np.array([1,0,0,0])
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
                print(angle, current_time-previous_time, (angle*60/(current_time-previous_time))/360)
                previous_q = current_q
                previous_time = current_time

                # [iteration, period, acclX, acclY, acclZ, gyroX, gyroY, gyroZ, magX, magY, magZ] = [float(i) for i in val.decode("utf-8").split(",")]
                # print([iteration, period, acclX, acclY, acclZ, gyroX, gyroY, gyroZ, magX, magY, magZ])
                # gyro_data = np.array([gyroX, gyroY, gyroZ])
                # accl_data = np.array([acclX, acclY, acclZ])
                # mag_data = np.array([magX, magY, magZ])
                # j = ekf.update(q=q, gyr=gyro_data, acc=accl_data, mag=mag_data)
                # # calculate RPM
                # angle = 2*math.degrees(math.acos(abs(np.dot(q,j))))
                # print(angle)
                # q = j
                # print(q)
                # await asyncio.sleep()
                # perform kalman filter
                # calculate rpm
                # plot
                


asyncio.run(main())