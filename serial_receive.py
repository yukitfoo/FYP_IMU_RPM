import os
from datetime import date, datetime
import asyncio
from bleak import BleakScanner, BleakClient


# today = date.today()
# start_time = datetime.now().strftime("%H:%M:%S")
# root = "C:/Users/Yu Kit/Desktop/FYP/Data"
# current = os.path.join(root, str(today))
# if str(today) not in os.listdir(root):
#     os.mkdir(os.path.join(root, str(today)))


#     store_folder = f"{root}/Data/{today}/{start_time}.csv"

char_uuid = '19b10001-e8f2-537e-4f6c-d104768a1214'


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
                await asyncio.sleep(10)

asyncio.run(main())