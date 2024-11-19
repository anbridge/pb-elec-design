from numpy import cos, sin, radians
import math
from quaternion import *
import matplotlib.pyplot as plt
import asyncio
import array
from bleak import BleakClient



# CHANGE MAC ADDRESS TO MATCH YOUR BOARD!
pb_ble_mac = "1EDD3F2B-A067-81F7-ECC3-4F9C01117620" 
CHAR_UUID = "00001234-8e22-4541-9d4c-21edae82ed19"
#CHAR_UUID = "1EDD3F2B-A067-81F7-ECC3-4F9C01117620"

RAW = False # raw = raw readings, else read quaternion

plt.ion()

#math stuff
x = quaternion(0, 1, 0, 0)
y = quaternion(0, 0, 1, 0)
z = quaternion(0, 0, 0, 1)

fig, ax = plt.subplots(figsize=(5, 6), subplot_kw={'projection': '3d'})
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

#td, = ax.plot(xs=(0, q.x), ys=(0, -q.y), zs=(0, -q.z), c='red')

# Adjust plots
ax.axes.set_xlim3d(left=-1, right=1)
ax.axes.set_ylim3d(bottom=-1, top=1)
ax.axes.set_zlim3d(bottom=-1, top=1)
ax.set_box_aspect([1,1,1])

async def main(address):
    async with BleakClient(address) as client:
        for s in client.services:
            print(s)
            for c in s.characteristics:
                print(c)

        while 1:
            if RAW:
                char_bytes = await client.read_gatt_char(CHAR_UUID)
                floats = array.array('f', char_bytes)
                print("accel: <{x},{y},{z}>".format(x=floats[0], y=floats[1], z=floats[2]))
                print("gyro: <{x},{y},{z}>".format(x=floats[3], y=floats[4], z=floats[5]))
                print("mag: <{x},{y},{z}>".format(x=floats[6], y=floats[7], z=floats[8]))
                print("temp: "+str(floats[9])+"C\n\n")
            else:
                char_bytes = await client.read_gatt_char(CHAR_UUID)
                doubles = array.array('d', char_bytes)
                print("q: ", doubles)
                q = from_float_array(doubles[0:4])
                q_c = q.conjugate()
                t = q * y * q_c
                p = q * x * q_c
                n = q * z * q_c
                ax.axes.set_xlim3d(left=-1, right=1)
                ax.axes.set_ylim3d(bottom=-1, top=1)
                ax.axes.set_zlim3d(bottom=-1, top=1)
                ax.set_box_aspect([1,1,1])
                ax.plot(xs=(0, n.x), ys=(0,n.y), zs=(0, n.z), c=('red', 'blue')[0])
                ax.plot(xs=(0, t.x), ys=(0,t.y), zs=(0, t.z), c=('red', 'blue')[0])
                ax.plot(xs=(0, p.x), ys=(0,p.y), zs=(0, p.z), c=('red', 'blue')[0])
                plt.pause(0.05)
                ax.clear()



asyncio.run(main(pb_ble_mac))
#asyncio.run(main(CHAR_UUID))