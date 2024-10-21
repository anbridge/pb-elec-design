from numpy import cos, sin, radians
from quaternion import *
import matplotlib.pyplot as plt
import asyncio
import array
from bleak import BleakClient
plt.ion()

pb_ble_mac = "00:80:E1:21:10:0F"
CHAR_UUID = "00001234-8e22-4541-9d4c-21edae82ed19"

#math stuff
q = quaternion(0, 1, 0, 0) # placeholder, will be quaternion "end point"
a = quaternion(0, 0, 0, 1) # axis

fig, ax = plt.subplots(figsize=(5, 6), subplot_kw={'projection': '3d'})
ax.set_xlabel('Roll axis')
ax.set_ylabel('Pitch axis')
ax.set_zlabel('Yaw axis')

td, = ax.plot(xs=(0, q.x), ys=(0, -q.y), zs=(0, -q.z), c='red')

# Adjust plots
ax.set_box_aspect([1,1,1])

async def main(address):
    async with BleakClient(address) as client:
        for s in client.services:
            print(s)
            for c in s.characteristics:
                print(c)
        while 1:
            char_bytes = await client.read_gatt_char(CHAR_UUID)
            quat_doubles = array.array('d', char_bytes)
            print("characteristic value: ", quat_doubles)
            q = from_float_array(quat_doubles)
            q_c = q.conjugate()
            p = q * a * q_c

            ax.axes.set_xlim3d(left=-1, right=1)
            ax.axes.set_ylim3d(bottom=-1, top=1)
            ax.axes.set_zlim3d(bottom=-1, top=1)
            ax.plot(xs=(0, p.x), ys=(0, p.y), zs=(0, p.z), c='red')
            plt.pause(0.25)
            ax.clear()


asyncio.run(main(pb_ble_mac))
