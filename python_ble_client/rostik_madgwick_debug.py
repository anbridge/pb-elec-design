

from numpy import cos, sin, radians
import math
from quaternion import *
import matplotlib.pyplot as plt
import asyncio
import array
from bleak import BleakClient


plt.ion()

pb_ble_mac = "00:80:E1:21:13:5A" #"00:80:E1:21:10:29" #"00:80:E1:21:10:0F"
CHAR_UUID = "00001234-8e22-4541-9d4c-21edae82ed19"

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

qfixed = quaternion(0.5, 0.5, 0.5, 0.5)
adjustments = [None, None, None] 
            

async def main(address):
    async with BleakClient(address) as client:
        for s in client.services:
            print(s)
            for c in s.characteristics:
                print(c)

        while 1:
            ax.axes.set_xlim3d(left=-1, right=1)
            ax.axes.set_ylim3d(bottom=-1, top=1)
            ax.axes.set_zlim3d(bottom=-1, top=1)
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')
            
            char_bytes = await client.read_gatt_char(CHAR_UUID)
            floats = array.array('f', char_bytes[:40])
            print("accel: <{x},{y},{z}>".format(x=floats[0], y=floats[1], z=floats[2]))
            print("gyro: <{x},{y},{z}>".format(x=floats[3], y=floats[4], z=floats[5]))
            print("mag: <{x},{y},{z}>".format(x=floats[6], y=floats[7], z=floats[8]))
            print("temp: "+str(floats[9])+"C\n\n")

            quat_doubles = array.array('d', char_bytes[40:])
            #print("characteristic value: ", quat_doubles)
            tangents = []; perpendiculars = []; normals = [];
            num_imus = int(len(quat_doubles)/4);
            for i in range(num_imus):
                qd = quat_doubles[i*4:i*4+4]
                if adjustments[i] == None:
                    pass #adjustments[i] 
                q = from_float_array(qd)
                q_c = q.conjugate()
                t = q * y * q_c
                p = q * x * q_c
                n = q * z * q_c
                tangents.append(t); perpendiculars.append(p); normals.append(n)
            # draw imus with orientations
            for i in range(num_imus): 
                ts = tangents[i]; ps = perpendiculars[i]; ns = normals[i]
                base = quaternion(1,0,0,0)
                ax.plot( xs=(base.x-0.05*ts.x, base.x+0.15*ts.x), ys=(base.y-0.05*ts.y, base.y+0.15*ts.y), zs=(base.z-0.05*ts.z, base.z+0.15*ts.z), c='red')
                ax.plot( xs=(base.x-0.05*ps.x, base.x+0.15*ps.x), ys=(base.y-0.05*ps.y, base.y+0.15*ps.y), zs=(base.z-0.05*ps.z, base.z+0.15*ps.z), c='green')
                ax.plot( xs=(base.x-0.05*ns.x, base.x+0.15*ns.x), ys=(base.y-0.05*ns.y, base.y+0.15*ns.y), zs=(base.z-0.05*ns.z, base.z+0.15*ns.z), c='blue')
            plt.pause(0.05)
            ax.clear()


asyncio.run(main(pb_ble_mac))
