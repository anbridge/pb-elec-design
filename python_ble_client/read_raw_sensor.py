import asyncio
import array
from bleak import BleakClient



# CHANGE MAC ADDRESS TO MATCH YOUR BOARD!
pb_ble_mac = "00:80:E1:21:10:0F" 
CHAR_UUID = "00001234-8e22-4541-9d4c-21edae82ed19"


           
async def main(address):
    async with BleakClient(address) as client:
        for s in client.services:
            print(s)
            for c in s.characteristics:
                print(c)

        points_i = [ Vector((0,0,0)), Vector((0,1,0)) ]
        while 1:
            ax.axes.set_xlim3d(left=-1, right=1)
            ax.axes.set_ylim3d(bottom=-1, top=1)
            ax.axes.set_zlim3d(bottom=-1, top=1)
            
            char_bytes = await client.read_gatt_char(CHAR_UUID)
            floats = array.array('f', char_bytes)
            print("accel: <{x},{y},{z}>".format(x=floats[0], y=floats[1], z=floats[2]))
            print("gyro: <{x},{y},{z}>".format(x=floats[3], y=floats[4], z=floats[5]))
            print("mag: <{x},{y},{z}>".format(x=floats[6], y=floats[7], z=floats[8]))
            print("temp: "+str(floats[9])+"C\n\n")

            plt.pause(0.05)
            ax.clear()


asyncio.run(main(pb_ble_mac))
