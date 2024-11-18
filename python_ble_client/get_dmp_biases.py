import asyncio
import array
from bleak import BleakClient


pb_ble_mac = "00:80:E1:21:13:5A" #"00:80:E1:21:10:29" #"00:80:E1:21:10:0F"
CHAR_UUID = "00001235-8e22-4541-9d4c-21edae82ed19"


async def main(address):
    async with BleakClient(address) as client:
        for s in client.services:
            print(s)
            for c in s.characteristics:
                print(c)
            
        char_bytes = await client.read_gatt_char(CHAR_UUID)
        print(len(char_bytes))
        biases = array.array('i', char_bytes)
        for i in range(int(len(biases)/9)):
            print("DMP f{i+1}")
            print("Accel:")
            print("x: ", biases[i*9+0])
            print("y: ", biases[i*9+1])
            print("z: ", biases[i*9+2], "\n")
            print("Gyro:")
            print("x: ", biases[i*9+3])
            print("y: ", biases[i*9+4])
            print("z: ", biases[i*9+5], "\n")
            print("Compass :")
            print("x: ", biases[i*9+6])
            print("y: ", biases[i*9+7])
            print("z: ", biases[i*9+8], "\n")
        
asyncio.run(main(pb_ble_mac))
#points = [ Vector((0,0,0)), Vector((0,0,1)), Vector((0,0,2))]
#tangents = [ Vector((0,0,1)), Vector((1,0,0)), Vector((0,0,1)) ]
#perps = [ Vector((0,1,0)), Vector((0,0,1)), Vector((0,1,0)) ]
#normals = [ Vector((-1,0,0)), Vector((0,1,0)), Vector((-1,0,0)) ]
#orientation = quaternion(0.707, 0.707, 0, 0)
#new_points, new_tangents, new_perps, new_normals = adjust_to_master_orientation( points, tangents, perps, normals, orientation)

#pts, prps = generate_spine(new_points, new_tangents, new_perps, 10)
#print(pts)
#for i in range(len(pts)):
#    ax.plot( xs=(pts[i].x-0.15*prps[i].x, pts[i].x+0.15*prps[i].x), ys=(pts[i].y-0.15*prps[i].y, pts[i].y+0.15*prps[i].y), zs=(pts[i].z-0.15*prps[i].z, pts[i].z+0.15*prps[i].z), c='blue')
#    if i != (len(pts)-1): # can't draw new line from last point
#        ax.plot( xs=(pts[i].x, pts[i+1].x), ys=(pts[i].y, pts[i+1].y), zs=(pts[i].z, pts[i+1].z), c='red')

#plt.show()
