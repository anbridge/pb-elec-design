

from numpy import cos, sin, radians
import math
from quaternion import *
import matplotlib.pyplot as plt
import asyncio
import array
from bleak import BleakClient
from mathutils import Vector


plt.ion()

pb_ble_mac = "00:80:E1:21:10:0F"
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



# 
def adjust_to_master_orientation(initial_points, i_t, i_p, i_n,  q_pt0):
    adjusted_points = [ x - initial_points[0] for x in initial_points ] # shift relative to root point
    adjusted_tangents = i_t.copy()
    adjusted_perps = i_p.copy()
    adjusted_normals = i_n.copy()
    q_pt0_c = q_pt0.conjugate()
    for i in range(len(adjusted_points)):
        pt = adjusted_points[i]
        q_pt = q_pt0*quaternion(0, pt.x, pt.y, pt.z)*q_pt0_c
        q_t = q_pt0*quaternion(0, i_t[i].x, i_t[i].y, i_t[i].z)*q_pt0_c
        q_p = q_pt0*quaternion(0, i_p[i].x, i_p[i].y, i_p[i].z)*q_pt0_c
        q_n = q_pt0*quaternion(0, i_n[i].x, i_n[i].y, i_n[i].z)*q_pt0_c
        adjusted_points[i] = Vector((q_pt.x, q_pt.y, q_pt.z)) + initial_points[0]
        adjusted_tangents[i] = Vector((q_t.x, q_t.y, q_t.z))
        adjusted_perps[i] = Vector((q_p.x, q_p.y, q_p.z))
        adjusted_normals[i] = Vector((q_n.x, q_n.y, q_n.z))
    print("adjusted points:\n", adjusted_points)
    print("adjusted tans:\n", adjusted_tangents)
    print("adjusted perps:\n", adjusted_perps)
    print("adjusted norms:\n", adjusted_normals)
    return adjusted_points, adjusted_tangents, adjusted_perps, adjusted_normals



def generate_spine(initial_points, tangents, perpendiculars, n_sub):
    if len(tangents) != len(initial_points):
        return []
    
    lengths = [ (initial_points[i+1] - initial_points[i]).length for i in range(len(initial_points)-1) ]

    all_points = [ initial_points[0].copy() ]
    key_points = [ initial_points[0].copy() ]
    all_perps = [ perpendiculars[0].copy() ]
    current_location = initial_points[0].copy()
    for i in range(len(initial_points) - 1):
        d = lengths[i]/(n_sub)

        current_location += tangents[i]*(d/2)
        perp_interp = perpendiculars[i].slerp(perpendiculars[i+1], 1/(n_sub*2))
        all_points.append(current_location.copy())
        all_perps.append(perp_interp.copy())
        
        for j in range(1, n_sub):
            interp_tangent = tangents[i].slerp(tangents[i+1], j/n_sub)
            current_location += interp_tangent*d
            perp_interp = perpendiculars[i].slerp(perpendiculars[i+1], (1+2*j)/(n_sub*2))
            all_perps.append(perp_interp.copy())
            all_points.append(current_location.copy())
        
        current_location += tangents[i+1]*(d/2)
        all_perps.append(perpendiculars[i+1].copy())
        all_points.append(current_location.copy())
        key_points.append(current_location.copy())
    
    return key_points, all_points, all_perps
            

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
            quat_doubles = array.array('d', char_bytes)
            #print("characteristic value: ", quat_doubles)
            tangents = []; perpendiculars = []; normals = [];
            num_imus = int(len(quat_doubles)/4);
            for i in range(num_imus):
                qd = quat_doubles[i*4:i*4+4]
                q = from_float_array(qd)
                q_c = q.conjugate()
                t = q * y * q_c
                p = q * x * q_c
                n = q * z * q_c
                tangents.append(Vector((t.x, t.y, t.z))); perpendiculars.append(Vector((p.x, p.y, p.z))); normals.append(Vector((n.x, n.y, n.z)))
                #ax.plot(xs=(0, n.x), ys=(0,n.y), zs=(0, n.z), c=('red', 'blue')[i])
                #ax.plot(xs=(0, t.x), ys=(0,t.y), zs=(0, t.z), c=('red', 'blue')[i])
                #ax.plot(xs=(0, p.x), ys=(0,p.y), zs=(0, p.z), c=('red', 'blue')[i])
            #plt.pause(0.15)
            #ax.clear()
            #continue
            #print("p: ", perpendiculars)
            #print("n: ", normals)
            #print("t: ", tangents)
            imu_pos, pts, prps = generate_spine(points_i, tangents, perpendiculars, 10)
            # draw imus with orientations
            for i in range(num_imus): 
                ts = tangents[i]; ps = perpendiculars[i]; ns = normals[i]
                base = imu_pos[i]
                ax.plot( xs=(base.x-0.15*ts.x, base.x+0.15*ts.x), ys=(base.y-0.15*ts.y, base.y+0.15*ts.y), zs=(base.z-0.15*ts.z, base.z+0.15*ts.z), c='red')
                ax.plot( xs=(base.x-0.15*ps.x, base.x+0.15*ps.x), ys=(base.y-0.15*ps.y, base.y+0.15*ps.y), zs=(base.z-0.15*ps.z, base.z+0.15*ps.z), c='green')
                ax.plot( xs=(base.x-0.15*ns.x, base.x+0.15*ns.x), ys=(base.y-0.15*ns.y, base.y+0.15*ns.y), zs=(base.z-0.15*ns.z, base.z+0.15*ns.z), c='blue')

            # draw interpolated spine
            for i in range(len(pts)):
                ax.plot( xs=(pts[i].x-0.15*prps[i].x, pts[i].x+0.15*prps[i].x), ys=(pts[i].y-0.15*prps[i].y, pts[i].y+0.15*prps[i].y), zs=(pts[i].z-0.15*prps[i].z, pts[i].z+0.15*prps[i].z), c='green')
                if i != (len(pts)-1): # can't draw new line from last point
                    ax.plot( xs=(pts[i].x, pts[i+1].x), ys=(pts[i].y, pts[i+1].y), zs=(pts[i].z, pts[i+1].z), c='black')

            plt.pause(0.05)
            ax.clear()


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
