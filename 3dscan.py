import serial
import numpy as np
import open3d as o3d
from math import cos, sin, radians

s = serial.Serial('COM7', 115200, timeout=10)
print("Opening: " + s.name)

s.reset_output_buffer()
s.reset_input_buffer()
s.write('s'.encode())

degree = ""
sensor_data = []
count = 0
total_steps = 0
width = 0
while True:
    x = input('communicate?[Y/N]')
    
    if (x == 'N'):
        print("closing..")
        break
    total_steps = 0
    while True:
        x = s.readline()
        print(x.decode())

        try:
            degree = x.decode().split(", ")[1].strip()
            distance = x.decode().split(", ")[0].strip()
            sensor_data.append([width, distance, degree])
            total_steps+=1

        except:
            print("")
   
        if (degree.strip() == "360.000000"):        
            break

    width += 0
    count+=1

s.close()

arr = []

for data in sensor_data:
    x = data[0]
    y = float(data[1]) * cos(radians(float(data[2])))
    z = float(data[1]) * sin(radians(float(data[2])))
    cartesian_point = [x, y, z]
    arr.append(cartesian_point)

if __name__ == "__main__":
    f = open("demofile2dx.xyz", "w")    
    
    
    for arrs in arr:
            f.write('{0:d} {1:.4f} {2:.4f}\n'.format(arrs[0], arrs[1], arrs[2]))
        
    f.close()                            
    
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("demofile2dx.xyz", format="xyz")

    print("The PCD array:")
    print(np.asarray(pcd.points))

    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])

    yz_slice_vertex = []
    for x in range(0,total_steps*count):
        yz_slice_vertex.append([x])

    lines = []  
    stepper = 1
    for x in range(0,total_steps*count,total_steps):
        for i in range(x, (stepper*total_steps)-1,1):
            lines.append([yz_slice_vertex[i], yz_slice_vertex[i+1]])
        lines.append([yz_slice_vertex[x+total_steps-1], yz_slice_vertex[x]])
        stepper+=1

    stepper = 1
    if (count != 1):
        count -= 1
        for x in range(0,total_steps*(count),1):
            lines.append([yz_slice_vertex[x], yz_slice_vertex[x+total_steps]])

            stepper +=1

    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
    o3d.visualization.draw_geometries([line_set])