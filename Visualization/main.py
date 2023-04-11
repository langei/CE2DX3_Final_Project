# COMPENG 2DX3 Final Project
# Ivan Lange
# April 12, 2023


import serial
import struct
import math
import open3d as o3d
import numpy as np


s = serial.Serial('COM5', 115200, timeout = 10)
f = open("test.xyz", "w")

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()


# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())


measurementNumByte =  s.read()
numSamplesByte = s.read()

measurementNum = int.from_bytes(measurementNumByte, 'little')
numSamples = int.from_bytes(numSamplesByte, 'little')

anglePerSample = 360/numSamples


print(measurementNum)
print(numSamples)
print(anglePerSample)

dataStore = 0
temp = 0
z = 0
y = 0

for i in range(measurementNum):
    startingAngle = 90-anglePerSample
    for j in range(numSamples):
        dataStore = s.read(2);
        temp = struct.unpack('H', dataStore)[0]
        print(temp)
        z = math.cos(startingAngle*math.pi/180)*temp
        y = math.sin(startingAngle*math.pi/180)*temp
        f.write('{0:d} {1:f} {2:f} \n'.format(i*300, y, z))
        startingAngle = startingAngle - anglePerSample
f.close()

#Read the test data in from the file we created        
print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("test.xyz", format="xyz")

#Lets see what our point cloud data looks like numerically       
print("The PCD array:")
print(np.asarray(pcd.points))

#Lets see what our point cloud data looks like graphically       
print("Lets visualize the PCD: (spawns seperate interactive window)")
o3d.visualization.draw_geometries([pcd])

#OK, good, but not great, lets add some lines to connect the vertices
#   For creating a lineset we will need to tell the packahe which vertices need connected
#   Remember each vertex actually contains one x,y,z coordinate

#Give each vertex a unique number
yz_slice_vertex = []
for x in range(0,measurementNum*numSamples):
    yz_slice_vertex.append([x])

#Define coordinates to connect lines in each yz slice        
lines = []  
for i in range(0,measurementNum*numSamples, numSamples):
    for j in range(0, numSamples):
        if (j==numSamples-1):
            lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i]])
        else:
            lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i+j+1]])
    
#Define coordinates to connect lines between current and next yz slice        
for i in range(0,measurementNum*numSamples-numSamples, numSamples):
    for j in range(0, numSamples):
        lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i+j+numSamples]])

#This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically       
o3d.visualization.draw_geometries([line_set])
        
s.close()      
