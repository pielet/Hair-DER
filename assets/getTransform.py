import os
from struct import unpack, pack

def printMatrix(mat):
    for i in range(4):
        print('{0} {1} {2} {3}'.format(*mat[4*i:4*i+4]))
    print('\n')

with open('./Rotation1.bkad', 'rb') as f:
    num_bone, num_point, num_prim, num_sking, num_frame = unpack('iiiii', f.read(4 * 5))
    time_interval = unpack('f', f.read(4))[0]
    print(num_bone, num_point, num_prim, num_sking, num_frame)
    
    num_skip = 3 * num_prim + 3 * num_point + num_point * num_bone * 2
    sth = unpack('f' * num_skip, f.read(4 * num_skip))
    
    rest_bone = []
    for i in range(num_bone):
        rest_bone.append(unpack('f' * 16, f.read(4 * 16)))
    
    bone1 = []
    bone2 = []
    for i in range(num_frame):
        bone1.append(unpack('f' * 16, f.read(64)))
        bone2.append(unpack('f' * 16, f.read(64)))
    
with open('transform.dat', 'wb') as f:
    f.write(pack('i', num_frame))
    f.write(pack('f', time_interval))
    for i in range(num_frame):
        f.write(pack('f' * 16, *bone1[i]))