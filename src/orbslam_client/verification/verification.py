import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

mpl.rcParams['legend.fontsize'] = 10


def readData( filename , x, y, z):
    with open(filename,'r') as f:  
        for line in f.readlines():  
            linestr = line.strip()  
            linestrlist = linestr.split()  
            linelist = map(float,linestrlist)
            x.append(float(linelist[1]))
            y.append(float(linelist[2]))
            z.append(float(linelist[3]))
              
data1 = "KeyFrameTrajectory.txt" 
x1 = []
y1 = []
z1 = []
readData( data1, x1, y1, z1)
data2 = "KeyFrameTrajectorystd.txt" 
x2 = []
y2 = []
z2 = []
readData( data2, x2, y2, z2)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x1, y1, z1,label='test')
ax.plot(x2, y2, z2,label='std')
ax.legend()

plt.show()