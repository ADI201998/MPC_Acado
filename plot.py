from distutils import dist
from turtle import color
import numpy as np
import matplotlib.pyplot as plt

path1 = "/home/adi99/Documents/DRDO-project/milestone-2/Data/Crossing/data/crossing-default.txt"
path2 = "/home/adi99/Documents/DRDO-project/milestone-2/Data/Crossing/data/crossing-20m.txt"
path3 = "/home/adi99/Documents/DRDO-project/milestone-2/Data/Crossing/data/crossing-10m.txt"

print(path1)
print(path2)
print(path3)
data1 = np.loadtxt(path1)
data2 = np.loadtxt(path2)
data3 = np.loadtxt(path3)

accx1 = data1[:, 3]
accx2 = data2[:, 3]
accx3 = data3[:, 3]

plt.plot(accx1, label="Detecting Distance = 30")
plt.plot(accx2, label="Detecting Distance = 20")
plt.plot(accx3, label="Detecting Distance = 10")
plt.title("Velocity for different Detecting Distances")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.show()