from distutils import dist
from turtle import color
import numpy as np
import matplotlib.pyplot as plt

path1 = "/home/adi99/Documents/DRDO-project/milestone-2/Data/Overtake/data/overtake-10-obstacle-default.txt"
path2 = "/home/adi99/Documents/DRDO-project/milestone-2/Data/Overtake/data/overtake-10-obstacle-default.txt"
path3 = "/home/adi99/Documents/DRDO-project/milestone-2/Data/Overtake/data/overtake-10-obstacle-default.txt"

print(path1)
print(path2)
print(path3)
data = np.loadtxt(path1)
data2 = np.loadtxt(path2)
data3 = np.loadtxt(path3)

ax = data[:,0]
ay = data[:,1]

obsx = np.vstack((data[:,9], data[:,11], data[:,13], data[:,15], data[:,17], data[:,19], data[:,21], data[:,23], data[:,25], data[:,27]))


obsy = np.vstack((data[:,10], data[:,12], data[:,14], data[:,16], data[:,18], data[:,20], data[:,22], data[:,24], data[:,26], data[:,28]))

#print(obsx.shape, obsy.shape, ax.shape)

#dist_obs = ((ax - obsx)**2 + (ay - obsy)**2)**0.5
dist_obs = (ax - obsx)
obsd = []
for i in range(10):
    a = dist_obs[i][dist_obs[i]>-10]
    obsd.append(a[a<10])
print(dist_obs.shape)

for i in range(dist_obs.shape[0]):
    plt.plot(obsd[i], label="Obstacle " + str(i+1))

#plt.plot(accx1, label="Planning Horizon = 50")
#plt.plot(accx2, label="Planning Horizon = 30")
#plt.plot(accx3, label="Planning Horizon = 70")
plt.title("X coordinate difference between Ego Vehicle and Obstacle for 50 steps")
plt.ylabel("Difference between x coordinate of ego vehicle and obstacle (m)")
plt.legend()
plt.show()