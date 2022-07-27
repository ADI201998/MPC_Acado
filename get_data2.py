from distutils import dist
from turtle import color
import numpy as np
import matplotlib.pyplot as plt

path = "/home/adi99/Documents/DRDO-project/milestone-2/Data/Follow-1obs/data/follow-path-to-overtake-1-obstacle-13ms.txt"

print(path)
data = np.loadtxt(path)

idx = np.where((data[:,-1] == 2))[0]
data_f = data[idx]
#print(data)

ax = data_f[:,0]
ay = data_f[:,1]

obsx = np.vstack((data_f[:,9], data_f[:,11], data_f[:,13], data_f[:,15], data_f[:,17], data_f[:,19], data_f[:,21], data_f[:,23], data_f[:,25], data_f[:,27]))


obsy = np.vstack((data_f[:,10], data_f[:,12], data_f[:,14], data_f[:,16], data_f[:,18], data_f[:,20], data_f[:,22], data_f[:,24], data_f[:,26], data_f[:,28]))


dist_obs = ((ax - obsx)**2 + (ay - obsy)**2)**0.5
print(np.min(dist_obs))
#print(np.min(sdobs))
"""for i in range(len(ax)):
    plt.clf()
    #plt.scatter(ax[i], ay[i], color='r')
    #plt.scatter(obsx[:,i], obsy[:,i], color='b')
    agent = plt.Circle((ax[i], ay[i]), 1.0, color='g')
    id == 0
    plt.gca().add_patch(agent)
    for j in range(len(obsx[:,i])):
        if obsx[j,i]>700:
            continue
        id = j
        obs = plt.Circle((obsx[j,i], obsy[j,i]), 1.0, color='r')
        plt.gca().add_patch(obs)
    plt.xlim(ax[i]-50, ax[i]+50)
    plt.ylim(ay[i]-50, ay[i]+50)
    print(obsx[id,i], ax[i], abs(obsx[id,i] - ax[i]))
    plt.pause(0.01)"""

idx_2 = idx[-1]
#print(idx_2)
data_o = data[idx_2+1:, :]
ax = data_o[:,0]
ay = data_o[:,1]

obsx = data_o[:,9]


obsy = data_o[:,10]
#print(ax)
#print(obsx)
idx = np.where(((ax-obsx)<4.0))
data_o = data_o[idx]
ax = data_o[:,0]
ay = data_o[:,1]

obsx = data_o[:,9]


obsy = data_o[:,10]

print(len(ax))

"""for i in range(len(ax)):
    plt.clf()
    #plt.scatter(ax[i], ay[i], color='r')
    #plt.scatter(obsx[:,i], obsy[:,i], color='b')
    agent = plt.Circle((ax[i], ay[i]), 1.0, color='g')
    id == 0
    plt.gca().add_patch(agent)
    obs = plt.Circle((obsx[i], obsy[i]), 1.0, color='r')
    plt.gca().add_patch(obs)
    plt.xlim(ax[i]-50, ax[i]+50)
    plt.ylim(ay[i]-50, ay[i]+50)
    print(obsx[i], ax[i], abs(obsx[i] - ax[i]))
    plt.pause(0.01)"""