#!/usr/bin/env python3

import pickle
import matplotlib.pyplot as plt
import glob


# load pickle file
# find the most recently created Dead Reckoning Pickle file
files = sorted(glob.glob('DeadReck*.p'))
filename = files[-1]
robot0 = pickle.load(open(filename,"rb"))


fig0 = plt.figure()
pathline, = plt.plot(robot0.estimated_xytheta_history[:,0], robot0.estimated_xytheta_history[:,1],'b--')
plt.axis('equal')
plt.show()


##separate version to plot them all: 
#for filename in files:
#    robot0 = pickle.load(open(filename,"rb"))
#    
#    fig0 = plt.figure()
#    pathline, = plt.plot(robot0.estimated_xytheta_history[:,0], robot0.estimated_xytheta_history[:,1],'b--')
#    plt.axis('equal')
#    plt.show()