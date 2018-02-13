# pendulumVizOnly.py

# Import necessary Python modules
import csv
import numpy as np
import math
from math import pi
import trep
from trep import tx, ty, tz, rx, ry, rz

# Build a pendulum system
m = 1.0 # Mass of pendulum
l = 1.0 # Length of pendulum
q0 = 3./4.*pi # initial configuration of pendulum
t0 = 0.0 # initial time
tf = 5.0 # final time
dt = 0.1 # timestep

system = trep.System() # initialize system

frames = [rx('theta', name="pendulumShoulder"), [tz(-l, name="pendulumArm", mass=m)]]
system.import_frames(frames) # add frames

# Initialize empty arrays for time and configuration
tArr = [];
qArr = [];

# Open the csv file
with open('simlog.csv','rb') as csvfile:
	simlogreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
	for row in simlogreader:
		print row[0],row[1]
		try:
			t,q = float(row[0]),float(row[1])
			tArr = np.append(tArr, t)
			qArr = np.append(qArr, q)
		except ValueError:
			print("Error extracting float from string")


# announce that visualization is about to happen
print len(tArr),len(qArr)
print("Visualization ready")

# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, tArr, qArr) ])
