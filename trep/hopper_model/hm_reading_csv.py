import trep
from trep import tx,ty,tz,rx,ry,rz
import trep.potentials
import time
import trep.visual as visual
import math
import numpy as np
from math import sin
import csv
import pandas as pd


dt = 0.01
tf = 10.0
# Set up the system frames
system = trep.System()
system.import_frames([
    # Define the 3 Center Links
    rx('theta1', kinematic=True),[
        tz(-0.7, name='pend1'),[
            rx('theta2'),[
                tz(-0.7, name='pend2'),[
                    rx('theta3'),[
                        tz(-0.5, name='legConnection'),[
                            tz(-0.5, mass=1, name='COMLeg'),[
                                tz(-1, name='pend3')]]]]]]],
    # Define the 2 Right Links
    ty(1), [
        rx('theta4', kinematic=True),[
            tz(-0.5, mass=1, name='COM4'),[
                tz(-0.5),[
                    rx('theta5'),[
                        tz(-1, name='pend5')]]]]],
    # Define the 2 Left Links
    ty(-1), [
        rx('theta6', kinematic=True),[
            tz(-0.5, mass=1, name='COM6'),[
                tz(-0.5, name='pend6'),[
                    rx('theta7'),[
                        tz(-1, name='pend7')
                    ]
                ]
            ]
        ]
    ]
])

# Establish gravity
trep.potentials.Gravity(system, name="Gravity")
# trep.forces.Damping(system, 0.1)

# Input Torque
# trep.forces.ConfigForce(system, 'theta1', 'theta1-torque-middle')
# trep.forces.ConfigForce(system, 'theta4', 'theta4-torque-right')
# trep.forces.ConfigForce(system, 'theta6', 'theta6-torque-left')
# def forcing(t):
#     return (0.0*math.exp(-(t-5)**2), -7*math.exp(-(t-5)**2), 7*math.exp(-(t-5)**2))

# Add constraints
trep.constraints.PointToPoint2D(system,'yz','pend5','legConnection')
trep.constraints.PointToPoint2D(system,'yz','pend7','legConnection')

# Assign values to the system initial configuration
pie = math.pi
# system.q = {
#     'theta1' : pie/3,
#     'theta2' : -2*pie/3,
#     'theta3' : pie/3,
#     'theta4' : 0,
#     'theta5' : -pie/2,
#     'theta6' : 0,
#     'theta7' : pie/2
#     }
# system.q = (pie/3,-2*pie/3,pie/3,0,-pie/2,0,pie/2)

#these values are matched from the MATLAB init conditions
system.q = (-2.4119+pie/2,1.6821,-0.8411,0+pie/2,-pie/2,0-pie/2,pie/2)

system.satisfy_constraints()

# Simulate
start = time.clock()
q0 = system.q
qk2_0 = system.qk

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(0.0, q0, dt, q0)

# Initialize empty arrays for time and configuration
theta6velocity = []
theta1velocity = []
theta4velocity = []

# Open and read the csv file
simlogreader = np.genfromtxt('C:/Users/ziwan/School/Hop3r/trep/hopper_model/JointVelocities.csv',delimiter=',')

for row in simlogreader:
    # print row[0],row[1],row[2]
    try:
        q6,q1,q4 = float(row[0]),float(row[1]),float(row[2])
        theta6velocity = np.append(theta6velocity, q6)
        theta1velocity = np.append(theta1velocity, q1)
        theta4velocity = np.append(theta4velocity, q4)
    except ValueError:
        print("Error extracting float from string")


# This is our simulation loop.  We save the results in two lists.
q = [mvi.q2]
t = [mvi.t2]
while mvi.t1 < tf:
    qk2 = list(qk2_0)
    qk2[system.get_config('theta1').k_index] += theta1velocity[(int(mvi.t1/dt))]
    qk2[system.get_config('theta4').k_index] += theta4velocity[(int(mvi.t1/dt))]
    qk2[system.get_config('theta6').k_index] += theta6velocity[(int(mvi.t1/dt))]
    mvi.step(mvi.t2+dt, (), tuple(qk2))
    q.append(mvi.q2)
    t.append(mvi.t2)

finish = time.clock()

# Display
print "Simulation: dt=%f, tf=%f, runtime=%f s" % (dt, tf, finish-start)
visual.visualize_3d([ visual.VisualItem3D(system, t, q) ])
