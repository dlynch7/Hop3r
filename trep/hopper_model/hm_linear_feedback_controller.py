import trep
from trep import tx,ty,tz,rx,ry,rz
import trep.potentials
import trep.discopt
import time
import trep.visual as visual
import math
import numpy as np
from numpy import dot
from math import sin
import csv
import pylab

csvPath = '/Users/gregniederschulte/Documents/GitHub/Hop3r/trep/hopper_model/JointVelocities.csv'

t0 = 0.0
dt = 0.01
tf = 10.0
# Set up the system frames
system = trep.System()
system.import_frames([
    # Define the 3 Center Links
    rx('theta1', kinematic=True),[
        tz(-0.75, name='pend1'),[
            rx('theta2'),[
                tz(-0.75, name='pend2'),[
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

# Input Torque
trep.forces.ConfigForce(system, 'theta1', 'theta1-torque-middle')
trep.forces.ConfigForce(system, 'theta4', 'theta4-torque-right')
trep.forces.ConfigForce(system, 'theta6', 'theta6-torque-left')
def forcing(t):
    return (0.0*math.exp(-(t-5)**2), -7*math.exp(-(t-5)**2), 7*math.exp(-(t-5)**2))

# Add constraints
trep.constraints.PointToPoint2D(system,'yz','pend5','legConnection')
trep.constraints.PointToPoint2D(system,'yz','pend7','legConnection')

# Initial Configuration
q0 = (1.6821, -0.8411, -0.8957, 0.8957, -2.4119+math.pi/2, -1.7110+math.pi/2, -1.4306+(math.pi/2))
system.q = q0

# Desired Configuration
qBar = 
Q = np.eye(7) # Cost weights for states
R = np.eye(3) # Cost weights for inputs

system.satisfy_constraints(tolerance=1e-10, keep_kinematic=True)

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(t0, q0, dt, q0)

# Create discrete system
TVec = np.arange(t0, tf+dt, dt) # Initialize discrete time vector
dsys = trep.discopt.DSystem(mvi, TVec) # Initialize discrete system
xBar = dsys.build_state(qBar) # Create desired state configuration

# Design linear feedback controller
Qd = np.zeros((len(TVec), dsys.system.nQ)) # Initialize desired configuration trajectory
thetaIndex = (dsys.system.get_config('theta1').index, dsys.system.get_config('theta4').index, dsys.system.get_config('theta6').index) # Find index of theta config variable
for i,t in enumerate(TVec):
    Qd[i, thetaIndex] = qBar # Set desired configuration trajectory
    (Xd, Ud) = dsys.build_trajectory(Qd) # Set desired state and input trajectory

Qk = lambda k: Q # Create lambda function for state cost weights
Rk = lambda k: R # Create lambda function for input cost weights
KVec = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk) # Solve for linear feedback controller gain
KStabilize = KVec[0] # Use only use first value to approximate infinite-horizon optimal controller gain

# Reset discrete system state
dsys.set(np.array([q0, 0.]), np.array([0.]), 0)



# Initialize empty arrays for angular velocities of actuated joints
theta6velocity = []
theta1velocity = []
theta4velocity = []

# Open and read the csv file
simlogreader = np.genfromtxt(csvPath,delimiter=',')

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
qk2 = list(qk2_0)
while mvi.t1 < tf:
    qk2[system.get_config('theta1').k_index] += 0.01*theta1velocity[(int(mvi.t1/dt))]
    qk2[system.get_config('theta4').k_index] += 0.01*theta4velocity[(int(mvi.t1/dt))]
    qk2[system.get_config('theta6').k_index] += 0.01*theta6velocity[(int(mvi.t1/dt))]
    mvi.step(mvi.t2+dt, (), tuple(qk2))
    q.append(mvi.q2)
    t.append(mvi.t2)

finish = time.clock()

# Display
print "Simulation: dt=%f, tf=%f, runtime=%f s" % (dt, tf, finish-start)
visual.visualize_3d([ visual.VisualItem3D(system, t, q) ])