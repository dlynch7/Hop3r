import trep
from trep import tx,ty,tz,rx,ry,rz
import trep.potentials
import time
import trep.visual as visual
import math
import numpy as np
from math import sin
import csv

csvPath = '/Users/gregniederschulte/Documents/GitHub/Hop3r/trep/hopper_model/JointVelocities.csv'

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
# These values are matched from the MATLAB init conditions
# system.q = {
#     'theta2' : 1.6821,
#     'theta3' : -0.8411,
#     'theta5' : -0.8957,
#     'theta7' : 0.8957
#     }

# system.qk = {
#     'theta1' : -2.4119+math.pi/2,
#     'theta4' : -1.7110+math.pi/2,
#     'theta6' : -1.4306+(math.pi/2)
#     }

# These values are matched from the MATLAB init conditions
# system.q = (-2.4119+math.pi/2, 1.6821, -0.8411, -1.7110+math.pi/2, -0.8957, -1.4306+(math.pi/2), 0.8957)

system.q = (1.6821, -0.8411, -0.8957, 0.8957)
system.qk = (-2.4119+math.pi/2, -1.7110+math.pi/2, -1.4306+(math.pi/2))

# Starting the simulation from the configuration it jumps to
#system.q = (2.24633566,-1.36090603,-1.41895841,1.41896977,-1.01981214,-0.01004199,0.01003075)



system.satisfy_constraints(tolerance=1e-10, keep_kinematic=True)

# Simulate
start = time.clock()
q0 = system.q
qk0 = system.qk

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(0.0, q0, dt, q0)

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
qk2 = list(qk0)
while mvi.t1 < tf:
    system.dqk[0] = theta1velocity[(int(mvi.t1/dt))]
    system.dqk[1] = theta4velocity[(int(mvi.t1/dt))]
    system.dqk[2] = theta6velocity[(int(mvi.t1/dt))]
    mvi.step(mvi.t2+dt, (), system.dqk)
    q.append(mvi.q2)
    t.append(mvi.t2)

finish = time.clock()

def f_k(system):
	"""
	Implement Eq. 3.30b from Elliot's thesis...not currently handling
	constraints or dynamic vars:
	"""
	qk = system.configs[4:7]
	term2 = system.L_ddqddq(qk,qk)*system.ddqk
	term4 = system.L_ddqdq(qk,qk)*system.dqk
	term5 = system.L_dq(qk)
	fk = term2 + term4 - term5
	return fk

# print "torque (trep) = ", f_k(system)[0]

# Display
print "Simulation: dt=%f, tf=%f, runtime=%f s" % (dt, tf, finish-start)
visual.visualize_3d([ visual.VisualItem3D(system, t, q) ])