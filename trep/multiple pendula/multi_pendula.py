import trep
from trep import tx,ty,tz,rx,ry,rz
import trep.potentials
import time
import trep.visual as visual
import math
import numpy as np

dt = 0.01
tf = 10.0

def simulate_system(system):
    # Now we'll extract the current configuration into a tuple to use as
    # initial conditions for a variational integrator.
    q0 = system.q

    # Create and initialize the variational integrator
    mvi = trep.MidpointVI(system)
    mvi.initialize_from_configs(0.0, q0, dt, q0)
    system.satisfy_constraints(tolerance=1e-1)

    # This is our simulation loop.  We save the results in two lists.
    q = [mvi.q2]
    t = [mvi.t2]
    while mvi.t1 < tf:
        mvi.step(mvi.t2+dt)
        q.append(mvi.q2)
        t.append(mvi.t2)

    return (t,q)

# Set up the system frames
system = trep.System()
system.import_frames([
    rx('theta1'),[
        tz(-0.7, mass=1, name='pend1'),[
            rx('theta2'),[
                tz(-0.8, mass=1, name='pend2'),[
                    rx('theta3'),[
                        tz(-0.5, name='legConnection'),[
                            tz(-1.5, mass=1, name='pend3')]]]]]],
    ty(1), [
        rx('theta4'),[
            tz(-1, mass=1, name='pend4'),[
                rx('theta5'),[
                    tz(-1, mass=1, name='pend5')]]]],
    ty(-1), [
        rx('theta6'),[
            tz(-1, mass=1, name='pend6'),[
                rx('theta7'),[
                    tz(-1, mass=1, name='pend7')
                ]
            ]
        ]
    ]
])

# Establish gravity
trep.potentials.Gravity(system, name="Gravity")

# Input Torque
# trep.forces.ConfigForce(system, 'theta4', 'tau1')
# tau1 = -2
# system.u = tau1
# system.f()

# Assign values to the system initial configuration
pie = math.pi
system.q = [pie/3,-2*pie/3,pie/3,0,-pie/2,0,pie/2]

# Add constraints
trep.constraints.PointToPoint2D(system,'yz','pend5','legConnection')
trep.constraints.PointToPoint2D(system,'yz','pend7','legConnection')

# Simulate
start = time.clock()
(t, q) = simulate_system(system)
finish = time.clock()

# Display
print "Simulation: dt=%f, tf=%f, runtime=%f s" % (dt, tf, finish-start)
visual.visualize_3d([ visual.VisualItem3D(system, t, q) ])
