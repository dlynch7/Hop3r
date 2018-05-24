import trep
from trep import tx,ty,tz,rx,ry,rz
import trep.potentials
import time
import trep.visual as visual
import math
import numpy as np
from math import sin, cos, exp, pi
from scipy.integrate import odeint

dt = 0.01
tf = 10.0

# Set up the system frames
system = trep.System()
system.import_frames([
    # Define the 3 Center Links
    rx('theta1', kinematic=True),[
        tz(-0.35, name='pend1', mass=0.25), [
            tz(-.35), [
                rx('theta2'),[
                    tz(-0.35, name='pend2', mass=0.25), [
                        tz(-.35), [
                            rx('theta3'), [
                                tz(-0.5, name='legConnection'),[
                                    tz(-0.5, mass=0.25, name='COMLeg'),[
                                        tz(-1, name='pend3')]]]]]]]]],
    # Define the 2 Right Links
    ty(1), [
        rx('theta4', kinematic=True),[
            tz(-0.5, mass=0.25, name='COM4'), [
                tz(-0.5),[
                    rx('theta5'), [
                        tz(-0.5, mass=0.25, name="COM5"), [
                            tz(-0.5, name='pend5')]]]]]],
    # Define the 2 Left Links
    ty(-1), [
        rx('theta6', kinematic=True),[
            tz(-0.5, mass=0.25, name='COM6'),[
                tz(-0.5, name='pend6'),[
                    rx('theta7'), [
                        tz(-0.5, mass=0.25, name="COM7"), [
                            tz(-0.5, name='pend7')]]]]]]
])

# Establish gravity
trep.potentials.Gravity(system, name="Gravity")
trep.forces.Damping(system, 0.1)

# Add constraints
trep.constraints.PointToPoint2D(system,'yz','pend5','legConnection')
trep.constraints.PointToPoint2D(system,'yz','pend7','legConnection')

# CENTER LEG:
qarr = np.zeros(system.nQ)
qarr[system.get_config('theta1').index] = -.839
qarr[system.get_config('theta2').index] = 2.09
qarr[system.get_config('theta3').index] = -1.22

# RIGHT LEG:
qarr[system.get_config('theta4').index] = -0.151
qarr[system.get_config('theta5').index] = -0.5

# LEFT LEG
qarr[system.get_config('theta6').index] = 0.151
qarr[system.get_config('theta7').index] = 0.5

system.q = qarr

system.satisfy_constraints()
# system.minimize_potential_energy()
# system.satisfy_constraints(keep_kinematic=True)

# Simulate
q0 = system.q
qk2_0 = system.qk

# control parameters:
a1, a2, a3 = 0.25, 0.0, 0.0
f = 0.6
qf = lambda t: (1-exp(-t))*sin(f*pi*t)
ddqf = lambda t: 2*exp(-t)*f*pi*cos(f*pi*t) - exp(-t)*sin(f*pi*t) - (1-exp(-t))*f*f*pi*pi*sin(f*pi*t)

# return configuration of kinematic configuration variables at time t:
def qk_func(system, t):
    qk2 = list(qk2_0)
    q = qf(t)
    qk2[system.get_config('theta1').k_index] += a1*q
    qk2[system.get_config('theta4').k_index] += a2*q
    qk2[system.get_config('theta6').k_index] += a3*q
    return qk2

# # return acceleration of kinematic configuration variables at time t:
def qk_dd_func(system, t):
    ddqk = [0.0]*len(system.kin_configs)
    ddq = ddqf(t)
    ddqk[system.get_config('theta1').k_index] = a1*ddq
    ddqk[system.get_config('theta4').k_index] = a2*ddq
    ddqk[system.get_config('theta6').k_index] = a3*ddq
    return ddqk


def generate_vi_sim(system, q0):
    # Create and initialize the variational integrator
    mvi = trep.MidpointVI(system)
    mvi.initialize_from_configs(0.0, q0, dt, q0)

    # This is our simulation loop.  We save the results in two lists.
    q = [mvi.q2]
    t = [mvi.t2]
    while mvi.t1 < tf:
        qk2 = qk_func(system, mvi.t2)
        try:
            mvi.step(mvi.t2+dt, (), qk2)
        except trep.ConvergenceError:
            print "VI integration failed at",mvi.t2+dt,"seconds"
            break
        q.append(mvi.q2)
        t.append(mvi.t2)
    return t,q


def generate_cont_sim(system, q0):
    n = len(system.configs)

    def f(x, t):
        q = x.tolist()[:n]
        dq = x.tolist()[n:]
        system.set_state(q=q, dq=dq, ddqk=qk_dd_func(system, t), t=t)
        system.f()  
        ddq = system.ddq
        return np.concatenate((dq, ddq))

    x0 = np.concatenate((system.q, system.dq))
    # Generate a list of time values we want to calculate the configuration for.
    t = [dt*i for i in range(int(tf/dt))]
    # Run the numerical integrator
    x = odeint(f, x0, t)
    # Extract the configuration out of the resulting trajectory.
    q = [xi.tolist()[:n] for xi in x]
    return t,q

# t,q = generate_vi_sim(system, q0)
tc,qc = generate_cont_sim(system, q0)

# Display
visual.visualize_3d([ visual.VisualItem3D(system, tc, qc) ],
                    camera_pos=[ 5.76000000e+00, -9.30731567e-17, -1.28000000e+00])
