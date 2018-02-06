# Simulate an arbitrarily long pendulum.

import sys
import math
import time
import trep
import trep.potentials
import trep.visual as visual
import numpy as np

links = 1
dt = 0.01
tf = 10.0

def simulate_system(system, tau=0.0):
	# Now we'll extract the current configuration into a tuple to use as
	# initial conditions for a variational integrator
	q0 = system.q

	# Create and initialize the variational integrator
	mvi = trep.MidpointVI(system)
	mvi.initialize_from_configs(0.0, q0, dt, q0)

	# This is our simulation loop. We save the results in two lists.
	q = [mvi.q2]
	t = [mvi.t2]
	while mvi.t1 < tf:
		mvi.step(mvi.t2+dt, u1=[tau])
		q.append(mvi.q2)
		t.append(mvi.t2)

	return (t,q)

def make_pendulum(num_links, kinematic=False):
	"""
	make_pendulum(num_links) -> System

	Create a pendulum system with num_links.
	"""
	def add_level(frame, link=0, kinematic=False):
		"""
		Recursively add links to a system by attaching a new link to
		the specified frame.
		"""
		if link == num_links:
			return

		# Create a rotation for the pendulum.
		# The first argument is the name of the frame, the second is
		# the transformation type, and the third is the name of the
		# configuration variable that parameterizes the
		# transformation.
		child = trep.Frame(frame, trep.RX, "link-%d" % link, "link-%d" % link, kinematic=kinematic)

		# Move down to create the length of the pendulum link.
		child = trep.Frame(child, trep.TZ, -1)
		# Add mass to the end of the link (only a point mass, no
		# rotational intertia
		child.set_mass(1.0)

		add_level(child, link+1)

	# Create a new system, add the pendulum links, and rotate the top
	# pendulum.
	system = trep.System()
	trep.potentials.Gravity(system, name="Gravity")
	add_level(system.world_frame, kinematic=kinematic)
	system.get_config("link-0").q = math.pi/2.
	return system


# Create
system = make_pendulum(links)

# Add forcing:
trep.forces.ConfigForce(system, 'link-0', 'tau')

# Now let's initialize the system:
# system.q = math.pi/3.
# tau = 9.8*1*math.sin(system.q)
system.q = np.random.uniform(-np.pi,np.pi)
tau = np.random.uniform(-10,10)

# Now let's check that trep can properly calculate the acceleration
system.u = tau
system.f()
print "acceleration (trep) = ", system.ddq[0]
print "acceleration (hand) = ", tau-9.8*math.sin(system.q)

# Now let's see if we can calculate the inverse dynamics:
system2 = make_pendulum(links, True)
# let's set the config, velocity, and acceleration:
# system2.q = math.pi/3.
# alpha = 1.2
system.q = np.random.uniform(-np.pi,np.pi)
alpha = np.random.uniform(-10,10)

system2.ddqk = alpha
system2.f()


def f_k(system):
	"""
	Implement Eq. 3.30b from Elliot's thesis...not currently handling
	constraints or dynamic vars:
	"""
	qk = system.configs[0]
	term2 = system.L_ddqddq(qk,qk)*system.ddqk
	term4 = system.L_ddqdq(qk,qk)*system.dqk
	term5 = system.L_dq(qk)
	fk = term2 + term4 - term5
	return fk

print "torque (trep) = ", f_k(system2)[0]
print "torque (hand) = ", 9.8*math.sin(system2.q) + alpha



# Simulate
start = time.clock()
(t, q) = simulate_system(system, tau)
finish = time.clock()

# Display
print "%d-link pendulum, dt=%f, tf=%f...runtime=%f s" % (links, dt, tf, finish-start)
visual.visualize_3d([ visual.VisualItem3D(system, t, q) ])
