# import libraries

from shared import *
# the main class to do the simulation
from swarm_simulation import World

# the main control loop

# initialize the simulation
var = Shared()
world = World(var)

t = 0.
counter = 0

# starts a simulation
while True:
    world.stepSimulation()
