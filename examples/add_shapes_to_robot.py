#!/usr/bin/env python

"""
This example show how to add mesh from a collada file to another,
in order to display complex meshes during visualization.
"""

from arboris.core import World
from arboris.robots import icub

##### Create world
w = World()

icub.add(w, create_shapes=False)
w._up[:] = [0,0,1]


##### Add ctrl
from arboris.controllers import WeightController

w.register(WeightController())


##### Add observers
from arboris.observers import PerfMonitor, Hdf5Logger

from arboris.visu.dae_writer import write_collada_scene, write_collada_animation, add_shapes_to_dae
from arboris.visu import wsDaenimCom
flat = False
write_collada_scene(w, "./scene.dae", flat=flat)
add_shapes_to_dae("./scene.dae", "./icub_simple.dae")

obs = []

pobs = PerfMonitor(True)
dobs = wsDaenimCom("./scene.dae", flat=flat)
h5obs = Hdf5Logger("sim.h5", mode="w", flat=flat)
obs.append(pobs)
obs.append(dobs)
obs.append(h5obs)



##### Simulate
from arboris.core import simulate
from numpy import arange

timeline = arange(0, 1.03, 0.005)
simulate(w, timeline, obs)


##### Results

print pobs.get_summary()

write_collada_animation("anim.dae", "scene.dae", "sim.h5")



