#!/usr/bin/env python

"""
This example show how to add mesh from a collada file to another,
in order to display complex meshes during visualization.
"""

from arboris.core import World
from arboris.robots import simplearm
from arboris.homogeneousmatrix import transl

##### Create world
w = World()

simplearm.add_simplearm(w, with_shapes=True)


##### Init
joints = w.getjoints()

for i in range(3):
    joints[i].gpos[:] = 0.5

w.update_dynamic()


##### Add ctrl
from arboris.controllers import WeightController

w.register(WeightController())


##### Add observers
from arboris.observers import PerfMonitor, Hdf5Logger

from arboris.visu.dae_writer import write_collada_scene, write_collada_animation, add_shapes_to_dae
from arboris.visu            import pydaenimCom
flat = False
write_collada_scene(w, "./scene.dae", flat=flat)

shapes_info = [
    ["Arm"        , "./dae/icub_simple.dae#head"],                                 # parent frame, child shape node id
    ["Forearm"    , "./dae/icub_simple.dae#head", transl(0,0.3,0)],                # parent frame, child shape node id, H_frame_shape
    ["EndEffector", "./dae/icub_simple.dae"     , transl(0,0.3,0), (0.3,0.3,0.3)], # parent frame, child shape file   , H_frame_shape, shape_scale
]
add_shapes_to_dae("./scene.dae", shapes_info)     # add argument output_file="out.dae" if you want to keep original dae file


obs = []
pobs = PerfMonitor(True)
dobs = pydaenimCom("./scene.dae", flat=flat)
h5obs = Hdf5Logger("sim.h5", mode="w", flat=flat)
obs.append(pobs)
obs.append(dobs)
obs.append(h5obs)



##### Simulate
from arboris.core import simulate
from numpy import arange

timeline = arange(0, 5., 0.005)
simulate(w, timeline, obs)


##### Results

print pobs.get_summary()

write_collada_animation("anim.dae", "scene.dae", "sim.h5")



