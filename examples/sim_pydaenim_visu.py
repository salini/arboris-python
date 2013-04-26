#!/urs/bin/python
#coding=utf-8
#author=joseph salini
#date=25 april 2013

from arboris.core import World, simulate, NamedObjectsList
from numpy import array, arange
import os

########################################################
## WORLD BUILDING
########################################################
w = World()

from arboris.robots import simplearm
simplearm.add_simplearm(w, with_shapes=True)

w.update_dynamic()



########################################################
## INIT
########################################################
joints = w.getjoints()
bodies = w.getbodies()
frames = w.getframes()
shapes = w.getshapes()

for i in range(3):
    joints[i].gpos[:] = 0.5

w.update_dynamic()



########################################################
## CONTROLLERS
########################################################
from arboris.controllers import WeightController
w.register(WeightController())


########################################################
## OBSERVERS
########################################################
from arboris.visu.dae_writer import write_collada_scene
write_collada_scene(w, "scene.dae", flat=True)

from arboris.observers import PerfMonitor
obs = []
obs.append(PerfMonitor(True))

from arboris.observers import PickleLogger
obs.append(PickleLogger("sim.pkl", mode="wb", flat=True))

from arboris.visu import pydaenimCom
obs.append(pydaenimCom("scene.dae", flat=True))


########################################################
## SIMULATION
########################################################
dt = 5e-3
simulate(w, arange(0, 5., dt), obs)


########################################################
## RESULTS
########################################################

print(obs[0].get_summary())

from arboris.visu.dae_writer import write_collada_animation
write_collada_animation("anim_pkl.dae", "scene.dae", "sim.pkl")



