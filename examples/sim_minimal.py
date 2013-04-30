#!/urs/bin/python
#coding=utf-8
#author=joseph salini
#date=24 jan 2010

from arboris.core import World, simulate, NamedObjectsList
from numpy import array, arange
import os

########################################################
## WORLD BUILDING
########################################################
w = World()

from arboris.robots import human36
human36.add_human36(w)

w.update_dynamic()



########################################################
## INIT
########################################################
joints = w.getjoints()
bodies = w.getbodies()
frames = w.getframes()
shapes = w.getshapes()

joints[0].gpos[0:3,3] = array([.1, .2, .3])

w.update_dynamic()



########################################################
## CONTROLLERS
########################################################
from arboris.controllers import WeightController
w.register(WeightController())


########################################################
## OBSERVERS
########################################################
obs = []

from arboris.observers import PerfMonitor
obs.append(PerfMonitor(True))               # to print simulation time


from arboris.observers import Hdf5Logger
obs.append(Hdf5Logger("sim.h5", mode="w", flat=True))       # to save simulation data in hdf5 file ...

from arboris.observers import PickleLogger
obs.append(PickleLogger("sim.pkl", mode="wb", flat=True))   # ... or in pickle file


from arboris.visu.dae_writer import write_collada_scene
from arboris.visu import pydaenimCom                        # visualize with pydaenim ...
write_collada_scene(w, "scene.dae", flat=True)
obs.append(pydaenimCom("scene.dae", flat=True))

from arboris.visu.visu_collada import write_collada_scene as visu_collada_scene
from arboris.visu.visu_collada import DaenimCom             # ... or with daenim program if installed
visu_collada_scene(w, "scene_daenim.dae", flat=True)
obs.append(DaenimCom("scene_daenim.dae", flat=True))


########################################################
## SIMULATION
########################################################
dt = 5e-3
simulate(w, arange(0, 1., dt), obs)


########################################################
## RESULTS
########################################################

print(obs[0].get_summary())

from arboris.visu.dae_writer import write_collada_animation
write_collada_animation("anim_h5.dae", "scene.dae", "sim.h5")
write_collada_animation("anim_pkl.dae", "scene.dae", "sim.pkl")

from arboris.visu.visu_collada import write_collada_animation as visu_collada_animation
visu_collada_animation("anim_daenim_h5.dae", "scene_daenim.dae", "sim.h5")
visu_collada_animation("anim_daenim_pkl.dae", "scene_daenim.dae", "sim.pkl")

