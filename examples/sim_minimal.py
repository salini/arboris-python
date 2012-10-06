#!/urs/bin/python
#coding=utf-8
#author=joseph salini
#date=24 jan 2010

from arboris.core import World, simulate, NamedObjectsList
from numpy import array, arange


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
from arboris.visu_collada import write_collada_scene
write_collada_scene(w, "scene.dae", flat=True)

from arboris.observers import PerfMonitor
obs = []
obs.append(PerfMonitor(True))

try:
    from arboris.observers import Hdf5Logger
    obs.append(Hdf5Logger("sim.h5", mode="w", flat=True))
except ImportError:
    print("WARNING: cannot use Hdf5Logger. h5py may not be installed.")
    print("This module is not mandatory, but useful to save simulation data")

from arboris.observers import PickleLogger
obs.append(PickleLogger("sim.pkl", mode="wb", flat=True))

try:
    from arboris.observers import DaenimCom
    obs.append(DaenimCom("scene.dae", flat=True))
except:
    print("WARNING: cannot use DaenimCom. daenim may not be installed.")
    print("This program is not mandatory, but useful for visualization")

try:
    from arboris.observers import VPythonObserver
    obs.append(VPythonObserver())
except:
    print("WARNING: cannot use VPythonObserver. vpython may not be installed.")
    print("This module is not mandatory, but useful for visualization")


########################################################
## SIMULATION
########################################################
dt = 5e-3
simulate(w, arange(0, 1., dt), obs)


########################################################
## RESULTS
########################################################

print(obs[0].get_summary())

from arboris.visu_collada import write_collada_animation
try:
    write_collada_animation("anim_h5.dae", "scene.dae", "sim.h5")
    write_collada_animation("anim_pkl.dae", "scene.dae", "sim.pkl")
except:
    pass

