#coding = utf-8
#author Joseph Salini
#date 24 January 2010

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
write_collada_scene(w, "current_scene.dae", flat=True)

from arboris.observers import PerfMonitor, Hdf5Logger, DaenimCom
obs = []
obs.append(PerfMonitor(True))

obs.append(Hdf5Logger("current_anim.h5", mode="w", flat=True))
#obs.append(DaenimCom(r"C:\Program Files\daenim\daenim.exe", "current_scene.dae")) #for Windows
obs.append(DaenimCom(r"daenim", "current_scene.dae")) #for Linux


########################################################
## SIMULATION
########################################################
i=0
for f in w.getframes():
    if f.name is None:
        f.name = "unnamed_frame_"+str(i)
        i+=1

dt = 2e-2
simulate(w, arange(0, 2., dt), obs)


########################################################
## RESULTS
########################################################

print obs[0].get_summary()



