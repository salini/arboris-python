=============
Visualization
=============

The visualisation of a simulation is still a work
in progress. However it is quite useable right now.

Scene and animation are saved in collada file.
Furthermore, there are 2 possibilities to visualize the world and the simulation.


Save scene and animation in a collada file
==========================================

Save world in collada scene
---------------------------

You can save the world in a collada file when this one has been completely
initialized.

>>> from arboris.core import World
>>> from arboris.robots import simplearm
>>> from arboris.visu.dae_writer import write_collada_scene
>>>
>>> w = World()
>>> simplearm.add_simplearm(w)
>>>
>>> write_collada_scene(w, "scene.dae", flat=True)

Of course, modifications of the world after this command are not taken into
account in this file.

The ``flat`` argument indicates how you want to save the world structure.
If ``False``, it keeps the tree structure of the world;
If ``True``, the structure is flatten, and bodies are considered independent.


Save simulation results in collada animation
--------------------------------------------

First of all, you have to save a collada scene, as described above.
Then you have to an obsever to save the simulation data. To do so, use the
``PickleLogger`` or the ``Hdf5Logger``, then simulate

>>> from arboris.observers import PickleLogger, Hdf5Logger
>>>
>>> obs = []
>>> obs.append(PickleLogger("sim.pkl", flat=True))  # this one
>>> obs.append(Hdf5Logger("sim.h5", flat=True))     # or this one
>>>
>>> simulate(w, timeline, obs)

Be sure that the ``flat`` argument is the same that in the ``collada_write_scene``.
When the simulation is finished, you can write a collada animation

>>> from arboris.visu.dae_writer import write_collada_animation
>>>
>>> write_collada_animation("anim_pkl.dae", "scene.dae", "sim.pkl") # this one
>>> write_collada_animation("anim_h5.dae", "scene.dae", "sim.h5")   # or this one






Visualize with Daenim
=====================

Unlike Vpython, `daenim <http://github.com/sbarthelemy/daenim>`_ is not a
python module.
It is a C++ program `based on OpenSceneGraph <www.openscenegraph.org>`_ that can
read collada file and can communicate with arboris-python to display
the running simulation.
Of course, the following does not work if daenim is not installed on your
computer.


View the world
--------------

The current world is shown with daenim, through the function ``view`` as follows

>>> from arboris.core import World
>>> from arboris.robots import simplearm
>>> from arboris.visu.visu_collada import write_collada_scene, view
>>>
>>> w = World()
>>> simplearm.add_simplearm(w)
>>>
>>> view(w)


View simulation
---------------

Arboris-python can update the daenim visualization by communicating with ports.
The communication is possible through the observer ``DaenimCom``

>>> from arboris.visu.visu_collada import DaenimCom
>>>
>>> obs = []
>>> obs.append(DaenimCom())
>>>
>>> simulate(w, timeline, obs)


If you have already saved a collada scene file that represent the current world,
it can be used in argument (the function above create a temporary collada file)

>>> write_collada_scene(w, "scene.dae", flat=True)
>>>
>>> obs.append(DaenimCom("scene.dae", flat=True))

Be sure that the flat argument is the same.


View saved collada file (scene & animation)
-------------------------------------------

You can display a scene or replay an animation saved in a collada file.

>>> from arboris.visu.visu_collada import view
>>>
>>> view("scene.dae")
>>> view("anim.dae")
>>> view("scene.dae", "sim.h5")     # or "sim.pkl"

Note that you can directly call daenim in bash


.. code-block:: bash

   daenim scene.dae
   daenim anim.dae


