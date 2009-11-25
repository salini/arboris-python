#coding=utf-8
"""A set of WorldObservers.
"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from core import WorldObserver
from abc import ABCMeta, abstractmethod, abstractproperty
from numpy import dot, array, eye, linalg, vstack, hstack, zeros, diag
from time import time as _time
from massmatrix import principalframe

import logging
logging.basicConfig(level=logging.DEBUG)

import h5py

class EnergyMonitor(WorldObserver):
    """Compute and store the world energy at each time step.
    
    **Example:**

        >>> from arboris.core import ObservableWorld, simulate
        >>> from arboris.robots.simplearm import add_simplearm
        >>> w = ObservableWorld()
        >>> obs = EnergyMonitor(w)
        >>> w.observers.append(obs)
        >>> add_simplearm(w)
        >>> simulate(w, [0,1,2])
        >>> #obs.plot()

    """
    
    def __init__(self, world):
        self._world = world

    def init(self):
        self.time = []
        self.kinetic_energy = []
        self.potential_energy = []
        self.mechanichal_energy = []
        self._com_pos = {}
        self._bodies = self._world.ground.iter_descendant_bodies
        for body in self._bodies():
            self._com_pos[body]  = principalframe(body.mass)[:,3]

    def update(self, dt):
        self.time.append(self._world.current_time)
        Ec = dot(self._world.gvel, 
                 dot(self._world.mass, self._world.gvel) )/2.
        self.kinetic_energy.append(Ec)
        Ep = 0.
        for body in self._bodies():
            h = dot( dot(body.pose, self._com_pos[body])[0:3], self._world.up)
            Ep += body.mass[3,3] * h
        Ep *= 9.81
        self.potential_energy.append(Ep)
        self.mechanichal_energy.append(Ec+Ep)

    def plot(self):
        """Plot the energy evolution.
        """
        from pylab import plot, show, legend, xlabel, ylabel, title
        plot(self.time, self.kinetic_energy)
        plot(self.time, self.potential_energy)
        plot(self.time, self.mechanichal_energy)
        legend(('kinetic','potential','mechanical'))
        title('Energy evolution')
        xlabel('time (s)')
        ylabel('energy (J)')
        show()


class PerfMonitor(WorldObserver):
    """

    **Example:**

        >>> from arboris.core import ObservableWorld, simulate
        >>> from arboris.robots.simplearm import add_simplearm
        >>> w = ObservableWorld()
        >>> obs = PerfMonitor(w)
        >>> w.observers.append(obs)
        >>> add_simplearm(w)
        >>> simulate(w, [0,1,2])
        >>> print obs.get_summary() #doctest: +ELLIPSIS
        total computation time (s): ...
        min computation time (s): ...
        mean computation time (s): ...
        max computation time (s): ...
        >>> #obs.plot()

    """
    def __init__(self, world, log = False):
        if log:
            self._logger = logging.getLogger(self.__class__.__name__)
        else:
            self._logger = None
        self._last_time = None
        self._computation_time = []
        self._world = world

    def init(self):
        self._last_time = _time()

    def update(self, dt):
        current_time = _time()
        self._computation_time.append(current_time - self._last_time)
        self._last_time = current_time
        if self._logger is not None:
            self._logger.info('current time (s): %.3f', 
                              self._world.current_time)

    def plot(self):
        from pylab import plot, show, xlabel, ylabel, title
        plot(self._computation_time)
        title('Computation time for each simulation time step')
        xlabel('simulation time step')
        ylabel('computation time (s)')
        show()

    def get_summary(self):
        total = sum(self._computation_time)
        return """total computation time (s): {0}
min computation time (s): {1}
mean computation time (s): {2}
max computation time (s): {3}""".format(
    total, 
    min(self._computation_time), 
    total/len(self._computation_time), 
    max(self._computation_time))


class Hdf5Logger(WorldObserver):
    """An observer that saves the simulation data in an hdf5 file.
    """
    def __init__(self, world, nb_steps, filename, group = "/",
                    mode = 'a', save_viewer_data = True , 
		    save_dyn_model = False):
        # simulation data
        self._world = world
        self._nb_steps = nb_steps
        # hdf5 file handlers
        self._file = h5py.File(filename, mode)
        self._root = self._file
	for g in group.split('/'):
            if len(g) > 0:
                self._root = self._root.require_group(g)
        # what to save
        self._save_viewer_data = save_viewer_data
        self._save_dyn_model = save_dyn_model
 
    def init(self):
        """Create the datasets.
        """
        self._current_step = 0
        self._root.require_dataset("timeline", (self._nb_steps,), 'f8')
        
        if self._save_viewer_data:
            self._matrix = self._world.getbodies()[1:]
            for m in self._matrix:
                d = self._root.require_dataset(m.name, 
				               (self._nb_steps, 4,4),
					       'f8')
                d.attrs["ArborisViewerType"] = "matrix"
            #self._wrench = [] #TODO: how to get wrenches in the simulation
            #for w in self._wrench:
            #    d = self._root.require_dataset(w.name, (self._nb_steps, 6), 'f8')
            #    d.attrs["ArborisViewerType"] = "wrench"
            #    dset.attrs["ArborisViewerParent"] = w.parent.name
        if self._save_dyn_model:
            ndof = self._world.ndof
            #self._root.require_dataset("gpos", (self._nb_steps, 4, 4), 'f8') #TODO: change the allocated space
            self._root.require_dataset("gvel", (self._nb_steps, ndof), 'f8')
            self._root.require_dataset("mass", (self._nb_steps, ndof, ndof), 'f8')
            self._root.require_dataset("nleffects", (self._nb_steps, ndof, ndof), 'f8')

    def update(self, dt):
        """Save the current data (state...).
        """
	assert self._current_step <= self._nb_steps
        self._root["timeline"][self._current_step] = self._world._current_time
        if self._save_viewer_data:
            for m in self._matrix:
                self._root[m.name][self._current_step,:,:] = m.pose
            #for w in self._wrench:
            #    self._root[w.name][self._current_step,:] = w.value #TODO: check if value?!?
        if self._save_dyn_model:
            #self._root["gpos"][self._current_step,:,:] = self._world.ground.childrenjoints[0].frames[1].pose # gpos is NOT that
            self._root["gvel"][self._current_step,:] = self._world.gvel
            self._root["mass"][self._current_step,:,:] = self._world.mass.copy()
            self._root["nleffects"][self._current_step,:,:] = self._world.nleffects.copy()
        self._current_step += 1

    def finish(self):
        self._file.close()
