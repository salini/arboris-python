#coding=utf-8
"""A set of Observers.
"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from numpy import dot, zeros

from arboris.core import Observer, MovingSubFrame, name_all_elements
from arboris.massmatrix import principalframe

try:
    from pylab import plot, show, legend, xlabel, ylabel, title
except ImportError:
    pass

import pickle as pkl
try:
    import h5py
except ImportError:
    pass

import tempfile
import arboris._visu
from arboris.visu_collada import get_daenim_path, write_collada_scene
try:
    from arboris.visu_vpython import VPythonDriver
except ImportError:
    pass



from time import time as _time, sleep
import socket
import subprocess, shlex
import logging
logging.basicConfig(level=logging.DEBUG)

import struct
import os


class EnergyMonitor(Observer):
    """Compute and store the world energy at each time step.

    **Example:**

        >>> from arboris.core import World, simulate
        >>> from arboris.robots.simplearm import add_simplearm
        >>> w = World()
        >>> observers = [EnergyMonitor()]
        >>> add_simplearm(w)
        >>> simulate(w, [0,1,2], observers)
        >>> #obs.plot()

    """
    def __init__(self, name=None):
        Observer.__init__(self, name)
        self._world = None
        self.time = []
        self.kinetic_energy = []
        self.potential_energy = []
        self.mechanichal_energy = []
        self._com_pos = {}
        self._bodies = None

    def init(self, world, timeline):
        self._world = world
        self.time = []
        self.kinetic_energy = []
        self.potential_energy = []
        self.mechanichal_energy = []
        self._com_pos = {}
        self._bodies = self._world.ground.iter_descendant_bodies
        for body in self._bodies():
            self._com_pos[body]  = principalframe(body.mass)[:, 3]

    def update(self, dt):
        self.time.append(self._world.current_time)
        Ec = dot(self._world.gvel,
                 dot(self._world.mass, self._world.gvel) )/2.
        self.kinetic_energy.append(Ec)
        Ep = 0.
        for body in self._bodies():
            h = dot( dot(body.pose, self._com_pos[body])[0:3], self._world.up)
            Ep += body.mass[3, 3] * h
        Ep *= 9.81
        self.potential_energy.append(Ep)
        self.mechanichal_energy.append(Ec+Ep)

    def finish(self):
        pass

    def plot(self):
        """Plot the energy evolution.
        """
        try:
            plot(self.time, self.kinetic_energy)
            plot(self.time, self.potential_energy)
            plot(self.time, self.mechanichal_energy)
            legend(('kinetic', 'potential', 'mechanical'))
            title('Energy evolution')
            xlabel('time (s)')
            ylabel('energy (J)')
            show()
        except NameError:
            raise ImportError("pylab cannot be imported. Please check that pylab is installed on your computer.")


class PerfMonitor(Observer):
    """

    **Example:**

        >>> from arboris.core import World, simulate
        >>> from arboris.robots.simplearm import add_simplearm
        >>> w = World()
        >>> obs = PerfMonitor()
        >>> add_simplearm(w)
        >>> simulate(w, [0,1,2], [obs])
        >>> print obs.get_summary() #doctest: +ELLIPSIS
        total computation time (s): ...
        min computation time (s): ...
        mean computation time (s): ...
        max computation time (s): ...
        >>> #obs.plot()

    """
    def __init__(self, log=False, name=None):
        Observer.__init__(self, None)
        if log:
            self._logger = logging.getLogger(self.__class__.__name__)
        else:
            self._logger = None
        self._world = None
        self._last_time = None
        self._computation_time = []

    def init(self, world, timeline):
        self._world = world
        self._last_time = _time()

    def update(self, dt):
        current_time = _time()
        self._computation_time.append(current_time - self._last_time)
        self._last_time = current_time
        if self._logger is not None:
            self._logger.info('current time (s): %.3f',
                              self._world.current_time)

    def finish(self):
        pass

    def plot(self):
        try:
            plot(self._computation_time)
            title('Computation time for each simulation time step')
            xlabel('simulation time step')
            ylabel('computation time (s)')
            show()
        except NameError:
            raise ImportError("pylab cannot be imported. Please check that pylab is installed on your computer.")

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



class _SaveLogger(Observer):
    """ TODO
    """
    def __init__(self, save_state=False, save_transforms=True,
                       flat=False, save_model=False, name=None):
        Observer.__init__(self, name)
        # what to save
        self._save_state = save_state
        self._save_transforms = save_transforms
        self._flat = flat
        self._save_model = save_model

    def init(self, world, timeline):
        self._world = world
        self._nb_steps = len(timeline)-1
        self._current_step = 0

        self._root = {}
        self._timeline = zeros(self._nb_steps)
        self._root["timeline"] = self._timeline
        if self._save_state:
            self._gpositions = {}
            self._root['gpositions'] = self._gpositions
            self._gvelocities = {}
            self._root['gvelocities'] = self._gvelocities
            name_all_elements(self._world.getjoints(), check_unicity=True)
            for j in self._world.getjoints():
                self._gpositions[j.name] = zeros((self._nb_steps,)+j.gpos.shape[:])
                self._gvelocities[j.name] = zeros((self._nb_steps, j.ndof))

        if self._save_transforms:
            self._arb_transforms = {}
            self._transforms = {}
            self._root['transforms'] = self._transforms
            if self._flat:
                name_all_elements(self._world.iterbodies(), True)
                for b in self._world.iterbodies():
                    self._arb_transforms[b.name] = b
            else:
                nonflatlist = [j.frames[1] for j in self._world.getjoints()]
                name_all_elements(nonflatlist, True)
                for j in  self._world.getjoints():
                    self._arb_transforms[j.frames[1].name] = j
            name_all_elements(self._world.itermovingsubframes(), True)
            for f in self._world.itermovingsubframes():
                self._arb_transforms[f.name] = f
            for k in self._arb_transforms.keys():
                self._transforms[k] = zeros((self._nb_steps, 4,4))

        if self._save_model:
            ndof = self._world.ndof
            self._model = {}
            self._root['model'] = self._model
            self._model["gvel"] = zeros((self._nb_steps, ndof))
            self._model["mass"] = zeros((self._nb_steps, ndof, ndof))
            self._model["admittance"] = zeros((self._nb_steps, ndof, ndof))
            self._model["nleffects"]  = zeros((self._nb_steps, ndof, ndof))
            self._model["gforce"] = zeros((self._nb_steps, ndof))

    def update(self, dt):
        """Save the current data (state...).
        """
        assert self._current_step <= self._nb_steps
        self._root["timeline"][self._current_step] = self._world.current_time
        if self._save_state:
            for j in self._world.getjoints():
                self._gpositions[j.name][self._current_step] = j.gpos
                self._gvelocities[j.name][self._current_step] = j.gvel
        if self._save_transforms:
            for k, v in self._arb_transforms.items():
                if isinstance(v, MovingSubFrame):
                    if self._flat:
                        pose = v.pose
                    else:
                        pose = v.bpose
                else:
                    pose = v.pose
                self._transforms[k][self._current_step, :, :] = pose
        if self._save_model:
            self._model["gvel"][self._current_step, :]    = self._world.gvel
            self._model["mass"][self._current_step, :, :] = self._world.mass
            self._model["admittance"][self._current_step, :, :] = \
                    self._world.admittance
            self._model["nleffects"][self._current_step, :, :] = \
                    self._world.nleffects
            self._model["gforce"][self._current_step, :] = self._world.gforce
        self._current_step += 1


class PickleLogger(_SaveLogger):
    """ TODO
    """
    def __init__(self, filename, mode='wb', save_state=False,
                 save_transforms=True, flat=False, save_model=False, protocol=0, name=None):
        _SaveLogger.__init__(self, save_state, save_transforms, flat, save_model, name)
        self._filename = filename
        self._mode = mode
        self._protocol = protocol

    def finish(self):
        with open(self._filename, self._mode) as _file:
            pkl.dump(self._root, _file, self._protocol)


class Hdf5Logger(_SaveLogger):
    """An observer that saves the simulation data in an hdf5 file.

    :param filename: name of the output hdf5 file
    :type filename: str
    :param group: subgroup within the hdf5 file. Defaults to "/"
    :type group: str
    :param mode: mode used to open the hdf5 file. Can be 'w' or 'a' (default)
    :type mode: str
    :param save_state: toggle the write of the ``gpos`` and ``gvel`` groups
    :type save_state: bool
    :param save_transforms: toggle the write of the ``transforms`` group
    :type save_transforms: bool
    :param flat: whether to save body of joint poses in the ``transforms`` group
    :type flat: bool
    :param save_model: toggle the write of ``model`` group
    :type save_model: bool

    All the simulation data lies in a single user-chosen group, that is denoted
    ``root`` in the following, and which defaults to ``/``. All the data are in
    S.I. units (radians, meters, newtons and seconds).

    The hdf5 file has the following layout::

      root/timeline (nsteps,)
      root/gpositions/
      root/gvelocities/
      root/model/
      root/transforms/

    The ``gvelocities`` group contains the generalized velocities of the
    joints::

      NameOfJoint0 (nsteps, joint0.ndof)
      NameOfJoint1 (nsteps, joint1.ndof)
      ...

    while the ``gpositions`` group contains their generalized positions.

    The ``model`` group contains the matrices from the dynamical model::

      gvel (nsteps, ndof)
      gforce (nsteps, ndof)
      mass (nsteps, ndof, ndof)
      nleffects (nsteps, ndof, ndof)
      admittance (nsteps, ndof, ndof)

    The ``transforms`` group contains homogeneous transformations,
    useful for viewing an animation of the simulation.

      NameOfTransform0 (nsteps, 4, 4)
      NameOfTransform1 (nsteps, 4, 4)
      ...

    The name and value of the transforms depends on the ``flat`` parameter.
    If ``flat`` is True, then there is one transform per body, named after the
    body and whose value is the body absolute pose (``Body.pose``).
    If ``flat`` is False, there is one transform per joint, whose value is
    ``Joint.pose`` and whose name is taken from the joint second frame
    (``Joint.frames[1].name``).

    """
    def __init__(self, filename, group="/", mode='a', save_state=False,
                 save_transforms=True, flat=False, save_model=False, name=None):
        _SaveLogger.__init__(self, save_state, save_transforms, flat, save_model, name)
        try:
            self._file = h5py.File(filename, mode)  # hdf5 file handlers
        except NameError:
            raise ImportError("h5py cannot be imported. Please check that h5py is installed on your computer.")
        self._h5_root = self._file
        for g in group.split('/'):
            if g:
                self._h5_root = self._h5_root.require_group(g)

    def finish(self):
        dset = self._h5_root.require_dataset("timeline", (self._nb_steps,), "f8")
        dset[:] = self._root["timeline"]
        group_elements = list(self._root.keys())
        group_elements.remove("timeline")
        for elem in group_elements:
            g = self._h5_root.require_group(elem)
            for k, v in self._root[elem].items():
                dset = g.require_dataset(k, (self._nb_steps, 4,4), "f8")
                dset[:] = v

        self._file.close()



class SocketCom(Observer):
    """
    """
    def __init__(self, host="127.0.0.1", port=5000, timeout=3, name=None):
        Observer.__init__(self, name)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.settimeout(timeout)
        max_port = port + 50
        while port < max_port:
            try:
                self.s.bind((host, port))
                break
            except socket.error:
                port += 1
                print("change port!!!")
        self.host = host
        self.port = port
        self.conn = None
        self.addr = None

    def init(self, world, timeline):
        self.s.listen(1)
        try:
            self.conn, self.addr = self.s.accept()
            print('Connected by', self.addr)
        except socket.error:
            print("Connection error: no connection occurs")

    def finish(self):
        try:
            self.conn.send(b"close connection")
            self.s.close()
        except socket.error:
            pass


class DaenimCom(SocketCom):
    """
    """
    def __init__(self, daefile=None, daenim_path=None, host="127.0.0.1", port=5000, timeout=3, \
                 options = "", precision=5, flat=False, name=None):
        """
        """
        SocketCom.__init__(self, host, port, timeout, name)
        
        self.daefile = daefile

        if daenim_path is None:
            daenim_path = get_daenim_path()

        self.app_call = \
              [daenim_path, daefile, "-socket", self.host, str(self.port)] + \
               shlex.split(options)
        self.precision = precision
        self.flat = flat
        self.world = None

    def init(self, world, timeline):
        """
        """
        if self.daefile is None:
            tmp_daefile = tempfile.mkstemp(suffix='scene.dae', text=True)[1]
            write_collada_scene(world, tmp_daefile, flat=self.flat)
            self.app_call[1] = tmp_daefile

        try:
            subprocess.Popen(self.app_call)
        except OSError:
            raise OSError("Cannot find program "+self.app_call[0]+". Please check where the arboris viewer (daenim) is installed on your computer.")
        SocketCom.init(self, world, timeline)
        self.world = world
        sleep(.1)
        if self.flat:
            name_all_elements(self.world.getbodies(), True)
        else:
            nonflatlist = [j.frames[1] for j in self.world.getjoints()]
            name_all_elements(nonflatlist, True)
        name_all_elements(self.world.itermovingsubframes(), True)

    def update(self, dt):
        """
        """
        if self.flat:
            msg = b""
            for b in self.world.getbodies():
                H = b.pose
                msg += self.packing_transform(b.name, H)
        else:
            msg = ""
            for j in self.world.getjoints():
                H = j.pose
                msg += self.packing_transform(j.frames[1].name, H)

        for f in self.world.itermovingsubframes():
            H = f.pose if self.flat else f.bpose
            msg += self.packing_transform(f.name, H)


        msg += b"update done"
        try:
            self.conn.send(msg)
            self.conn.recv(2)
        except socket.error:
            print("connection lost")


    def packing_transform(self, _name, _H):
        bytes_name = _name.encode()     #this convert is made to have compatibility between python2.x and python3.x
        res = struct.pack('64s12f', bytes_name, *[float(v) for v in _H[0:3, :].reshape(12)])
        return res


class VPythonObserver(Observer):
    def __init__(self, scale=1, options=None, flat=False, color_generator=None, name=None):
        Observer.__init__(self, None)
        try:
            self.driver = VPythonDriver(scale, options)
        except NameError:
            raise ImportError("vpython cannot be imported. Please check that vpython is installed on your computer.")
        self.drawer = arboris._visu.Drawer(self.driver, flat, color_generator)
        self.flat = flat

    def init(self, world, timeline):
        Observer.init(self, world, timeline)
        self.driver.init()
        self.world = world
        if self.flat:
            name_all_elements(world.getbodies(), True)
        else:
            nonflatlist = [j.frames[1] for j in world.getjoints()]
            name_all_elements(nonflatlist, True)
        name_all_elements(world.itermovingsubframes(), True)

        world.update_geometric()
        world.parse(self.drawer)
        self.drawer.finish()



    def update(self, dt):
        self.driver.check_keyboard()
        if self. flat is True:
            for b in self.world.getbodies():
                self.driver.update_transform(b.name, b.pose)
            for f in self.world.itermovingsubframes():
                self.driver.update_transform(f.name, f.pose)
        else:
            for j in self.world.getjoints():
                self.driver.update_transform(j.frames[1].name, j.pose)
            for f in self.world.itermovingsubframes():
                self.driver.update_transform(f.name, f.bpose)

    def finish(self):
        pass



