#coding=utf-8
"""A set of Observers.
"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from numpy import dot

from arboris.core import Observer, MovingSubFrame, name_all_elements
from arboris.massmatrix import principalframe

from pylab import plot, show, legend, xlabel, ylabel, title

try:
    import h5py
except ImportError:
    print """
    ************************************************************
    WARNING: h5py is not installed! HDF5Logger is not available.
    Install h5py first: http://alfven.org/wp/hdf5-for-python/
    ************************************************************
    """

from time import time as _time, sleep
import socket
import subprocess, shlex
import logging
logging.basicConfig(level=logging.DEBUG)

import struct


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
    def __init__(self):
        Observer.__init__(self)
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
        plot(self.time, self.kinetic_energy)
        plot(self.time, self.potential_energy)
        plot(self.time, self.mechanichal_energy)
        legend(('kinetic', 'potential', 'mechanical'))
        title('Energy evolution')
        xlabel('time (s)')
        ylabel('energy (J)')
        show()


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
    def __init__(self, log=False):
        Observer.__init__(self)
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


class Hdf5Logger(Observer):
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
                 save_transforms=True, flat=False, save_model=False):
        Observer.__init__(self)
        # hdf5 file handlers
        self._file = h5py.File(filename, mode)
        self._root = self._file
        for g in group.split('/'):
            if g:
                self._root = self._root.require_group(g)
        # what to save
        self._save_state = save_state
        self._save_transforms = save_transforms
        self._flat = flat
        self._save_model = save_model

        self._world         = None
        self._nb_steps      = -1
        self._current_step  = 0
        self._gpositions    = None
        self._gvelocities   = None
        self._transforms    = None
        self._arb_transforms = {}
        self._model         = None

    @property
    def root(self):
        return self._root

    def init(self, world, timeline):
        """Create the datasets.
        """
        self._world = world
        self._nb_steps = len(timeline)-1
        self._current_step = 0
        self._root.require_dataset("timeline", (self._nb_steps,), 'f8')
        if self._save_state:
            self._gpositions = self._root.require_group('gpositions')
            self._gvelocities = self._root.require_group('gvelocities')
            name_all_elements(self._world.getjoints(), check_unicity=True)
            for j in self._world.getjoints():
                self._gpositions.require_dataset(j.name,
                        (self._nb_steps,) + j.gpos.shape[:], 'f8')
                self._gvelocities.require_dataset(j.name,
                        (self._nb_steps, j.ndof), 'f8')
        if self._save_transforms:
            self._arb_transforms = {}
            self._transforms = self._root.require_group('transforms')
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
                self._transforms.require_dataset(k,
                                                 (self._nb_steps, 4,4),
                                                 'f8')
        if self._save_model:
            self._model = self._root.require_group('model')
            ndof = self._world.ndof
            self._model.require_dataset("gvel",
                    (self._nb_steps, ndof), 'f8')
            self._model.require_dataset("mass",
                    (self._nb_steps, ndof, ndof), 'f8')
            self._model.require_dataset("admittance",
                    (self._nb_steps, ndof, ndof), 'f8')
            self._model.require_dataset("nleffects",
                    (self._nb_steps, ndof, ndof), 'f8')
            self._model.require_dataset("gforce",
                    (self._nb_steps, ndof), 'f8')

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

    def finish(self):
        self._file.close()



class SocketCom(Observer):
    """
    """
    def __init__(self, host="127.0.0.1", port=5000, timeout=3):
        Observer.__init__(self)
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
            self.conn.send("close connection")
            self.s.close()
        except socket.error:
            pass


class DaenimCom(SocketCom):
    """
    """
    def __init__(self, arborisViewer, daefile, host="127.0.0.1", port=5000, \
                 options = "", precision=5, flat=False):
        """
        """
        SocketCom.__init__(self, host, port)
        self.app_call = \
              [arborisViewer, daefile, "-socket", self.host, str(self.port)] + \
               shlex.split(options)
        self.precision = precision
        self.flat = flat
        self.world = None

    def init(self, world, timeline):
        """
        """
        subprocess.Popen(self.app_call)
        SocketCom.init(self, world, timeline)
        self.world = world
        sleep(1.)
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
            msg = ""
            for b in self.world.getbodies():
                H = b.pose
                msg += struct.pack('64s12f', b.name,
                                 *[float(val) for val in H[0:3, :].reshape(12)])
        else:
            msg = ""
            for j in self.world.getjoints():
                H = j.pose
                msg += struct.pack('64s12f', j.frames[1].name,
                                 *[float(val) for val in H[0:3, :].reshape(12)])

        for f in self.world.itermovingsubframes():
            H = f.pose if self.flat else f.bpose
            msg += struct.pack('64s12f', f.name,
                                 *[float(val) for val in H[0:3, :].reshape(12)])

        msg += "update done"
        try:
            self.conn.send(msg)
            self.conn.recv(2)
        except socket.error:
            print("connection lost")


from arboris.visu_vpython import VPythonDriver
import arboris._visu

class VPythonObserver(Observer):
    def __init__(self, scale=1, options=None, flat=False, color_generator=None):
        Observer.__init__(self)
        self.driver = VPythonDriver(scale, options)
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
        self.check_keyboard()
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

    def check_keyboard(self):
        for i in range(self.driver.scene.kb.keys):
            k = self.driver.scene.kb.getkey()
            if   k == "f": objList = self.driver.frame_arrows
            elif k == "s": objList = self.driver.shapes
            elif k == "i": objList = self.driver.inertia
            else: continue
            for obj in objList:
                obj.visible = not obj.visible


