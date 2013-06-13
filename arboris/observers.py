#coding=utf-8
"""A set of Observers.
"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from numpy import dot, zeros, arange

from arboris.core import Observer, MovingSubFrame, name_all_elements
from arboris.massmatrix import principalframe
from arboris.collisions import choose_solver
from arboris.homogeneousmatrix import iadjoint, dAdjoint


from pylab import plot, show, legend, xlabel, ylabel, title


import pickle as pkl
import h5py


from   time import time as _time
import socket
import logging
logging.basicConfig(level=logging.DEBUG)


class EnergyMonitor(Observer):
    """ Compute and store the world energy at each time step.

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
        """
        :param string name: the instance name of the observer

        """
        Observer.__init__(self, name)
        self._world             = None
        self.time               = []
        self.kinetic_energy     = []
        self.potential_energy   = []
        self.mechanichal_energy = []
        self._com_pos           = {}
        self._bodies            = None

    def init(self, world, timeline):
        self._world             = world
        self.time               = []
        self.kinetic_energy     = []
        self.potential_energy   = []
        self.mechanichal_energy = []
        self._com_pos           = {}
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
        """ Plot the energy evolution. """

        plot(self.time, self.kinetic_energy)
        plot(self.time, self.potential_energy)
        plot(self.time, self.mechanichal_energy)
        legend(('kinetic', 'potential', 'mechanical'))
        title('Energy evolution')
        xlabel('time (s)')
        ylabel('energy (J)')
        show()


class PerfMonitor(Observer):
    """ Get current world time during simulation, and overall simulation performances.

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
        """
        :param bool log: whether to show current time of simulation
        :param string name: the instance name of the observer
        
        """
        Observer.__init__(self, name)
        if log:
            self._logger = logging.getLogger(self.__class__.__name__)
        else:
            self._logger = None
        self._world            = None
        self._last_time        = None
        self._computation_time = []

    def init(self, world, timeline):
        self._world     = world
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



class SaveLogger(Observer):
    """ An abstract observer that saves the simulation data.

    All the data are in S.I. units (radians, meters, newtons and seconds).

    The possible recorded data for each time step are::

        timeline (nsteps,)
        gpositions/
        gvelocities/
        model/
        transforms/

    The ``gvelocities`` contains the generalized velocities of the
    joints::

        NameOfJoint0 (nsteps, joint0.ndof)
        NameOfJoint1 (nsteps, joint1.ndof)
        ...

    while the ``gpositions`` contains their generalized positions.

    The ``model`` contains the matrices from the dynamical model::

        gvel (nsteps, ndof)
        gforce (nsteps, ndof)
        mass (nsteps, ndof, ndof)
        nleffects (nsteps, ndof, ndof)
        admittance (nsteps, ndof, ndof)

    The ``transforms`` contains homogeneous transformations,
    useful for viewing an animation of the simulation::

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

    def __init__(self, save_transforms=True, save_state=False, save_model=False,
                       flat=False,  name=None):
        """
        :param bool save_transforms: toggle the write of the ``transforms`` group
        :param bool save_state: toggle the write of the ``gpos`` and ``gvel`` groups
        :param bool save_model: toggle the write of ``model`` group
        :param bool flat: whether to save body of joint poses in the ``transforms`` group
        :param string name: the instance name of the observer

        """
        Observer.__init__(self, name)
        # what to save
        self._save_transforms = save_transforms
        self._save_state      = save_state
        self._save_model      = save_model
        self._flat            = flat
        
        # recorded values
        self._world          = None
        self._nb_steps       = 0
        self._current_step   = 0
        self._root           = {}
        self._timeline       = []
        self._arb_transforms = {}
        self._transforms     = {}
        self._gpositions     = {}
        self._gvelocities    = {}
        self._model          = {}

    def init(self, world, timeline):
        self._world        = world
        self._nb_steps     = len(timeline)-1
        self._current_step = 0
        self._root         = {}
        self._timeline     = zeros(self._nb_steps)
        self._root["timeline"] = self._timeline

        if self._save_state:
            self._gpositions          = {}
            self._gvelocities         = {}
            self._root['gpositions']  = self._gpositions
            self._root['gvelocities'] = self._gvelocities
            name_all_elements(self._world.getjoints(), check_unicity=True)
            for j in self._world.getjoints():
                self._gpositions[j.name]  = zeros((self._nb_steps,)+j.gpos.shape[:])
                self._gvelocities[j.name] = zeros((self._nb_steps, j.ndof))

        if self._save_transforms:
            self._arb_transforms     = {}
            self._transforms         = {}
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
            self._model               = {}
            self._root['model']       = self._model
            self._model["gvel"]       = zeros((self._nb_steps, ndof))
            self._model["mass"]       = zeros((self._nb_steps, ndof, ndof))
            self._model["admittance"] = zeros((self._nb_steps, ndof, ndof))
            self._model["nleffects"]  = zeros((self._nb_steps, ndof, ndof))
            self._model["gforce"]     = zeros((self._nb_steps, ndof))

    def update(self, dt):
        """ Save the current data (state...). """

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
            self._model["gvel"][self._current_step, :]          = self._world.gvel
            self._model["mass"][self._current_step, :, :]       = self._world.mass
            self._model["admittance"][self._current_step, :, :] = self._world.admittance
            self._model["nleffects"][self._current_step, :, :]  = self._world.nleffects
            self._model["gforce"][self._current_step, :]        = self._world.gforce
        self._current_step += 1


class PickleLogger(SaveLogger):
    """ A concrete class to save simulation data in pickle file.

    See :class:`~arboris.observers.SaveLogger` to have more info on saved data.

    Here, ``gpositions, gvelocities, model, transforms`` are saved as dictionnaries
    in pickle file.

    """
    def __init__(self, filename, mode='wb', save_transforms=True, save_state=False,
                  save_model=False, flat=False, protocol=0, name=None):
        """
        :param string filename: the name of the pickle file where to write the data
        :param string mode: mode to open the pickle file
        :param bool save_transforms: toggle the write of the ``transforms`` group
        :param bool save_state: toggle the write of the ``gpos`` and ``gvel`` groups
        :param bool save_model: toggle the write of ``model`` group
        :param bool flat: whether to save body of joint poses in the ``transforms`` group
        :param string name: the instance name of the observer

        """
        SaveLogger.__init__(self, save_transforms, save_state, save_model, flat, name)
        self._filename = filename
        self._mode = mode
        self._protocol = protocol

    def finish(self):
        with open(self._filename, self._mode) as _file:
            pkl.dump(self._root, _file, self._protocol)


class Hdf5Logger(SaveLogger):
    """ A concrete class to save simulation data in hdf5 file.

    See :class:`~arboris.observers.SaveLogger` to have more info on saved data.

    All the simulation data lies in a single user-chosen group, that is denoted
    ``root`` in the following, and which defaults to ``/``.

    The hdf5 file has the following layout::

      root/timeline (nsteps,)
      root/gpositions/
      root/gvelocities/
      root/model/
      root/transforms/

    Here, ``gpositions, gvelocities, model, transforms`` represent hdf5 groups.

    """
    def __init__(self, filename, group="/", mode='a', save_transforms=True,
                   save_state=False, save_model=False, flat=False, name=None):
        """
        :param string filename: the name of the hdf5 file where to write the data
        :param string group: the group name inside the hdf5 file where to write data
        :param string mode: mode to open the hdf5 file
        :param bool save_transforms: toggle the write of the ``transforms`` group
        :param bool save_state: toggle the write of the ``gpos`` and ``gvel`` groups
        :param bool save_model: toggle the write of ``model`` group
        :param bool flat: whether to save body of joint poses in the ``transforms`` group
        :param string name: the instance name of the observer

        """
        SaveLogger.__init__(self, save_transforms, save_state, save_model, flat, name)
        self._file = h5py.File(filename, mode)  # hdf5 file handlers
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
    """ An abstract observer that can communicate with another application through sockets.

    Children class must defined the ``update`` method.

    """

    def __init__(self, host="127.0.0.1", port=5000, timeout=3, name=None):
        """
        :param string host: the remote host address
        :param int port: the remote connection port
        :param float timeout: time before raising error if not connection found
        :param string name: the instance name of the observer

        """
        Observer.__init__(self, name)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
            self.conn.send("close_connection")
            self.s.close()
        except socket.error:
            pass



class CoMObserver(Observer):
    """ Get Center of Mass (CoM) position and jacobian of a group of bodies.

    This observer can also be used in another observer to record CoM data,
    or in a controller to control the CoM motion.

    """
    def __init__(self, bodies, compute_Jacobians=True, name=None):
        """
        :param bodies: the bodies on which the CoM is computed
        :type  bodies: list of :class:`~arboris.core.Body`
        :param bool compute_Jacobians: whether CoM jacobian and its derivative has to be computed
        :param string name: the instance name of the observer

        """
        Observer.__init__(self, name)
        self.user_bodies = bodies
        self.compute_Jacobians = compute_Jacobians

        self.H_body_com    = []
        self.mass          = []
        self.bodies        = []
        self.total_mass    = 0.
        self._CoMPosition  = zeros(3)
        self._CoMJacobian  = zeros((3, 0))
        self._CoMdJacobian = zeros((3, 0))

    def init(self, world, timeline=None):
        self.H_body_com = []
        self.mass       = []
        self.bodies     = [b for b in self.user_bodies if b.mass[5,5]>0]
        for b in self.bodies:
            self.H_body_com.append(principalframe(b.mass))
            self.mass.append(b.mass[5,5])

        self.total_mass    = sum(self.mass)
        self._CoMPosition  = zeros(3)
        self._CoMJacobian  = zeros((3, world.ndof))
        self._CoMdJacobian = zeros((3, world.ndof))

    def update(self, dt):
        self._CoMPosition[:]  = 0.
        self._CoMJacobian[:]  = 0.
        self._CoMdJacobian[:] = 0.
        
        for i in arange(len(self.bodies)):
            H_0_com_i = dot(self.bodies[i].pose, self.H_body_com[i])
            self._CoMPosition += self.mass[i] * H_0_com_i[0:3,3]
            
            if self.compute_Jacobians is True:
                ##### For jacobian
                H_com_i_com2 = zeros((4,4))       #com2 is aligned with ground
                H_com_i_com2[0:3, 0:3] = H_0_com_i[0:3,0:3].T
                H_com_i_com2[3,3] = 1.
                
                Ad_com2_body = iadjoint(dot(self.H_body_com[i], H_com_i_com2))
                self._CoMJacobian[:] += self.mass[i] * dot(Ad_com2_body, self.bodies[i].jacobian)[3:6,:]

                ##### For dJacobian
                T_com2_body = self.bodies[i].twist.copy()
                T_com2_body[3:6] = 0.
                dAd_com2_body = dAdjoint(Ad_com2_body, T_com2_body)
                dJ_com2 = dot(Ad_com2_body, self.bodies[i].djacobian) + dot(dAd_com2_body, self.bodies[i].jacobian)
                self._CoMdJacobian[:] += self.mass[i] * dJ_com2[3:6,:]

        self._CoMPosition  /= self.total_mass
        self._CoMJacobian  /= self.total_mass
        self._CoMdJacobian /= self.total_mass

    def finish(self):
        pass

    def get_CoMPosition(self):
        return self._CoMPosition.copy()

    def get_CoMJacobian(self):
        return self._CoMJacobian.copy()

    def get_CoMdJacobian(self):
        return self._CoMdJacobian.copy()


class _Recorder(Observer):
    """ An abstract observer to record any world information in a list.

    In children class, when an data has to be saved in the ``update`` method,
    one should use the method ``save_data``.
    
    When simulation is finished, recorded data can be obtain with method ``get_record``.

    """

    def __init__(self, name=None):
        """
        :param string name: the instance name of the observer

        """
        Observer.__init__(self, name)
        self._record = []
        self._index = 0

    def init(self, world, timeline):
        self._record = [None]*(len(timeline)-1)
        self._index = 0

    def update(self, dt):
        pass

    def save_data(self, data):
        """ Save ``data`` in inner list, and increment index. """
        self._record[self._index] = data
        self._index += 1

    def finish(self):
        pass

    def get_record(self):
        """ Return inner list of recorded data. """
        return self._record



class RecordJointGpos(_Recorder):
    def __init__(self, joint, name=None):
        """ Record the generalized pose of a joint.
        
        :param joint: the joint to track
        :type  joint: :class:`~arboris.core.Joint`
        :param string name: the instance name of the observer

        """
        _Recorder.__init__(self, name)
        self.joint = joint

    def update(self, dt):
        self.save_data(self.joint.gpos.copy())

class RecordJointGvel(_Recorder):
    def __init__(self, joint, name=None):
        """ Record the generalized velocity of a joint.
        
        :param joint: the joint to track
        :type  joint: :class:`~arboris.core.Joint`
        :param string name: the instance name of the observer

        """
        _Recorder.__init__(self, name)
        self.joint = joint

    def update(self, dt):
        self.save_data(self.joint.gvel.copy())


class RecordFramePose(_Recorder):
    def __init__(self, frame, name=None):
        """ Record the pose of a frame (`f`) relative to the ground (`g`) `\H_{gf}`.
        
        :param joint: the frame to track
        :type  joint: :class:`~arboris.core.Frame`
        :param string name: the instance name of the observer

        """
        _Recorder.__init__(self, name)
        self.frame = frame

    def update(self, dt):
        self.save_data(self.frame.pose)


class RecordFrameTwist(_Recorder):
    def __init__(self, frame, name=None):
        r""" Record the twist of a frame (`f`) relative to the ground (`g`) expressed in frame `\twist{f}_{f/g}`.
        
        :param joint: the frame to track
        :type  joint: :class:`~arboris.core.Frame`
        :param string name: the instance name of the observer

        """
        _Recorder.__init__(self, name)
        self.frame = frame

    def update(self, dt):
        self.save_data(self.frame.twist)


class RecordConstForce(_Recorder):
    def __init__(self, const, name=None):
        """ Record the generalized force of a constraint.
        
        :param joint: the constraint to track
        :type  joint: :class:`~arboris.core.Constraint`
        :param string name: the instance name of the observer

        """
        _Recorder.__init__(self, name)
        self.const = const

    def update(self, dt):
        self.save_data(self.const._force.copy())


class RecordDistance(_Recorder):
    def __init__(self, sh1, sh2, name=None):
        """ Record the distance between two shapes.

        :param sh1: the first shape to compute the distance
        :type  sh1: :class:`~arboris.core.Shape`
        :param sh2: the first shape to compute the distance
        :type  sh2: :class:`~arboris.core.Shape`
        :param string name: the instance name of the observer

        **Warning:**

        This recorder works only if a distance solver for these two shapes
        is returned by :meth:`~arboris.collisions.choose_solver`.

        """
        _Recorder.__init__(self, name)
        self.shapes, self.solver = choose_solver(sh1, sh2)

    def update(self, dt):
        dist = self.solver(self.shapes)[0]
        self.save_data(dist)


class RecordCoMPosition(_Recorder):
    def __init__(self, bodies, name=None):
        """ Record the CoM position of a group of bodies.
        
        :param bodies: the bodies on which the CoM is computed
        :type  bodies: list of :class:`~arboris.core.Body`
        :param string name: the instance name of the observer

        """
        _Recorder.__init__(self, name)
        self.com_obs = CoMObserver(bodies, compute_Jacobians=False)

    def init(self, world, timeline):
        _Recorder.init(self, world, timeline)
        self.com_obs.init(world, timeline)

    def update(self, dt):
        self.com_obs.update(dt)
        self.save_data(self.com_obs.get_CoMPosition())


