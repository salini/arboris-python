# coding=utf-8
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from numpy        import array, zeros, dot, ix_
from numpy.linalg import norm
import arboris.homogeneousmatrix as Hg
from   arboris.core   import Controller, World
from   arboris.joints import LinearConfigurationSpaceJoint

class WeightController(Controller):
    """ A contoller which applies weight to joints.

    **Test:**

    >>> from arboris.core import simplearm
    >>> w = simplearm()
    >>> joints = w.getjoints()
    >>> joints['Shoulder'].gpos[0] = 3.14/4
    >>> joints['Elbow'].gpos[0]    = 3.14/4
    >>> joints['Wrist'].gpos[0]    = 3.14/4
    >>> c = WeightController()
    >>> w.register(c)
    >>> w.init()
    >>> w.update_dynamic()
    >>> (gforce, impedance) = c.update()

    """
    def __init__(self, gravity=-9.81, name=None):
        """ Simulate gravity along the world up axis.

        :param float gravity: the gravity magnitude along world up axis (can be negative)
        :param name: the name of the controller
        :type  name: string or None

        """
        Controller.__init__(self, name=name)
        self.gravity         = float(gravity)
        self._bodies         = None
        self._wndof          = None
        self._gravity_dtwist = None
        self._impedance      = None


    def init(self, world):
        """ Initialize controller and configure for one :class:`~arboris.core.World` instance.

        This method is generaly called in :meth:`~arboris.core.simulate`, at
        the beginning, just before the simulation loop.

        """
        assert isinstance(world, World)
        self._bodies = [x for x in world.ground.iter_descendant_bodies() \
                        if norm(x.mass>0.)]
        self._wndof               = world.ndof
        self._gravity_dtwist      = zeros(6)
        self._gravity_dtwist[3:6] = float(self.gravity)*world.up
        self._impedance           = zeros( (self._wndof, self._wndof) )

    def update(self, dt=None):
        """ Compute gravity vector for current state. """
        gforce = zeros(self._wndof)
        for b in self._bodies:
            g       = dot(Hg.iadjoint(b.pose), self._gravity_dtwist)
            gforce += dot(b.jacobian.T, dot(b.mass, g))

        return (gforce, self._impedance)


class ProportionalDerivativeController(Controller):
    r""" A proportional-derivative controller.

    A `n`-dimensional PD controller which delivers a torque `\torque`:

    .. math::
        \torque(t) &= K_p (\q_d - \q(t+\dt)) + K_d (\dq_d - \dq(t+\dt))

    where `\q_d` and `\dq_d` are the respectively the desired position
    and velocity and where `\torque(t)` is assumed constant on the `[t,t+dt]`
    interval.

    Here

    .. math::
        \q(t+\dt) = \q(t) + \dq(t+\dt) \dt

    so

    .. math::
        \torque(t) &= K_p (\q_d  - (\q(t) + \dq(t+\dt) \dt ) )
                    + K_d (\dq_d - \dq(t+\dt)) \\
                   &= K_p (\q_d - \q(t))
                    + K_d \dq_d - (K_p \dt + K_d)\dq(t+\dt) \\
                   &= \torque_0(t) + Z(t) \dq(t+\dt)

    with

    .. math::
        \torque_0(t) &= K_p (\q_d - q(t)) + K_d \dq_d \\
                Z(t) &= -(K_p \dt + K_d)

    `\torque_0(t)` can be denoted as the **reactive part** of the control,
    while `Z(t) \dq(t+\dt)` can be considered as the **proactive part** which
    depends on the future generalized velocity `\dq(t+\dt)` (after integration).
    
    `Z` is denoted the impedance of the controller.

    """
    def __init__(self, joints, kp=None, kd=None, gpos_des=None, gvel_des=None, name=None):
        """ Track an error with a proportional-derivative controller.

        :param iterable joints: a list of :class:`~arboris.core.LinearConfigurationSpaceJoint` one wants to control (**x** is the total dimension of the controlled joints)
        :param kp: the proportional gains
        :type  kp: (x,x)-array
        :param kd: the derivative gains
        :type  kd: (x,x)-array
        :param gpos_des: desired generalized position
        :type  gpos_des: (x,)-array
        :param gvel_des: desired generalized velocity
        :type  gvel_des: (x,)-array
        :param name: the name of the controller

        """
        Controller.__init__(self, name=name)
        self._cndof = 0
        dof_map = []
        for j in joints:
            if not isinstance(j, LinearConfigurationSpaceJoint):
                raise ValueError('Joints must be ' +\
                        'LinearConfigurationSpaceJoint instances')
            else:
                self._cndof += j.ndof
                dof_map.extend(list(range(j.dof.start, j.dof.stop)))
        self._dof_map = array(dof_map)
        self.joints = joints
        self._wndof = None

        if kp is None:
            self.kp = zeros((self._cndof, self._cndof))
        else :
            self.kp = array(kp).reshape((self._cndof, self._cndof))

        if kd is None:
            self.kd = zeros((self._cndof, self._cndof))
        else :
            self.kd = array(kd).reshape((self._cndof, self._cndof))

        if gpos_des is None:
            self.gpos_des = zeros(self._cndof)
        else:
            self.gpos_des = array(gpos_des).reshape(self._cndof)

        if gvel_des is None:
            self.gvel_des = zeros(self._cndof)
        else:
            self.gvel_des = array(gvel_des).reshape(self._cndof)

    def init(self, world):
        """ Initialize controller and configure for one :class:`~arboris.core.World` instance.

        This method is generaly called in :meth:`~arboris.core.simulate`, at
        the beginning, just before the simulation loop.

        """
        self._wndof = world.ndof
        dof_map = []
        for j in self.joints:
            dof_map.extend(list(range(j.dof.start, j.dof.stop)))
        self._dof_map = array(dof_map)

    def update(self, dt):
        """ Compute the generalized force that generate a PD-control. """
        gforce = zeros(self._wndof)
        impedance = zeros((self._wndof, self._wndof))

        gpos = []
        gvel = []
        for j in self.joints:
            gpos.append(j.gpos)
            gvel.append(j.gvel)
        gpos = array(gpos).reshape(self._cndof)
        gvel = array(gvel).reshape(self._cndof)
        gforce[self._dof_map] = \
                dot(self.kp, self.gpos_des - gpos) + \
                dot(self.kd, self.gvel_des)
        impedance[ix_(self._dof_map, self._dof_map)] = -(dt*self.kp+self.kd)
        return (gforce, impedance)


