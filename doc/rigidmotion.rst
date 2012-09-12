============
Rigid Motion
============

Frames and rigid bodies
=======================

A frame `\Frame{}` is *an abstract class* which defines the position and
orientation of a particular part of the system.
The frame class has three concrete subclasses, :class:`arboris.core.Body`, 
:class:`arboris.core.SubFrame` and :class:`arboris.core.MovingSubFrame`.


The Body class incorporates the inertia and viscosity properties of some
rigid parts. In a general manner, the inertia matrix is defined as follows

.. math::
    \begin{bmatrix}
        \pre[b]{\In} & m \skew{r}   \\
        m \skew{r}\tp       & m \Id{}
    \end{bmatrix}

where `r` represents the center of mass location relative to `b` the body frame
`\Frame{b}`, `\skew{\bullet}` is skew-symetric matrix and `\bullet\tp` is
transposition.
Bodies can have some subframes to locate some specific locations, as shown in
the image below where `b` has 2 subframes, `\Frame{sf1}` and `\Frame{sf2}`.

.. image:: img/body_alone.svg
   :width: 200 px
   :align: center





Position of a coordinate frame
==============================

An homogeneous matrix `\HM` is a matrix of the form

.. math::
    \HM = 
    \begin{bmatrix}
        \Rot & p \\
        \begin{smallmatrix}
            0 & 0 & 0
        \end{smallmatrix} & 1
    \end{bmatrix}
    \in \R{4\times4}

with `\Rot^{-1}=\Rot \tp \in \R{3\times3}` and `p \in \R{3\times1}`.

The *pose* (position and orientation, also known as the *configuration*)
of a (right-handed) coordinate frame `\Frame{b}` regarding to a reference 
(right-handed) coordinate frame `\Frame{a}`: can be described by an 
homogeneous matrix

.. math::
    \HM_{ab} = 
    \begin{bmatrix}
        \Rot_{ab} & p_{ab} \\
        \begin{smallmatrix}
            0 & 0 & 0
        \end{smallmatrix} & 1
    \end{bmatrix}

with:

- `p_{ab}` defined as the `3 \times 1` column vector of coordinates of 
  the origin of `\Frame{b}` expressed in `\Frame{a}`.

- `\Rot_{ab}` defined as the `3 \times 3` matrix with the columns equal to
  the coordinates of the three unit vectors along the frame axes of 
  `\Frame{b}` expressed in `\Frame{a}`.


The inverse pose is computed as follows

 .. math::
    \HM_{ba} = \HM_{ab}^{-1} =
    \begin{bmatrix}
        \Rot_{ba} & p_{ba} \\
        \begin{smallmatrix}
            0 & 0 & 0
        \end{smallmatrix} & 1
    \end{bmatrix}
    =
    \begin{bmatrix}
        \Rot_{ab}\tp & -\Rot_{ab}\tp p_{ab} \\
        \begin{smallmatrix}
            0 & 0 & 0
        \end{smallmatrix} & 1
    \end{bmatrix}

Velocity of a coordinate frame
==============================

The velocity of a rigid body can be described by a twist.

.. math::
    \twist[c]_{a/b} = 
    \begin{bmatrix}
        \pre[c]\omega_{a/b}\\
        \pre[c]v_{a/b}\\
    \end{bmatrix}

The adjoint matrix `\Ad_{ab}` which depends on the homogeneous matrix `\HM_{ab}`
describes the twist displacement from `\Frame{a}` to `\Frame{b}`

.. math::
    \Ad_{cd} = 
    \begin{bmatrix}
        \Rot_{cd}  & 0 \\
        \skew{p}_{cd} \Rot_{cd} & \Rot_{cd}
    \end{bmatrix}
    %
    \hspace{100px}
    \twist[c]_{a/b} = \Ad_{cd} \cdot \twist[d]_{a/b}


TODO: add adjoint matrix and relative velocities formulas

Wrenches
========

A generalized force acting on a rigid body consist in a linear component
(pure force) `f` and angular component (pure moment) `\tau`. The 
pair force/moment is named a *wrench* and can be represented using 
a vector in `\R{6}`:

.. math::
    \wrench[c] = 
    \begin{bmatrix}
        \pre[c]\tau(t)\\
        \pre[c]f(t)\\
    \end{bmatrix}
    

The displacement of a wrench from a frame to another is done through the use of
the adjoint matrix

 .. math::
    \wrench[c] = \Ad_{dc}\tp \cdot \wrench[d]

Acceleration of a coordinate frame
==================================

TODO: introduce adjacency

Newton-Euler equations for a rigid body
=======================================

.. math::
    \begin{bmatrix}
        \pre[b]{\mathcal{I}} & 0   \\
        0                   & m I
    \end{bmatrix}
    \begin{bmatrix}
        \pre[b]{\dot{\omega}}_{b/g}(t) \\
        \pre[b]{\dot{v}}_{b/g}(t)
    \end{bmatrix}
    +
    \begin{bmatrix}
        0 & \pre[b]\omega_{b/g}(t) \times \pre[b]{\mathcal{I}} \\
        0 & \pre[b]\omega_{b/g}(t) \times
    \end{bmatrix}
    \begin{bmatrix}
        \pre[b]\omega_{b/g}(t) \\
        \pre[b]v_{b/g}(t)
    \end{bmatrix}
    =
    \begin{bmatrix}
        \pre[b]\tau(t)\\
        \pre[b]f(t)\\
    \end{bmatrix}
    
where `\pre[b]{\mathcal{I}}` is the body inertial tensor, expressed 
in the body frame, `b`

Implementation
==============

The modules :mod:`arboris.twistvector`, :mod:`arboris.homogeneousmatrix` and 
:mod:`arboris.adjointmatrix` respectively  implement "low level" operations on 
twist and on homogeneous and adjoint matrices.
For instance, the following excerp creates the homogeneous matrix of a 
translation and then inverts it.

.. doctest::

  >>> import arboris.homogeneousmatrix as homogeneousmatrix
  >>> H = homogeneousmatrix.transl(1., 0., 2./3.)
  >>> H
  array([[ 1.        ,  0.        ,  0.        ,  1.        ],
         [ 0.        ,  1.        ,  0.        ,  0.        ],
         [ 0.        ,  0.        ,  1.        ,  0.66666667],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])
  >>> Hinv = homogeneousmatrix.inv(H)
  >>> Hinv
  array([[ 1.        ,  0.        ,  0.        , -1.        ],
         [ 0.        ,  1.        ,  0.        , -0.        ],
         [ 0.        ,  0.        ,  1.        , -0.66666667],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])

A more convenient way of dealing with rigid motion is planned, by using
a child class of :class:`rigidmotion.RigidMotion`,  which wraps all the 
elementary functions in an object-oriented way. However, this child 
class does not exist yet, one may use :class:`rigidmotion.FreeJoint` 
(see next chapter) instead.


Dynamics
========

TODO: document 1st and 2nd order dynamics for a single rigid body.
