============
Rigid Motion
============

Frames and rigid bodies
=======================

A frame `\Frame` is *an abstract class* which defines the position and
orientation of a particular part of the system.
The frame class has three concrete subclasses, :class:`~arboris.core.Body`, 
:class:`~arboris.core.SubFrame` and :class:`~arboris.core.MovingSubFrame`.


The Body class incorporates the inertia and viscosity properties of some
rigid parts. In a general manner, the inertia matrix is defined as follows

.. math::
    \begin{bmatrix}
          \In[b]        & m \skew{\ve[r]}   \\
        m \skew{r}\tp   & m \Id{}
    \end{bmatrix}

where `\ve[r]` represents the center of mass location relative to `b` the body
frame `\Frame{b}`, `\skew{\bullet}` is skew-symetric matrix and `\bullet\tp` is
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
        \Rot    & \pt \\
        \ve[0]  & 1
    \end{bmatrix}
    \in \R{4\times4}

with `\Rot^{-1}=\Rot \tp \quad \in \R{3\times3}` and `\pt \in \R{3}`.

The *pose* (position and orientation, also known as the *configuration*)
of a (right-handed) coordinate frame `\Frame{b}` regarding to a reference 
(right-handed) coordinate frame `\Frame{a}`: can be described by an 
homogeneous matrix

.. math::
    \HM_{ab} = 
    \begin{bmatrix}
        \Rot\ft{a}{b}   & p\ft{a}{b} \\
        \ve[0]          & 1
    \end{bmatrix}

with:

- `\pt\ft{a}{b}` defined as the `3 \times 1` column vector of coordinates of 
  the origin of `\Frame{b}` expressed in `\Frame{a}`.

- `\Rot_{ab}` defined as the `3 \times 3` matrix with the columns equal to
  the coordinates of the three unit vectors along the frame axes of 
  `\Frame{b}` expressed in `\Frame{a}`.


The inverse pose is computed as follows

 .. math::
    \HM\ft{b}{a} = \HM\ft{a}{b}^{-1} =
    \begin{bmatrix}
        \Rot\ft{b}{a} & \pt\ft{b}{a} \\
        \ve[0] & 1
    \end{bmatrix}
    =
    \begin{bmatrix}
        \Rot\ft{a}{b}\tp    & - \Rot\ft{a}{b}\tp \pt\ft{a}{b} \\
        \ve[0]              & 1
    \end{bmatrix}

Velocity of a coordinate frame
==============================

The velocity of a rigid body can be described by a twist.

.. math::
    \twist[c]\rt{a}{b} = 
    \begin{bmatrix}
        \rotvel[c]\rt{a}{b} \\
        \linvel[c]\rt{a}{b} \\
    \end{bmatrix}

The adjoint matrix `\Ad\ft{a}{b}` which depends on the homogeneous matrix `\HM\ft{a}{b}`
describes the twist displacement from `\Frame{a}` to `\Frame{b}`

.. math::
    \Ad\ft{c}{d} = 
    \begin{bmatrix}
        \Rot\ft{c}{d}                       & 0 \\
        \skew{\pt}\ft{c}{d} \Rot\ft{c}{d}   & \Rot\ft{c}{d}
    \end{bmatrix}
    %
    \hspace{100px}
    \twist[c]\rt{a}{b} = \Ad\ft{c}{d} \cdot \twist[d]\rt{a}{b}


TODO: add adjoint matrix and relative velocities formulas

Wrenches
========

A generalized force acting on a rigid body consist in a linear component
(pure force) `\linforce` and angular component (pure moment) `\rotforce`.
The pair force/moment is named a *wrench* and can be represented using 
a vector in `\R{6}`:

.. math::
    \wrench[c] = 
    \begin{bmatrix}
        \rotforce[c] \\
        \linforce[c] \\
    \end{bmatrix}
    

The displacement of a wrench from a frame to another is done through the use of
the adjoint matrix

 .. math::
    \wrench[c] = \Ad\ft{d}{c}\tp \cdot \wrench[d]

Acceleration of a coordinate frame
==================================

TODO: introduce adjacency

Newton-Euler equations for a rigid body
=======================================

.. math::
    \begin{bmatrix}
        \In[b]    & 0   \\
        0         & m \Id{}
    \end{bmatrix}
    \begin{bmatrix}
        \icf[b]{\dot{\rotvel}}\rt{b}{g}(t) \\
        \icf[b]{\dot{\linvel}}\rt{b}{g}(t)
    \end{bmatrix}
    +
    \begin{bmatrix}
        0 & \rotvel[b]\rt{b}{g}(t) \times \In[b] \\
        0 & \rotvel[b]\rt{b}{g}(t) \times
    \end{bmatrix}
    \begin{bmatrix}
        \rotvel[b]\rt{b}{g}(t) \\
        \linvel[b]\rt{b}{g}(t)
    \end{bmatrix}
    =
    \begin{bmatrix}
        \rotforce[b](t)\\
        \linforce[b](t)\\
    \end{bmatrix}
    
where `\In[b]` is the body inertial tensor, expressed 
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
