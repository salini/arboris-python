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
          \In[b]        & m \skew{\vec{r}}   \\
        m \skew{r}\tp   & m \Id{}
    \end{bmatrix}

where `\vec{r}` represents the center of mass location relative to `b` the body
frame `\Frame{b}`, `\skew{\bullet}` is skew-symetric matrix and `\bullet\tp` is
transposition.
Bodies can have some subframes to locate some specific locations, as shown in
the image below where `b` has 2 subframes, `\Frame{sf1}` and `\Frame{sf2}`.

.. image:: img/body_alone.svg
   :width: 200 px
   :align: center





Position of a coordinate frame
==============================

An homogeneous matrix `\H` is a matrix of the form

.. math::
    \H = 
    \begin{bmatrix}
        \R       & \pt \\
        \vec{0}  & 1
    \end{bmatrix}
    \in \Real{4\times4}

with `\R^{-1}=\R \tp \quad \in \Real{3\times3}` and `\pt \in \Real{3}`.

The *pose* (position and orientation, also known as the *configuration*)
of a (right-handed) coordinate frame `\Frame{b}` regarding to a reference 
(right-handed) coordinate frame `\Frame{a}`: can be described by an 
homogeneous matrix

.. math::
    \H_{ab} = 
    \begin{bmatrix}
        \R_{ab}  & \pt_{ab} \\
        \vec{0}  & 1
    \end{bmatrix}

with:

- `\pt_{ab}` defined as the `3 \times 1` column vector of coordinates of 
  the origin of `\Frame{b}` expressed in `\Frame{a}`.

- `\R_{ab}` defined as the `3 \times 3` matrix with the columns equal to
  the coordinates of the three unit vectors along the frame axes of 
  `\Frame{b}` expressed in `\Frame{a}`.


The inverse pose is computed as follows

 .. math::
    \H_{ba} = \H_{ab}^{-1} =
    \begin{bmatrix}
        \R_{ba}     & \pt_{ba} \\
        \vec{0}     & 1
    \end{bmatrix}
    =
    \begin{bmatrix}
        \R_{ab}\tp  & - \R_{ab}\tp \pt_{ab} \\
        \vec{0}     & 1
    \end{bmatrix}

Velocity of a coordinate frame
==============================

The velocity of a rigid body can be described by a twist.

.. math::
    \twist[c]_{a/b} = 
    \begin{bmatrix}
        \rotvel[c]_{a/b} \\
        \linvel[c]_{a/b} \\
    \end{bmatrix}

The adjoint matrix `\Ad_{ab}` which depends on the homogeneous matrix `\H_{ab}`
describes the twist displacement from `\Frame{a}` to `\Frame{b}`

.. math::
    \Ad_{cd} = 
    \begin{bmatrix}
        \R_{cd}                   & 0       \\
        \skew{\pt}_{cd} \R_{cd}   & \R_{cd}
    \end{bmatrix}
    %
    \hspace{100px}
    \twist[c]_{a/b} = \Ad_{cd} \cdot \twist[d]_{a/b}


TODO: add adjoint matrix and relative velocities formulas

Wrenches
========

A generalized force acting on a rigid body consist in a linear component
(pure force) `\linforce` and angular component (pure moment) `\rotforce`.
The pair force/moment is named a *wrench* and can be represented using 
a vector in `\Real{6}`:

.. math::
    \wrench[c] = 
    \begin{bmatrix}
        \rotforce[c] \\
        \linforce[c] \\
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
        \In[b]    & 0   \\
        0         & m \Id{}
    \end{bmatrix}
    \begin{bmatrix}
        {\dot{\rotvel[b]}}_{b/g}(t) \\
        {\dot{\linvel[b]}}_{b/g}(t)
    \end{bmatrix}
    +
    \begin{bmatrix}
        0 & \rotvel[b]_{b/g}(t) \times \In[b] \\
        0 & \rotvel[b]_{b/g}(t) \times
    \end{bmatrix}
    \begin{bmatrix}
        \rotvel[b]_{b/g}(t) \\
        \linvel[b]_{b/g}(t)
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
