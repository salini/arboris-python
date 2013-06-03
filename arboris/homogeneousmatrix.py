# coding=utf-8
"""Functions for working with homogeneous matrices."""

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from numpy import array, zeros, sin, cos, dot, arctan2
import numpy

tol = 1e-9

def transl(t_x, t_y, t_z):
    """ Homogeneous matrix of a translation.

    :param float t_x, t_y, t_z: coordinates of the translation vector in 3d space
    :return: homogeneous matrix of the translation
    :rtype: (4,4)-array

    **Example:**

    >>> transl(1., 2., 3.)
    array([[ 1.,  0.,  0.,  1.],
           [ 0.,  1.,  0.,  2.],
           [ 0.,  0.,  1.,  3.],
           [ 0.,  0.,  0.,  1.]])

    """
    return array(
        [[ 1. , 0., 0., t_x],
         [ 0. , 1., 0., t_y],
         [ 0. , 0., 1., t_z],
         [ 0.,  0., 0., 1.]])


def rotzyx(angle_z, angle_y, angle_x):
    """ Homogeneous transformation matrix from roll-pitch-yaw angles.

    :param float angle_z: yaw angle in radian
    :param float angle_y: pitch angle in radian
    :param float angle_x: roll angle in radian
    :return: homogeneous matrix of the roll-pitch-yaw orientation
    :rtype: (4,4)-array
    
    In short, return: \Rot = \Rot_{z} * \Rot_{y} * \Rot_{x}

    **Example:**

    >>> rotzyx(3.14/6, 3.14/4, 3.14/3)
    array([[ 0.61271008,  0.27992274,  0.73907349,  0.        ],
           [ 0.35353151,  0.73930695, -0.57309746,  0.        ],
           [-0.70682518,  0.61242835,  0.35401931,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """

    sz = sin(angle_z)
    cz = cos(angle_z)
    sy = sin(angle_y)
    cy = cos(angle_y)
    sx = sin(angle_x)
    cx = cos(angle_x)
    return array(
        [[ cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx, 0.],
         [ sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx, 0.],
         [-sy   , cy*sx         , cy*cx         , 0.],
         [ 0.   , 0.            , 0.            , 1.]])


def rotzy(angle_z, angle_y):
    """ Homogeneous transformation matrix from pitch-yaw angles.

    :param float angle_z: yaw angle in radian
    :param float angle_y: pitch angle in radian
    :return: homogeneous matrix of the pitch-yaw orientation
    :rtype: (4,4)-array
    
    In short, return: \Rot = \Rot_{z} * \Rot_{y}

    Example:

    >>> rotzy(3.14/6, 3.14/4)
    array([[ 0.61271008, -0.4997701 ,  0.61222235,  0.        ],
           [ 0.35353151,  0.86615809,  0.35325009,  0.        ],
           [-0.70682518,  0.        ,  0.70738827,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    sz = sin(angle_z)
    cz = cos(angle_z)
    sy = sin(angle_y)
    cy = cos(angle_y)
    return array(
        [[ cz*cy,-sz, cz*sy, 0.],
         [ sz*cy, cz, sz*sy, 0.],
         [-sy   , 0., cy   , 0.],
         [ 0.   , 0., 0.   , 1.]])


def rotzx(angle_z, angle_x):
    """ Homogeneous transformation matrix from roll-yaw angles.

    :param float angle_z: yaw angle in radian
    :param float angle_x: roll angle in radian
    :return: homogeneous matrix of the roll-yaw orientation
    :rtype: (4,4)-array
    
    In short, return: \Rot = \Rot_{z} * \Rot_{x}

    **Example:**

    >>> rotzx(3.14/6, 3.14/3)
    array([[ 0.86615809, -0.25011479,  0.43268088,  0.        ],
           [ 0.4997701 ,  0.43347721, -0.74988489,  0.        ],
           [ 0.        ,  0.86575984,  0.50045969,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    sz = sin(angle_z)
    cz = cos(angle_z)
    sx = sin(angle_x)
    cx = cos(angle_x)
    return array(
        [[ cz,-sz*cx, sz*sx, 0.],
         [ sz, cz*cx,-cz*sx, 0.],
         [ 0., sx   , cx   , 0.],
         [ 0., 0.   , 0.   , 1.]])


def rotyx(angle_y, angle_x):
    """ Homogeneous transformation matrix from roll-pitch angles.

    :param float angle_y: pitch angle in radian
    :param float angle_x: roll angle in radian
    :return: homogeneous matrix of the roll-pitch orientation
    :rtype: (4,4)-array
    
    In short, return: \Rot = \Rot_{y} * \Rot_{x}

    **Example:**

    >>> rotyx(3.14/4, 3.14/3)
    array([[ 0.70738827,  0.61194086,  0.35373751,  0.        ],
           [ 0.        ,  0.50045969, -0.86575984,  0.        ],
           [-0.70682518,  0.61242835,  0.35401931,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    sy = sin(angle_y)
    cy = cos(angle_y)
    sx = sin(angle_x)
    cx = cos(angle_x)
    return array(
        [[ cy, sy*sx, sy*cx, 0.],
         [ 0., cx   ,-sx   , 0.],
         [-sy, cy*sx, cy*cx, 0.],
         [ 0., 0.   , 0.   , 1.]])

def rotx(angle):
    """ Homogeneous matrix of a rotation around the x-axis.

    :param float angle: angle around x-axis in radian
    :return: homogeneous matrix
    :rtype: (4,4)-array

    **Example:**

    >>> rotx(3.14/6)
    array([[ 1.        ,  0.        ,  0.        ,  0.        ],
           [ 0.        ,  0.86615809, -0.4997701 ,  0.        ],
           [ 0.        ,  0.4997701 ,  0.86615809,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])
    """
    ca = cos(angle)
    sa = sin(angle)
    H = array(
        [[1,  0,   0,  0],
         [0, ca, -sa,  0],
         [0, sa,  ca,  0],
         [0,  0,   0,  1]])
    return H

def roty(angle):
    """ Homogeneous matrix of a rotation around the y-axis.

    :param float angle: angle around y-axis in radian
    :return: homogeneous matrix
    :rtype: (4,4)-array

    **Example:**

    >>> roty(3.14/6)
    array([[ 0.86615809,  0.        ,  0.4997701 ,  0.        ],
           [ 0.        ,  1.        ,  0.        ,  0.        ],
           [-0.4997701 ,  0.        ,  0.86615809,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])
    """
    ca = cos(angle)
    sa = sin(angle)
    H = array(
        [[ ca,  0,  sa,  0],
         [  0,  1,   0,  0],
         [-sa,  0,  ca,  0],
         [  0,  0,   0,  1]])
    return H

def rotz(angle):
    """ Homogeneous matrix of a rotation around the z-axis.

    :param float angle: angle around z-axis in radian
    :return: homogeneous matrix
    :rtype: (4,4)-array

    **Example:**

    >>> rotz(3.14/6)
    array([[ 0.86615809, -0.4997701 ,  0.        ,  0.        ],
           [ 0.4997701 ,  0.86615809,  0.        ,  0.        ],
           [ 0.        ,  0.        ,  1.        ,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])
    """
    ca = cos(angle)
    sa = sin(angle)
    H = array(
        [[ca, -sa, 0, 0],
         [sa,  ca, 0, 0],
         [ 0,   0, 1, 0],
         [ 0,   0, 0, 1]])
    return H

def zaligned(vec):
    """ Returns an homogeneous matrix whose z-axis is colinear vith *vec*.

    :param vec: input vector, assumed to be normalized
    :type  vec: (3,)-array
    :return: homogeneous matrix of the frame aligned with vec
    :rtype: (4,4)-array

    **Example:**

    >>> zaligned((1.,0.,0.))
    array([[-0.,  0.,  1.,  0.],
           [ 0., -1.,  0.,  0.],
           [ 1.,  0.,  0.,  0.],
           [ 0.,  0.,  0.,  1.]])

    """
    assert numpy.abs(numpy.linalg.norm(vec)-1) < 1e-9
    H = numpy.eye(4)
    x = H[0:3, 0]
    y = H[0:3, 1]
    z = H[0:3, 2]
    # z-axis, normal to the tangent plane:
    z[:] = vec
    idx = numpy.argsort(numpy.absolute(z))
    # x axis, normal to z-axis
    x[idx[0]] = 0
    x[idx[1]] = z[idx[2]]
    x[idx[2]] = -z[idx[1]]
    x /= numpy.linalg.norm(x)
    y[:] = numpy.cross(z, x)
    return H

def ishomogeneousmatrix(H, _tol=tol):
    """ Return true if input is an homogeneous matrix.

    :param H: the homogeneous matrix to check
    :type  H: (4,4)-array
    :param float _tol: the tolerance for the rotation matrix determinant
    :return: True if homogeneous matrix, False otherwise

    """
    return (H.shape == (4, 4)) \
        and (numpy.abs(numpy.linalg.det(H[0:3, 0:3])-1) <= _tol) \
        and (H[3, 0:4]==[0, 0, 0, 1]).all()

def pdot(H, point):
    r""" Frame displacement for a point.

    :param H: the homogeneous matrix to check
    :type  H: (4,4)-array
    :param point: point one wants to displace
    :type  point: (3,)-array
    :return: the displaced point
    :rtype: (3,)-array

    `\icf[a]{\pt} = \HM\ft{a}{b}  \icf[b]{\pt}`, where `\HM\ft{a}{b}` is the
    homogeneous matrix from `\Frame{a}` to `\Frame{b}`, and `\icf[b]{\pt}` is
    the point expressed in `\Frame{b}`.

    """
    assert ishomogeneousmatrix(H)
    return dot(H[0:3, 0:3], point) + H[0:3, 3]

def vdot(H, vec):
    r""" Frame displacement for a vector.

    :param H: the homogeneous matrix to check
    :type  H: (4,4)-array
    :param vec: point one wants to displace
    :type  vec: (3,)-array
    :return: the displaced point
    :rtype: (3,)-array

    `\icf[a]{\ve} = \Rot\ft{a}{b}  \icf[b]{\ve}`, where `\Rot\ft{a}{b}` is the
    rotation matrix from `\Frame{a}` to `\Frame{b}`, and `\icf[b]{\ve}` is
    the vector expressed in `\Frame{b}`.

    """
    assert ishomogeneousmatrix(H)
    return dot(H[0:3, 0:3], vec)

def inv(H):
    """ Invert a homogeneous matrix.

    :param H: the homogeneous matrix to invert
    :type  H: (4,4)-array
    :return: inverted homogeneous matrix
    :rtype: (4,4)-array

    **Example:**

    >>> H = array(
    ...     [[ 0.70738827,  0.        , -0.70682518,  3.        ],
    ...      [ 0.61194086,  0.50045969,  0.61242835,  4.        ],
    ...      [ 0.35373751, -0.86575984,  0.35401931,  5.        ],
    ...      [ 0.        ,  0.        ,  0.        ,  1.        ]])
    >>> inv(H)
    array([[ 0.70738827,  0.61194086,  0.35373751, -6.3386158 ],
           [ 0.        ,  0.50045969, -0.86575984,  2.32696044],
           [-0.70682518,  0.61242835,  0.35401931, -2.09933441],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])
    """
    assert ishomogeneousmatrix(H)
    R = H[0:3, 0:3]
    p = H[0:3, 3]
    
    invH = zeros((4,4))
    invH[0:3, 0:3] = R.T
    invH[0:3,3]    = -dot(R.T, p)
    invH[3,3]      = 1
    return invH

def adjoint(H):
    """ Adjoint of the homogeneous matrix.

    :param H: homogeneous matrix
    :type  H: (4,4)-array
    :return: adjoint matrix
    :rtype: (6,6)-array

    **Example:**

    >>> H = array(
    ...     [[ 0.70738827,  0.        , -0.70682518,  3.        ],
    ...      [ 0.61194086,  0.50045969,  0.61242835,  4.        ],
    ...      [ 0.35373751, -0.86575984,  0.35401931,  5.        ],
    ...      [ 0.        ,  0.        ,  0.        ,  1.        ]])
    >>> adjoint(H)
    array([[ 0.70738827,  0.        , -0.70682518,  0.        ,  0.        ,
             0.        ],
           [ 0.61194086,  0.50045969,  0.61242835,  0.        ,  0.        ,
             0.        ],
           [ 0.35373751, -0.86575984,  0.35401931,  0.        ,  0.        ,
             0.        ],
           [-1.64475426, -5.96533781, -1.64606451,  0.70738827,  0.        ,
            -0.70682518],
           [ 2.47572882,  2.59727952, -4.59618383,  0.61194086,  0.50045969,
             0.61242835],
           [-0.9937305 ,  1.50137907,  4.66458577,  0.35373751, -0.86575984,
             0.35401931]])
    """
    assert ishomogeneousmatrix(H)
    R = H[0:3, 0:3]
    p = H[0:3, 3]
    pxR = dot(
        array(
            [[    0, -p[2],  p[1]],
             [ p[2],     0, -p[0]],
             [-p[1],  p[0],     0]]),
        R)

    Ad = zeros((6,6))
    Ad[0:3,0:3] = R
    Ad[3:6,0:3] = pxR
    Ad[3:6,3:6] = R
    return Ad


def iadjoint(H):
    """ Return the adjoint ((6,6) array) of the inverse homogeneous matrix.
    """
    return adjoint(inv(H))

def dAdjoint(Ad, T):
    """ Return the derivative of an Adjoint with respect to time.

    definition from arboris-matlab
    if H is defined as follow:
    x{a} = H * x{b}
    Ad = adjoint( H )
    T  = velocity of {b} relative to {a} expressed in {b}

    :param Ad: adjoint matrix
    :type Ad: (6,6) array
    :param T: twist vectors
    :type T: (6,) array

    :return: ((6,6) array) dAdjoint matrix
    """
    MT = array([[   0., -T[2],  T[1],    0.,    0.,    0.],
                [ T[2],    0., -T[0],    0.,    0.,    0.],
                [-T[1],  T[0],    0.,    0.,    0.,    0.],
                [   0., -T[5],  T[4],    0., -T[2],  T[1]],
                [ T[5],    0., -T[3],  T[2],    0., -T[0]],
                [-T[4],  T[3],    0., -T[1],  T[0],    0.]])
    return dot(Ad, MT)

def rotzyx_angles(H):
    """ Returns the roll-pitch-yaw angles corresponding to the rotation matrix of `\HM`.

    :param H: homogeneous matrix
    :type  H: (4,4)-array
    :return: angles of roll pitch yaw
    :rtype: (3,)-array

    Returns the angles such that `H[0:3, 0:3] = R_z(a_z) R_y(a_y) R_x(a_x)`.

    **Example:**

    >>> angles = array((3.14/3, 3.14/6, 1))
    >>> (rotzyx_angles(rotzyx(*angles)) == angles).all()
    True

    """
    assert ishomogeneousmatrix(H)
    if abs(H[0, 0]) < tol and abs(H[1, 0]) < tol:
        # singularity
        az = 0
        ay = arctan2(-H[2, 0], H[0, 0])
        ax = arctan2(-H[1, 2], H[1, 1])
    else:
        az = arctan2( H[1, 0], H[0, 0])
        sz = sin(az)
        cz = cos(az)
        ay = arctan2(-H[2, 0], cz*H[0, 0] + sz*H[1, 0])
        ax = arctan2(sz*H[0, 2] - cz*H[1, 2], cz*H[1, 1] - sz*H[0, 1])
    return (az, ay, ax)


