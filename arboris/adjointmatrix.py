# coding=utf-8

r""" Functions for working with adjoint matrices.

considering an homogeneous matrix `\H`, the related adjoint matrix `\Ad` is defined as follows

.. math::
    \H = \begin{bmatrix}
            \R  &  \pt  \\
            0   &  1
          \end{bmatrix}
    \hspace{50px}
    \Ad(\H) =  \begin{bmatrix}
                    \R                &  0_{3}    \\
                    \skew{\pt} \R     &  \R
                \end{bmatrix}

"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")


import numpy as np

def isadjointmatrix(Ad):
    """ Return true if *Ad* is an adjoint matrix. """
    return (Ad.shape == (6, 6)) and (
        np.linalg.det(Ad[0:3, 0:3])==1) and (
        Ad[0:3, 0:3]==Ad[3:6, 3:6]).all() and (
        Ad[0:3, 3:6]==np.zeros((3, 3))).all()


def inv(Ad):
    """ Invert an adjoint matrix. """
    R = Ad[0:3, 0:3].transpose()
    pxR = Ad[3:6, 0:3].transpose()
    
    invAd = np.zeros((6, 6))
    invAd[0:3, 0:3] = R
    invAd[3:6, 0:3] = pxR
    invAd[3:6, 3:6] = R
    return invAd

