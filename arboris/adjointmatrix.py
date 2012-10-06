# coding=utf-8
"""
Functions for working with adjoint matrices

.. math::
    H = [ R p
          0 1 ]
    Ad(H) = [  R   0
              pxR  R ]
"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")


import numpy as np
def isadjointmatrix(a):
    """Return true if a is an adjoint matrix
    """
#TODO: do a better check
    return (a.shape == (6, 6)) and (
        np.linalg.det(a[0:3, 0:3])==1) and (
        a[0:3, 0:3]==a[3:6, 3:6]).all() and (
        a[0:3, 3:6]==np.zeros((3, 3))).all()


def inv(Ad):
    """
    Invert an adjoint matrix
    """
    R = Ad[0:3, 0:3].transpose()
    pxR = Ad[3:6, 0:3].transpose()
    
    invAd = np.zeros((6,6))
    invAd[0:3, 0:3] = R
    invAd[3:6, 0:3] = pxR
    invAd[3:6, 3:6] = R
    return invAd

