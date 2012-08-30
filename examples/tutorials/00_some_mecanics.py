#!/usr/bin/python
#coding=utf-8
#author=joseph salini

"""This tutorial presents some mecanics with Arboris-python.

Matrices and vectors are representeed with numpy arrays.
Here, we present some functions and operations to do mechanics.
"""

import numpy as np

np.set_printoptions(suppress=True)


##### About homogeneous matrix #################################################

import arboris.homogeneousmatrix as Hg


H_o_a = Hg.transl(1,2,3)        # get a translation homogeneous matrix
H_a_b = Hg.rotzyx(.1,.2,.3)     # get a rotation R = Rz * Ry * Rx
                                # also Hg.rotzy  R = Rz * Ry
                                #      Hg.rotzx  R = Rz * Rx
                                #      Hg.rotyx  R = Ry * Rx
                                #      Hg.rotz   R = Rz
                                #      Hg.roty   R = Ry
                                #      Hg.rotx   R = Rx


H_o_b = np.dot(H_o_a, H_a_b)       # H from {o} to {b} is H{o}->{a} * H{a}->{b}


H_b_o = Hg.inv(H_o_b)           # obtain inverse, H from {b} to {o}
print("H_o_b * H_b_o\n", np.dot(H_o_b, H_b_o))


isHM =Hg.ishomogeneousmatrix(H_o_b)   # check validity of homogeneous matrix
print("is homogeneous matrix?", isHM)


p_b = np.array([.4,.5,.6])         # point p expressed in frame {b}
p_o = Hg.pdot(H_o_b, p_b)       # point p expressed in frame {o}


v_b = np.array([.4,.5,.6])         # idem for vector, here affine part is not
v_o = Hg.vdot(H_o_b, v_b)       # taken into account


Ad_o_b = Hg.adjoint(H_o_b)      # to obtain the adjoint related to displacement
Ad_b_o = Hg.iadjoint(H_o_b)     # to obtain the adjoint of the inverse




##### About adjoint matrix #####################################################

import arboris.adjointmatrix as Adm


isAd = Adm.isadjointmatrix(Ad_b_o)  # check validity of adjoint matrix
print("is adjoint matrix?", isAd)


Ad_b_o = Adm.inv(Ad_o_b)            # get inverse of adjoint matrix




##### About twist ##############################################################

rot_velocity = [.1,.2,.3]
lin_velocity = [.4,.5,.6]
twist = np.array(rot_velocity + lin_velocity) # rotation velocity first
print("twist", twist)



##### About mass matrix ########################################################

import arboris.massmatrix as Mm


# mass matrix expressed in principal inertia frame, length in "m", mass in "kg"
M_com = Mm.sphere(radius=.1, mass=10)
M_com = Mm.ellipsoid(radii=(.1, .2, .3), mass=10)
M_com = Mm.cylinder(length=.1, radius=.2, mass=10)  # revolution axis along z
M_com = Mm.box(half_extents=(.1, .2, .3), mass=10)



isMm = Mm.ismassmatrix(M_com)       # check validity of mass matrix
print("is mass matrix?", isMm)


H_com_a= Hg.transl(.4,.5,.6)        # translation from {a} (new frame) to {com}
M_a = Mm.transport(M_com, H_com_a)


H_a_com = Mm.principalframe(M_a)    # return principal inertia frame from {a}
print("H_com_a * H_a_com\n", np.dot(H_com_a, H_a_com))


