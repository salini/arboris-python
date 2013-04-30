#!/usr/bin/python
#coding=utf-8
#author=joseph salini

"""This tutorial presents the structure of Arboris-python.

It presents the World, Body, Joint, SubFrame and Constraint classes.
"""


##### About the World ##########################################################

from arboris.core import World
from arboris.robots import simplearm

w = World()                 # create an instance of world. 
simplearm.add_simplearm(w)  # add a simple arm robot

                                # The world is composed of ...
bodies = w.getbodies()          # ... named list of bodies
joints = w.getjoints()          # ... named list of joints
frames = w.getframes()          # ... named list of frames (including bodies)
shapes = w.getshapes()          # ... named list of shapes
consts = w.getconstraints()     # ... named list of constraints
ctrls  = w.getcontrollers()     # ... named list of controllers
                                # And that's all!
print("bodies", bodies)
print("joints", joints)

# These lists are named, so instances can be retrieved as follows:
body_Arm = bodies["Arm"]
joint_Shoulder = joints["Shoulder"]
print("Arm", body_Arm)
print("Shoulder", joint_Shoulder)


# At the beginning,  the world has one body corresponding to the galilean frame
galilean_frame = w.ground


# world give access to current time and the number of degrees of freedom
t    = w.current_time
ndof = w.ndof


# Given the dynamic equation of tree structure without control:
#
#     M * d(gvel)/dt + (N + B) * gvel = gforce
#
# - dt is given by user.
# - M, N, B, gvel, gforce are computed by world (depending on current state)
M      = w.mass
N      = w.nleffects
B      = w.viscosity
gvel   = w.gvel
gforce = w.gforce


# if the world state is modified (if joints positions or velocities change),
# one has to update the kinematic and dynamic vectors and matrices
w.update_geometric()    # make forward kinematic
w.update_dynamic()      # make forward dynamic


# Simulate the world is done with a loop containing these functions
dt = .01 #s
w.update_dynamic()              # update dynamic of the world
w.update_controllers(dt)        # ...... generalized force of controllers
w.update_constraints(dt)        # ...... constraints for kinematic loops
w.integrate(dt)                 # ...... the current state of the world



##### The Frame class ##########################################################

# This class represents the bodies and the subframes in the world
f = frames["EndEffector"]
name      = f.name
body      = f.body        # return parent body (itself if f is body)
pose      = f.pose        # return H_0_f      (0 is world.ground)
bpose     = f.bpose       # return H_b_f      (b is f.body)
twist     = f.twist       # return T_f_0_f (T of f relative to 0 expressed in f)
jacobian  = f.jacobian    # return J_f_0_f such as T_f_0_f = J_f_0_f * gvel
djacobian = f.djacobian   # return dJ_f_0_f



##### The Body class ###########################################################

# It derives from Frame class, so properties above are available.
# They are rigid and represent "nodes" in the kinematic tree.

# How to create a body
from arboris.core import Body
from arboris.massmatrix import sphere
M_sphere = sphere(radius=.1, mass=1.)
b = Body(mass=M_sphere, viscosity=None, name="myBody")

# body properties
M = b.mass
N = b.nleffects
B = b.viscosity

# information about its position in kinematic structure
p_joint = b.parentjoint         # return the parent joint
c_joints = b.childrenjoints     # return list of chidren joints



##### The SubFrame/MovingSubFrame classes ######################################

from arboris.core import SubFrame, MovingSubFrame
from arboris.homogeneousmatrix import transl

# How to create a fixed subframe
H_b_f = transl(.1,.2,.3) # homogeneous matrix from body to subframe
sf  = SubFrame(body=w.ground, bpose=H_b_f, name="myFrame") # bpose constant

# How to create a moving subframe
#Warning: these are not designed to create the kinematic tree.
#         they are intended to be used as target for example
msf = MovingSubFrame(w.ground, name="myMovingFrame")
msf.bpose = H_b_f # possible only with MovingSubFrame instance



##### The Joint class ##########################################################

# A joint relates a parent frame with a child frame by constraining their
# relative motion.

j = joints["Shoulder"]
pose      = j.pose          # inverse method: ipose = rm.ipose()
twist     = j.twist         # ..............: itwist
adjoint   = j.adjoint       # ..............: iadjoint
adjacency = j.adjacency     # ..............: iadjacency
dadjoint  = j.dadjoint      # ..............: idadjoint

gpos      = j.gpos          # generalized coordinates in joint subspace
gvel      = j.gvel          # generalized velocity ...
ndof      = j.ndof          # number of DoF
dof       = j.dof           # slice of the DoF in the world instance
jacobian  = j.jacobian      # inner Jacobian
djacobian = j.djacobian     # inner time derivative Jacobian


# Example on how to instanciate a new joint
from arboris.joints import RyRxJoint, FreeJoint #there are many others

j = RyRxJoint(gpos=[.1,.2], gvel=[.0,.1], name="RyRxj")
j = FreeJoint(gpos=transl(.1, .2, .3), gvel=[.1,.2,.3,.4,.5,.6], name="freej")



##### The Shape class ##########################################################

# Just some shapes for scene decoration or for collision detection.

from arboris.shapes import Point, Plane, Sphere, Cylinder, Box
f = frames["EndEffector"]
s0 = Point(frame=f, name="thePoint")
s1 = Plane(frame=f, coeffs=(0,1,0,.1), name="thePlane")
s2 = Sphere(frame=f, radius=.1, name="theSphere")
s3 = Cylinder(frame=f, length=1., radius=.1, name="theCylinder") # along z axis
s4 = Box(frame=f, half_extents=(.1, .2, .3), name="theBox")



##### The Constraint class #####################################################

# A constraint are actually a mean to create a loop in the kinematic structure.
# It generates a force to ensure that the loop is closed.

# how instanciate some contraint class
from arboris.constraints import JointLimits, BallAndSocketConstraint, SoftFingerContact
j = joints["Shoulder"]
c = JointLimits(joint=j, min_limits=[-3.14], max_limits=[3.14], name="JLim")

f0,f1 = w.ground, frames["EndEffector"]
c = BallAndSocketConstraint(frames=(f0,f1), name="BaSC")

c = SoftFingerContact(shapes=(s0,s1), friction_coeff= 1., name="SFC")


# every constraints have the following methods/properties
c.ndol          # number of degrees of 'linkage' = 6 - ndof
c.gforce        # generalized force generate by the constraint
c.jacobian      # Jacobian of the constraint

c.enable()      # to enable the constraint
c.disable()     # to disable .............
c.is_enabled()  # return the state of the constraint: True/False
c.is_active()   # return whether the kinematic loop closure is active or not


