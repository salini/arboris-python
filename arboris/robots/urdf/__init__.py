#!/usr/bin/env python

"""
"""

from arboris.robots.urdf import urdf_parser

import arboris
import arboris.joints
import arboris.shapes
import arboris.constraints

from arboris.homogeneousmatrix import rotz, roty, rotx

import numpy as np
np.set_printoptions(suppress=True)

#import scipy.linalg

import os

import arboris.visu
arboris_simple_shapes_path = os.path.dirname(arboris.visu.__file__) + os.sep + "simple_shapes.dae"

#def closest_rotation_matrix(Rapp):
#    return np.dot(Rapp,  np.linalg.inv( scipy.linalg.sqrtm(np.dot(Rapp.T, Rapp)) ) )


def roll_pitch_yaw_to_rotation_matrix(roll, pitch, yaw):
    """
    """
    R = np.dot( rotz(yaw), np.dot( roty(pitch), rotx(roll) ) )[0:3,0:3]
    return R

def rotation_matrix_to_roll_pitch_yaw(R):
    """
    """
    r =   np.arctan2(R[1,2], R[2,2])
    p = - np.arcsin( R[0,2])
    y =   np.arctan2(R[0,1], R[0,0])

    return np.array([r, p, y])


def to_arboris( obj , default=None):
    """
    """
    #print "obj:", obj.__class__ #, obj
    
    if   isinstance(obj, urdf_parser.Link):
        body = arboris.core.Body()
        body.name = obj.name
        body.mass = to_arboris(obj.inertial)
        return body

    elif isinstance(obj, urdf_parser.Inertial):
        m = obj.mass
        H_com_body = to_arboris(obj.origin, np.eye(4))
        xx, yy, zz, xy, xz, yz = [obj.matrix["i"+n] for n in ["xx", "yy", "zz", "xy", "xz", "yz"]]
        m  = max(1e-6, m)
        xx = max(1e-6, xx)
        yy = max(1e-6, yy)
        zz = max(1e-6, zz)
        M_com = np.array([[xx, xy, xz, 0., 0., 0.],
                          [xy, yy, yz, 0., 0., 0.],
                          [xz, yz, zz, 0., 0., 0.],
                          [0., 0., 0., m , 0., 0.],
                          [0., 0., 0., 0., m , 0.],
                          [0., 0., 0., 0., 0., m ]])
        return arboris.massmatrix.transport(M_com, H_com_body)

    elif isinstance(obj, urdf_parser.Pose):
        H          = arboris.homogeneousmatrix.transl( *obj.position)
        H[0:3,0:3] = roll_pitch_yaw_to_rotation_matrix(*obj.rotation)
        return H

    elif isinstance(obj, urdf_parser.Joint):
        joint = {}
        joint["parent"]         = obj.parent
        joint["child"]          = obj.child
        joint["H_parent_child"] = to_arboris(obj.origin, np.eye(4))
        axis = map(float, obj.axis.split()) if len(obj.axis.split())==3 else obj.axis

        if   obj.joint_type == urdf_parser.Joint.REVOLUTE:
            if   (axis in ["x", "X"]) or ( np.allclose(axis, [1,0,0])):
                joint["type"] = arboris.joints.RxJoint
            elif (axis in ["y", "Y"]) or ( np.allclose(axis, [0,1,0])):
                joint["type"] = arboris.joints.RyJoint
            elif (axis in ["z", "Z"]) or ( np.allclose(axis, [0,0,1])):
                joint["type"] = arboris.joints.RzJoint
            else:
                joint["H_child_joint"] = arboris.homogeneousmatrix.zaligned(axis)
                joint["type"]          = arboris.joints.RzJoint

        elif obj.joint_type == urdf_parser.Joint.PRISMATIC:
            if   (axis in ["x", "X"]) or ( np.allclose(axis, [1,0,0])):
                joint["type"] = arboris.joints.TxJoint
            elif (axis in ["y", "Y"]) or ( np.allclose(axis, [0,1,0])):
                joint["type"] = arboris.joints.TyJoint
            elif (axis in ["z", "Z"]) or ( np.allclose(axis, [0,0,1])):
                joint["type"] = arboris.joints.TzJoint
            else:
                joint["H_child_joint"] = arboris.homogeneousmatrix.zaligned(axis)
                joint["type"]          = arboris.joints.TzJoint
        
        elif obj.joint_type == urdf_parser.Joint.FIXED:
            joint["type"] = arboris.joints.FixedJoint

        elif obj.joint_type == urdf_parser.Joint.FLOATING:
            print "WARNING: urdf containt free joint; this joint is deprecated."
            joint["type"] = arboris.joints.FreeJoint

        else:
            raise KeyError, "Joint type: '"+obj.joint_type+"' is unknown or not not implemented yet."

        return joint

    elif isinstance(obj, urdf_parser.Visual):
        visual              = to_arboris(obj.geometry)
        visual["transform"] = to_arboris(obj.origin, np.eye(4))
        #material     = to_arboris(obj.material)
        return visual

    elif isinstance(obj, urdf_parser.Collision):
        collision              = to_arboris(obj.geometry)
        collision["transform"] = to_arboris(obj.origin, np.eye(4))
        return collision

    elif isinstance(obj, urdf_parser.Mesh):
        mesh_path = obj.filename
        scale = 1 if obj.scale is None else map(float, obj.scale.split())
        return {"shape": "mesh", "mesh_from_urdf": mesh_path,  "scale": scale}

    elif isinstance(obj, urdf_parser.Box):
        mesh_path = arboris_simple_shapes_path + "#simple_box_node"
        scale     = obj.dims
        return {"shape": "box", "mesh": mesh_path,  "scale": scale}

    elif isinstance(obj, urdf_parser.Sphere):
        mesh_path = arboris_simple_shapes_path + "#simple_sphere_node"
        scale     = obj.radius
        return {"shape": "sphere", "mesh": mesh_path,  "scale": scale}

    elif isinstance(obj, urdf_parser.Cylinder):
        mesh_path = arboris_simple_shapes_path + "#simple_cylinder_node"
        scale     = [obj.radius, obj.radius, obj.length]
        return {"shape": "cylinder", "mesh": mesh_path,  "scale": scale}

    elif isinstance(obj, None.__class__):
        if default is not None:
            return default

    else:
        print "WARNING: Cannot load object", obj.__class__, obj



class URDFConverter(object):
    """
    """
    def __init__(self, urdf_file_name=None, world=None, H_init=None, fixed_base=False, prefix="", suffix="", root_joint_name="root_joint", save_joint_limits=True):
        """
        """
        self.urdf_file_name = urdf_file_name
        
        self.fixed_base = fixed_base
        self.H_init     = H_init

        self.prefix = prefix
        self.suffix = suffix
        self.root_joint_name   = root_joint_name
        self.save_joint_limits = save_joint_limits

        if urdf_file_name is not None:
            self.parseURDF(urdf_file_name)

        if world is not None:
            self.load_in_world(world)


    def parseURDF(self, urdf_file_name):
        self.robot = urdf_parser.URDF.load_xml_file(urdf_file_name)


    def load_in_world(self, world):
        """
        """
        list_of_bodies = {}

        if self.robot is None:
            raise ValueError, "no URDF file has been loaded"

        # save all bodies
        for body_name, urdf_body in self.robot.links.items():

            name = self.prefix + body_name + self.suffix

            body = to_arboris(urdf_body)
            body.name = name
            list_of_bodies[name] = body

            # save all shapes for collision; we only want physical shapes, not visual shapes
            # visual shapes can be loaded later with function 'get_visual_shapes'
            collision = to_arboris(urdf_body.collision, None)
            
            if collision and (collision["shape"] is not "mesh"):

                H_body_shape = collision["transform"]
                shape_frame = arboris.core.SubFrame( list_of_bodies[name], H_body_shape )

                if   collision["shape"] == "box":
                    half_dims = np.array(collision["scale"])/2.
                    shape = arboris.shapes.Box( shape_frame, half_dims )
                elif collision["shape"] == "sphere":
                    radius = collision["scale"]
                    shape = arboris.shapes.Sphere( shape_frame, radius )
                elif collision["shape"] == "cylinder":
                    length = collision["scale"][2]
                    radius = collision["scale"][0]
                    shape = arboris.shapes.Cylinder( shape_frame, length, radius )

                world.register(shape)


        # save all joints
        for joint_name, urdf_joint in self.robot.joints.items():

            name = self.prefix + joint_name + self.suffix

            joint = to_arboris(urdf_joint)
            joint["type"].name = name

            parent = list_of_bodies[joint["parent"]]
            child  = list_of_bodies[joint["child"] ]

            if "H_child_joint" in joint: # it means the the axis is not along x, y or z
                H_parent_joint = np.dot( joint["H_parent_child"], joint["H_child_joint"])
                H_child_joint  =         joint["H_child_joint"]
                parent_joint_frame = arboris.core.SubFrame( parent, H_parent_joint )
                child_joint_frame  = arboris.core.SubFrame( child,  H_child_joint  )
            else:
                H_parent_joint = joint["H_parent_child"]
                parent_joint_frame = arboris.core.SubFrame( parent, H_parent_joint )
                child_joint_frame  = child

            joint_instance = joint["type"](name=name)
            world.add_link( parent_joint_frame, joint_instance, child_joint_frame )

            # save all joints limits if it has been requested
            if self.save_joint_limits is True:

                if urdf_joint.joint_type in [urdf_parser.Joint.REVOLUTE, urdf_parser.Joint.PRISMATIC]:
                    lower = urdf_joint.limits.lower
                    upper = urdf_joint.limits.upper
                    
                    if (lower is not None) and (upper is not None):
                        constraint = arboris.constraints.JointLimits(joint_instance, lower, upper)
                        
                        world.register(constraint)

                else:
                    print "WARNING: cannot save joint limit constraint for joint "+joint_name+" ("+urdf_joint.joint_type+")"


        # connect the root body of the robot with the ground frame
        root_body = list_of_bodies[self.robot.get_root()]
        ground = world.ground

        if self.H_init is not None:
            ground = arboris.core.SubFrame( world.ground, self.H_init)

        name = self.prefix + self.root_joint_name + self.suffix
        if self.fixed_base is True:
            world.add_link( ground, arboris.joints.FixedJoint(name=name), root_body )
        else:
            world.add_link( ground, arboris.joints.FreeJoint(name=name), root_body )

        world.init()


    def get_visual_shapes(self):
        """
        """
        visual_mapping = []
        urdf_path = os.path.dirname(self.urdf_file_name) + os.sep

        for body_name, urdf_body in self.robot.links.items():
            name = self.prefix + body_name + self.suffix

            if urdf_body.visual is not None:

                body_shape = to_arboris(urdf_body.visual)
                body_shape["frame"] = name

                if ("mesh" not in body_shape):
                    body_shape["mesh"] = urdf_path + body_shape["mesh_from_urdf"]

                visual_mapping.append( body_shape )

        return visual_mapping

    def get_collision_shapes(self):
        """
        """
        visual_mapping = []
        urdf_path = os.path.dirname(self.urdf_file_name) + os.sep

        for body_name, urdf_body in self.robot.links.items():
            name = self.prefix + body_name + self.suffix

            if urdf_body.collision is not None:

                body_shape = to_arboris(urdf_body.collision)
                body_shape["frame"] = name

                if ("mesh" not in body_shape):
                    body_shape["mesh"] = urdf_path + body_shape["mesh_from_urdf"]

                visual_mapping.append( body_shape )

        return visual_mapping


    def get_joint_limits(self):
        """
        """
        joint_limits = {}

        for joint_name, urdf_joint in self.robot.joints.items():

            name = self.prefix + joint_name + self.suffix

            lower = urdf_joint.limits.lower
            upper = urdf_joint.limits.upper
            if (lower is not None) and (upper is not None):
                joint_limits[name] = (lower, upper)

        return joint_limits

    def get_effort_limits(self):
        """
        """
        effort_limits = {}

        for joint_name, urdf_joint in self.robot.joints.items():

            name = self.prefix + joint_name + self.suffix
            effort_limits[name] = urdf_joint.limits.effort

        return joint_limits

    def get_velocity_limits(self):
        """
        """
        velocity_limits = {}

        for joint_name, urdf_joint in self.robot.joints.items():

            name = self.prefix + joint_name + self.suffix
            velocity_limits[name] = urdf_joint.limits.velocity

        return velocity_limits



