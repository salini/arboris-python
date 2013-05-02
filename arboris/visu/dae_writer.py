#!/usr/bin/env python

# coding=utf-8



import os
import warnings

from arboris.core import World, name_all_elements
from arboris.massmatrix import principalframe, transport
from arboris.visu.world_drawer import Drawer, DrawerDriver, ColorGenerator
from arboris.homogeneousmatrix import rotzyx_angles, zaligned

import collada
from   collada.common import E, tag

import numpy as np
from   numpy import all as np_all


import pickle as pkl
try:
    import h5py
except ImportError:
    pass



def write_collada_scene(world, dae_filename, scale=1, options=None, flat=False, color_generator=None):
    """
    """
    assert isinstance(world, World)
    if flat:
        name_all_elements(world.getbodies(), True)
    else:
        nonflatlist = [j.frames[1] for j in world.getjoints()]
        name_all_elements(nonflatlist, True)
    name_all_elements(world.itermovingsubframes(), True)
    
    if color_generator is None:
        color_generator = ColorGenerator( nb_colors=len(world.getbodies()) )
    
    world.update_geometric()
    
    drawer = Drawer(pydaenimColladaDriver(dae_filename, scale, options), flat, color_generator)
    world.parse(drawer)
    drawer.finish()




def write_collada_animation(collada_animation, collada_scene, sim_file, file_type=None, hdf5_group="/", precision=8):
    """Combine a collada scene and an HDF5 file into a collada animation.

    :param collada_animation: path of the output collada animation file
    :type collada_animation: str
    :param collada_scene: path of the input collada scene file
    :type collada_scene: str
    :param hdf5_file: path of the input HDF5 file.
    :type hdf5_file: str
    :param hdf5_group: subgroup within the HDF5 file. Defaults to "/".
    :type hdf5_group: str
    :param precision: rounding precision when saving animation data.
    :type precision: int
    """
    if file_type is None:
        file_type = sim_file.split(".")[-1]

    if file_type in ["hdf5", "h5"]:
        try:
            sim=h5py.File(sim_file, "r")
            timeline = np.array(sim[hdf5_group]["timeline"])
            transforms = {}
            for k,v in sim[hdf5_group]["transforms"].items():
                transforms[k] = np.array(v)
            sim.close()
        except NameError:
            raise ImportError("Cannot load file '"+sim_file+"'. h5py may not be installed.")
        except IOError:
            raise IOError("Cannot load file '"+sim_file+"'. It may not be a hdf5 file.")
    elif file_type in ["pickle", "pkl"]:
        try:
            f = open(sim_file,'r')
            sim = pkl.load(f)
            timeline = sim["timeline"]
            transforms = sim["transforms"]
            f.close()
        except IOError:
            raise IOError("Cannot load file '"+sim_file+"'. It may not be a pickle file.")

    write_collada_animation_from_transforms(collada_animation, collada_scene, timeline, transforms, precision)



def write_collada_animation_from_transforms(collada_animation, collada_scene, timeline, transforms, precision):
    """
    """
    def _create_anim_src_elem(anim_name, src_data, src_suffix, src_type, param_name, param_type):
        """"""
        idName = anim_name +"-"+src_suffix
        src_elem = E.source(id=idName)
        
        idName += "-array"
        count = len(src_data)/16 if param_type=="float4x4" else len(src_data)
        
        if src_type == "float":
            str_src_data = " ".join([str(round(val, precision)) for val in src_data])+" "
            str_src_data = str_src_data.replace(".0 ", " ")
        else:
            str_src_data = " ".join(map(str, src_data))
        src_elem.append( E(src_type+"_array", str_src_data, id=idName, count=str(count)) )
        
        src_elem.append( E.technique_common(
                            E.accessor(
                                E.param(name=param_name, type=param_type),  #accessor child
                                source="#"+idName, count=str(count)         #accessor attribs
                            )
                         )
                       )
        
        if param_type=="float4x4":
            src_elem.find( tag("technique_common") ).find( tag("accessor") ).set("stride", str(16))
        
        return src_elem

    def _create_anim_elem(name, timeline, value):
        """"""
        idName=name+"-animation"
        animElem = E.animation(id=idName)
        
        animElem.append( _create_anim_src_elem(idName, timeline              , "input"        , "float", "TIME", "float") )
        animElem.append( _create_anim_src_elem(idName, ["STEP"]*len(timeline), "interpolation", "Name" , "INTERPOLATION", "name") )
        animElem.append( _create_anim_src_elem(idName, value.flatten()       , "output"       , "float", "TRANSFORM", "float4x4") )

        animElem.append( E.sampler( *[E.input( semantic=sem.upper(), source="#"+idName+"-"+sem) for sem in ["input", "output", "interpolation"]], id=idName+"-sampler") )

        animElem.append( E.channel(source="#"+idName+"-sampler", target=name+"/matrix") )

        return animElem


    dae = collada.Collada(collada_scene)
    # save animation node: this is not supported by pycollada, so the xml node is written using pycollada.common.E
    anim_lib = dae.xmlnode.find( tag('library_animations') )
    if anim_lib is None:
        children = list(dae.xmlnode.getroot())
        dae.xmlnode.getroot().insert( children.index( dae.xmlnode.find( tag("scene"))), E('library_animations'))
        
        anim_lib = dae.xmlnode.find( tag('library_animations') )

    for name, val in transforms.items():
        anim_lib.append( _create_anim_elem(name, timeline, val) )

    _write_pycollada_in_file(dae, collada_animation)



def add_shapes_to_dae(input_file, added_shapes, output_file=None):
    """
    """
    if output_file is None:
        output_file = input_file

    dae = collada.Collada(input_file)
    dae_nodes = _get_all_nodes(dae.scene)

    if isinstance(added_shapes, str):
        #it means that this is an entire collada file
        shapes_dae       = collada.Collada(added_shapes)
        shapes_dae_nodes = _get_all_nodes(shapes_dae.scene)

        for node_id, node in dae_nodes.items():
            if node_id in shapes_dae_nodes:
                #create node that will contain scale, transform and instance_node
                instance_node = _get_void_node() 
                node.children.append(instance_node)
                
                child_shape_node = collada.scene.Node(_get_good_id_uri(added_shapes+".."+node_id), children = shapes_dae_nodes[node_id].children)
                child_shape_node.transforms.extend( shapes_dae_nodes[node_id].transforms )
                
                _save_all_descendant_geometryNode(child_shape_node, dae)
                instance_node.children.append(collada.scene.NodeNode(child_shape_node))
                if child_shape_node.id not in dae.nodes:
                    dae.nodes.append(child_shape_node)

            else:
                print "warning: cannot find shapes for node: '",node_id, "'"

    else:
        for mesh_info in added_shapes:
        
            if isinstance(mesh_info, dict):
                mesh_data = [mesh_info["frame"], mesh_info["mesh"]]
                if "transform" in mesh_info:
                    mesh_data.append(mesh_info["transform"])
                if "scale" in mesh_info:
                    mesh_data.append(mesh_info["scale"])
            else:
                mesh_data = mesh_info

            parent_node_id, shape_path = mesh_data[0:2]
            shape_file, sep, shape_id  = shape_path.partition("#")
            shapes_dae       = collada.Collada(shape_file)
            shapes_dae_nodes = _get_all_nodes(shapes_dae.scene)

             #check whether parent exist
            if parent_node_id not in dae_nodes:
                print "warning: cannot find node in input file: '", parent_node_id, "'"

             #check whether shape exists
            elif (shape_id != "") and (shape_id not in shapes_dae_nodes):
                print "warning: cannot find shape in collada file: '", shape_id, "'"

            else:
                 #create node that will contain scale, transform and instance_node
                instance_node = _get_void_node()
                dae_nodes[parent_node_id].children.append(instance_node)

                 #it means that user selects all the scene children
                if shape_id == "":
                    child_shape_node = collada.scene.Node(shape_id, children = shapes_dae.scene.nodes)

                #in this case, the shape_path is on the form: 'shape_file.dae#shape_id'
                else:
                    child_shape_node = collada.scene.Node(_get_good_id_uri(shape_path), children = shapes_dae_nodes[shape_id].children)
                    child_shape_node.transforms.extend( shapes_dae_nodes[shape_id].transforms )

                _save_all_descendant_geometryNode(child_shape_node, dae)
                instance_node.children.append(collada.scene.NodeNode(child_shape_node))
                if child_shape_node.id not in dae.nodes:
                    dae.nodes.append(child_shape_node)

                if len(mesh_data) >= 3:
                    #add a matrix tranform Node
                    H_frame_shape = mesh_data[2]
                    instance_node.transforms.append(collada.scene.MatrixTransform(H_frame_shape.flatten()))

                if len(mesh_data) >= 4:
                    #add a scale tranform Node
                    scale = mesh_data[3]
                    if isinstance(scale, (int, float, long)):
                        scale = [scale]*3
                    instance_node.transforms.append(collada.scene.ScaleTransform(*scale))


    _write_pycollada_in_file(dae, output_file)







def _write_pycollada_in_file(dae, output_file):
    """
    """
    with open(output_file, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>')
        dae.write(f)


def _get_all_nodes(root):
    """
    """
    collect = {}
    if   isinstance(root, collada.scene.Scene):
        for n in root.nodes:
            collect.update( _get_all_nodes(n) )

    elif isinstance(root, collada.scene.Node):
        if root.id is not None:
            collect[root.id] = root
        for c in root.children:
            collect.update( _get_all_nodes(c) )
    
    return collect



def _get_void_node(children=None):
    node = collada.scene.Node("", children=children)
    node.id = None
    node.xmlnode.attrib.pop("name", None)
    node.xmlnode.attrib.pop("id"  , None)
    return node


def _save_all_descendant_geometryNode(src_node, dest_dae):
    """
    """
    if isinstance(src_node, collada.scene.GeometryNode):
        if src_node.geometry.id not in dest_dae.geometries:
            dest_dae.geometries.append(src_node.geometry)
        for mat_node in src_node.materials:
            if mat_node.target.id not in dest_dae.materials:
                dest_dae.materials.append(mat_node.target)
            if mat_node.target.effect.id not in dest_dae.effects:
                dest_dae.effects.append(mat_node.target.effect)
    
    elif isinstance(src_node, collada.scene.Node):
        for child in src_node.children:
            _save_all_descendant_geometryNode(child, dest_dae)
    
    elif isinstance(src_node, collada.scene.Scene):
        for child in src_node.nodes:
            _save_all_descendant_geometryNode(child, dest_dae)


def _get_good_id_uri(uri):
    """
    """
    return "node_"+uri.replace("\\", "_").replace("/", "_").replace("#", "..").replace(":", "..")



class pydaenimColladaDriver(DrawerDriver):
    """ It is a collada driver dedicated to wsDaenim, meaning that it produces
    a light collada, with minimal information.
    """
    def __init__(self, filename, scale=1., options=None):
        """
        """
        DrawerDriver.__init__(self, scale)
        self.filename   = filename

        self.dae        = collada.Collada()
        self.shapes_dae = collada.Collada(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'simple_shapes.dae') )

        # create library_physics_models
        self.physics_model = E.physics_model(id="world")
        self.dae.xmlnode.getroot().insert(1, E.library_physics_models(self.physics_model)) # insert just after asset

        self._materials = {}
        self._names     = []

    def init(self):
        pass

    def _get_collada_node(self, name):
        if not name:
            node = _get_void_node()
            return node

        new_name = name
        if new_name in self._names:
            for i in range(100):
                new_name = name + "_{}".format(i)
                if new_name not in self._names:
                    break
            print "warning: name collision: "+name+": modify name to "+new_name
        self._names.append(new_name)
        node = collada.scene.Node(new_name)
        return node

    def _set_up_axis(self, up):
        """Add an up_axis element."""
        if   np_all(up == [1., 0., 0.]): up_elem = collada.asset.UP_AXIS.X_UP
        elif np_all(up == [0., 1., 0.]): up_elem = collada.asset.UP_AXIS.Y_UP
        elif np_all(up == [0., 0., 1.]): up_elem = collada.asset.UP_AXIS.Z_UP
        else:
            up_elem = up_elem = collada.asset.UP_AXIS.Y_UP
            warnings.warn('the up vector ('+up+') is not compatible with collada, set default.')
        self.dae.assetInfo.upaxis = up_elem

    def _add_new_material(self, color):
        """ Add a new color with effect and material."""
        if color in self._materials:
            idx, mat = self._materials[color]
        else:
            idx = str(len(self._materials))
            effect = collada.material.Effect("effect_"+idx, [], "phong", diffuse=color)
            mat    = collada.material.Material("material_"+idx, "material", effect)
            self.dae.effects.append(effect)
            self.dae.materials.append(mat)
            self._materials[color] = idx, mat
        matNode = collada.scene.MaterialNode("material", mat, inputs=[])
        return matNode

    def _add_new_geometry(self, name, color, geom):
        """ TODO"""
        node = self._get_collada_node(name)
        matnode = self._add_new_material(color)
        geom_instance = collada.scene.GeometryNode(geom, [matnode])
        node.children.append(geom_instance)
        return node

    def add_ground(self, up):
        self._set_up_axis(up)
        self.ground_node = collada.scene.Node("ground")
        
        tranform_matrix = collada.scene.MatrixTransform(np.eye(4).flatten())
        tranform_matrix.xmlnode.set("sid", "matrix")
        self.ground_node.transforms.append(tranform_matrix)
        
        self.scene = collada.scene.Scene("myscene", [self.ground_node])
        self.dae.scenes.append(self.scene)
        self.dae.scene = self.scene
        return self.ground_node

    def finish(self):
        _write_pycollada_in_file(self.dae, self.filename)


    def add_child(self, parent, child, category=None):
        if parent is not None and child is not None:
            parent.children.append(child)

    def create_transform(self, pose, is_constant, name=None):
        node = self._get_collada_node(name)
        tranform_matrix = collada.scene.MatrixTransform(pose.flatten())
        tranform_matrix.xmlnode.set("sid", "matrix")
        node.transforms.append(tranform_matrix)
        return node

    def create_ellipsoid(self, radii, color, name=None):
        if self.dae.geometries.get("sphere_geometry-mesh") is None:
            self.dae.geometries.append(self.shapes_dae.geometries.get("sphere_geometry-mesh"))
        sphere_geom = self.dae.geometries.get("sphere_geometry-mesh")
        
        node = self._add_new_geometry(name, color, sphere_geom)
        node.transforms.append(collada.scene.ScaleTransform(*radii))
        return node

    def create_box(self, half_extents, color, name=None):
        if self.dae.geometries.get("box_geometry-mesh") is None:
            self.dae.geometries.append(self.shapes_dae.geometries.get("box_geometry-mesh"))
        box_geom = self.dae.geometries.get("box_geometry-mesh")
        
        node = self._add_new_geometry(name, color, box_geom)
        node.transforms.append(collada.scene.ScaleTransform(*half_extents))
        return node

    def create_cylinder(self, length, radius, color, name=None):
        if self.dae.geometries.get("cylinder_geometry-mesh") is None:
            self.dae.geometries.append(self.shapes_dae.geometries.get("cylinder_geometry-mesh"))
        cylinder_geom = self.dae.geometries.get("cylinder_geometry-mesh")
        
        node = self._add_new_geometry(name, color, cylinder_geom)
        node.transforms.append(collada.scene.ScaleTransform(radius, radius, length))
        return node

    def create_plane(self, coeffs, color, name=""):
        if self.dae.geometries.get("plane_geometry-mesh") is None:
            self.dae.geometries.append(self.shapes_dae.geometries.get("plane_geometry-mesh"))
        plane_geom = self.dae.geometries.get("plane_geometry-mesh")
        
        H = zaligned(coeffs[0:3])
        H[0:3, 3] = coeffs[3] * coeffs[0:3]
        plane_he = self._options["plane half extents"]
        
        node = self._add_new_geometry(name, color, plane_geom)
        node.transforms.append(collada.scene.MatrixTransform(H.flatten()))
        node.transforms.append(collada.scene.ScaleTransform(plane_he[0], plane_he[1], 0.))
        return node

    def create_inertia(self, inertia, color, name=""):
        H_body_com  = principalframe(inertia)
        Mcom        = transport(inertia, H_body_com)
        mass        = Mcom[5,5]
        m_o_inertia = Mcom[[0,1,2], [0,1,2]]
        position    = H_body_com[0:3,3]
        yaw, pitch, roll = np.degrees( rotzyx_angles(H_body_com) ) # in degrees for collada specification

        self.physics_model.append(
            E.rigid_body(
                E.technique_common(
                    E.dynamic("true"),
                    E.mass(str(mass)),
                    E.mass_frame(
                        E.translate(" ".join(map(str, position))  , sid="location"),
                        E.rotate(" ".join(map(str, [0,0,1,yaw]))  , sid="rotationZ"),
                        E.rotate(" ".join(map(str, [0,1,0,pitch])), sid="rotationY"),
                        E.rotate(" ".join(map(str, [1,0,0,roll])) , sid="rotationX"),
                    ),
                    E.inertia(" ".join(map(str, m_o_inertia))),
                ),
            sid=name
            )
        )

    def create_frame_arrows(self):
        pass

    def create_line(self, start, end, color, name=""):
        pass



