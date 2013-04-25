#!/usr/bin/env python

# coding=utf-8



import os
import warnings

from arboris.core import World, name_all_elements
import arboris._visu

import collada
from collada.common import E, tag

import numpy as np
from numpy import all as np_all


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
    world.update_geometric()
    
    drawer = arboris._visu.Drawer(wsDaenimColladaDriver(dae_filename, scale, options), flat, color_generator)
    world.parse(drawer)
    drawer.finish()




def write_collada_animation(collada_animation, collada_scene, sim_file, file_type=None, hdf5_group="/"):
    """Combine a collada scene and an HDF5 file into a collada animation.

    :param collada_animation: path of the output collada animation file
    :type collada_animation: str
    :param collada_scene: path of the input collada scene file
    :type collada_scene: str
    :param hdf5_file: path of the input HDF5 file.
    :type hdf5_file: str
    :param hdf5_group: subgroup within the HDF5 file. Defaults to "/".
    :type hdf5_group: str
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

    write_collada_animation_from_transforms(collada_animation, collada_scene, timeline, transforms)



def write_collada_animation_from_transforms(collada_animation, collada_scene, timeline, transforms):
    """
    """
    def _create_anim_src_elem(anim_name, src_data, src_suffix, src_type, param_name, param_type):
        """"""
        idName = anim_name +"-"+src_suffix
        src_elem = E.source(id=idName)
        
        idName += "-array"
        count = len(src_data)/16 if param_type=="float4x4" else len(src_data)
        src_elem.append( E(src_type+"_array", " ".join(map(str, src_data)), id=idName, count=str(count)) )
        
        src_elem.append( E.technique_common(
                            E.accessor(
                                E.param(name=param_name, type=param_type),
                                source="#"+idName, count=str(count)
                            )
                         )
                       )
        
        if param_type=="float4x4":
            src_elem.find( tag("technique_common") ).find( tag("accessor") ).set("stride", str(16))
        
        return src_elem

    def _create_anim_elem(name, timeline, value):
        """"""
        idName=name+"-matrix"
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








def add_shapes_from_dae(input_file, added_shapes, output_file=None):
    """
    """
    if output_file is None:
        output_file = input_file

    dae = collada.Collada(input_file)
    dae_nodes = _get_all_nodes(dae.scene)

    if isinstance(added_shapes, str):
        shapes_dae = collada.Collada(added_shapes)
        shapes_dae_nodes = _get_all_nodes(shapes_dae.scene)

        # direct method: save all geometries, effects and materials in the new dae
        dae.geometries.extend(shapes_dae.geometries)
        dae.effects.extend(shapes_dae.effects)
        dae.materials.extend(shapes_dae.materials)

        for nid, node in dae_nodes.items():
            if nid in shapes_dae_nodes:
                node.children.extend(shapes_dae_nodes[nid].children)
            else:
                print "warning: cannot find shapes for node: '",nid, "'"

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
        collect[root.id] = root
        for c in root.children:
            collect.update( _get_all_nodes(c) )
    
    return collect










class wsDaenimColladaDriver(arboris._visu.DrawerDriver):
    """ It is a collada driver dedicated to webDaenim, meaning that it produced
    a light collada, with minimal information.
    """
    def __init__(self, filename, scale=1., options=None):
        """
        """
        arboris._visu.DrawerDriver.__init__(self, scale)
        self.filename = filename
        self.dae = collada.Collada()
        
        self.ground_node = collada.scene.Node("ground")
        self.scene = collada.scene.Scene("myscene", [self.ground_node])
        self.dae.scenes.append(self.scene)
        self.dae.scene = self.scene
        self.shapes_dae = collada.Collada(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'simple_shapes.dae') )
        
        self._materials = {}
        self._matnode_i = 0
        self._names     = []

    def init(self):
        pass

    def _get_valid_name(self, name):
        if not name:
            #name = "id_{}".format(len(self._names))
            return ""
        new_name = name
        if new_name in self._names:
            for i in range(100):
                new_name = name + "_{}".format(i)
                if new_name not in self._names:
                    break
            print "warning: name collision: "+name+": modify name to "+new_name
        self._names.append(new_name)
        return new_name

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
            mat = self._materials[color]
        else:
            i = str(len(self._materials))
            effect = collada.material.Effect("effect_"+i, [], "phong", diffuse=(0,0,0), specular=color)
            mat = collada.material.Material("material_"+i, "material", effect)
            self.dae.effects.append(effect)
            self.dae.materials.append(mat)
            self._materials[color] = mat
        matNode = collada.scene.MaterialNode("material_node_"+str(self._matnode_i), mat, inputs=[])
        self._matnode_i +=1
        return matNode

    def _add_new_geometry(self, name, color, geom):
        """ TODO"""
        node = collada.scene.Node(name)
        matnode = self._add_new_material(color)
        geom_instance = collada.scene.GeometryNode(geom, [matnode])
        node.children.append(geom_instance)
        return node

    def add_ground(self, up):
        self._set_up_axis(up)
        return self.ground_node

    def finish(self):
        _write_pycollada_in_file(self.dae, self.filename)


    def add_child(self, parent, child, category=None):
        if parent is not None and child is not None:
            parent.children.append(child)

    def create_transform(self, pose, is_constant, name=None):
        name = self._get_valid_name(name)
        node = collada.scene.Node(name)
        tranform_matrix = collada.scene.MatrixTransform(pose.flatten())
        tranform_matrix.xmlnode.set("sid", "matrix")
        node.transforms.append(tranform_matrix)
        return node

    def create_ellipsoid(self, radii, color, name=None):
        name = self._get_valid_name(name)
        if self.dae.geometries.get("sphere_geometry-mesh") is None:
            self.dae.geometries.append(self.shapes_dae.geometries.get("sphere_geometry-mesh"))
        sphere_geom = self.dae.geometries.get("sphere_geometry-mesh")
        
        node = self._add_new_geometry(name, color, sphere_geom)
        node.transforms.append(collada.scene.ScaleTransform(*radii))
        return node

    def create_box(self, half_extents, color, name=None):
        name = self._get_valid_name(name)
        if self.dae.geometries.get("box_geometry-mesh") is None:
            self.dae.geometries.append(self.shapes_dae.geometries.get("box_geometry-mesh"))
        box_geom = self.dae.geometries.get("box_geometry-mesh")
        
        node = self._add_new_geometry(name, color, box_geom)
        node.transforms.append(collada.scene.ScaleTransform(*half_extents))
        return node

    def create_cylinder(self, length, radius, color, name=None):
        name = self._get_valid_name(name)
        if self.dae.geometries.get("cylinder_geometry-mesh") is None:
            self.dae.geometries.append(self.shapes_dae.geometries.get("cylinder_geometry-mesh"))
        cylinder_geom = self.dae.geometries.get("cylinder_geometry-mesh")
        
        node = self._add_new_geometry(name, color, cylinder_geom)
        node.transforms.append(collada.scene.ScaleTransform(radius, radius, length))
        return node

    def create_plane(self, coeffs, color, name=""):
        pass

    def create_inertia(self, inertia, color, name=""):
        pass

    def create_frame_arrows(self):
        pass

    def create_line(self, start, end, color, name=""):
        pass


