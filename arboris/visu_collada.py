# coding=utf-8

import xml.etree.ElementTree as ET
from xml.etree.ElementTree import XML, Element, SubElement, tostring

from numpy import all as np_all, array, linalg

from arboris.massmatrix import principalframe, transport
from arboris.homogeneousmatrix import zaligned

from arboris.core import World, name_all_elements
import arboris._visu
import subprocess
import os
import tempfile
from datetime import datetime
import warnings

import pickle as pkl
try:
    import h5py
except ImportError:
    pass



NS = 'http://www.collada.org/2005/11/COLLADASchema'
SHAPES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'shapes.dae')


def QN(tag):
    """Return the qualified version of a collada tag name.

    Example:

    >>> QN('asset')
    '{http://www.collada.org/2005/11/COLLADASchema}asset'

    """
    assert tag[1] is not '{'
    return "{" + NS + "}" + tag

def indent(tree):
    """Indent the xml tree."""
    def _indent(elem, level):
        """Indent the xml subtree starting at elem."""
        istr = "\t"
        i = "\n" + level*istr
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + istr
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for e in elem:
                _indent(e, level+1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    _indent(tree.getroot(), 0)

def fix_namespace(tree):
    """Fix a tree to avoid namespace aliases/prefixes.

    This function replaces qualified elements tags from the collada namespace
    by the unqualified (or local) version and add the namespace declaration
    to the root element.

    **Background**

    ColladaCoherencyTest requires the collada file to declare the namespace as
    shown in this example::

        <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema"
                 version="1.4.1">
            <asset>
                ...
            </asset>
        </COLLADA>

    Due to this namespace declaration, when ElementTree parses the file, it
    remove the ``xmlns`` attribute from the ``COLLADA`` element and stores
    qualified versions of the tags. For the example ``Element``s' tags would
    be:

    - {http://www.collada.org/2005/11/COLLADASchema}COLLADA
    - {http://www.collada.org/2005/11/COLLADASchema}asset

    There is no way to disable this namespace resolution in ElementTree
    (besides writing a new parser).

    When ElementTree serializes the tree back to an XML file, it generates
    namespace aliases, such as "ns0" for every element.

        <ns0:COLLADA xmlns:ns0="http://www.collada.org/2005/11/COLLADASchema"
                     version="1.4.1">
            <ns0:asset>
                ...
            </ns0:asset>
        </ns0:COLLADA>

    One can customize the prefix, but not disable it (ie. declare a namespace
    to be default.)

    However,collada-dom does not support namespace prefixes and won't load such
    a file. This seems to be a limitation of the whole DTD thing which is not
    namespace-aware.

    In order to write collada-dom compatible files, this function "unqualifies"
    the collada tags and adds back the ``xmlns`` attribute to the root element.

    """
    def _fix_tag(elem):
        if elem.tag.startswith('{'+NS+'}'):
            elem.tag = elem.tag.split('}')[1]
        for e in elem:
            _fix_tag(e)
    root = tree.getroot()
    root.set('xmlns', NS)
    _fix_tag(root)

def find_by_id(root, _id, tag=None):
    """Find an element by id."""
    for e in root.iter(tag):
        if e.get('id') == _id:
            return(e)
    return None


def write_collada_tree_in_file(_filename, _tree):
    indent(_tree)
    fix_namespace(_tree)
    with open(_filename, 'w') as _file:
        _file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        tree_as_str = tostring(_tree.getroot(), 'utf-8')
        try:
            _file.write(tree_as_str)                # for python2.x
        except TypeError:
            _file.write(str(tree_as_str, 'utf-8'))  # for python3.x


def require_SubElement(root_node, tag, attrib=None, position=None):
    attrib = attrib or {}
    children = [e for e in list(root_node) if tag in e.tag]
    for c in children:
        for k, v in attrib.items():
            if c.get(k) != v:
                children.pop(c)
    if len(children)==0: #No subElement Found, return one
        elem = Element(tag, attrib)
        if position is None:
            root_node.append(elem)
        else:
            root_node.insert(position, elem)
        return elem
    else:
        return children[0]

def _add_osg_description(node, description):
    extra = require_SubElement(node, "extra", {"type":"Node"})
    tech = require_SubElement(extra, "technique", {"profile":"OpenSceneGraph"})
    descriptions = require_SubElement(tech, "Descriptions")
    SubElement(descriptions, "Description").text = description


def get_collada_default_options():
    options = {
               "stand alone": True,
              }
    return options


class ColladaDriver(arboris._visu.DrawerDriver):
    def __init__(self, filename, scale=1., options=None):
        arboris._visu.DrawerDriver.__init__(self, scale, \
                                            get_collada_default_options())
        if options is not None:
            self._options.update(options)
        self._colors = []
        self._filename = filename
        self._shapes = "" if self._options["stand alone"] \
                          else r"file:///"+SHAPES.replace("\\", "/")
        self._used_shapes = set(["frame_arrows_geometry"])
        scene = os.path.join(os.path.dirname(os.path.abspath(__file__)), \
                             'scene.dae')
        self._tree = ET.parse(scene)
        frame_arrows = find_by_id(self._tree, 'frame_arrows', QN('node'))
        instance_node = frame_arrows.find('./{'+NS+'}instance_node')
        instance_node.set('url', self._shapes + "#unitary_frame_arrows")
        # update the frame arrows scale
        scale = frame_arrows.find('./{'+NS+'}scale')
        scale.text = "{0} {0} {0}".format(self._options['frame arrows length'])

    def init(self):
        pass

    def add_ground(self, up):
        self._set_up_axis(up)
        self._set_date()
        ground = find_by_id(self._tree, 'ground', QN('node'))
        assert ground is not None
        return ground

    def _set_up_axis(self, up):
        """Add an up_axis element."""
        if np_all(up == [1., 0., 0.]):
            up_text = 'X_UP'
        elif np_all(up == [0., 1., 0.]):
            up_text = 'Y_UP'
        elif np_all(up == [0., 0., 1.]):
            up_text = 'Z_UP'
        else:
            up_text = None
            warnings.warn('the up vector is not compatible with collada')
        if up_text:
            up_element = SubElement(self._tree.find('{'+NS+'}asset'),
                    QN('up_axis'))
            up_element.text = up_text

    def _set_date(self, date=None):
        """Set the created ans modified elements"""
        if date is None:
            date = datetime.now()
        created = self._tree.find('{'+NS+'}asset/{'+NS+'}created')
        created.text = date.isoformat()
        modified = self._tree.find('{'+NS+'}asset/{'+NS+'}modified')
        modified.text = date.isoformat()


    def add_child(self, parent, child, category=None):
        def plural(noun):
            """Return the plural of a noun"""
            if noun[-1] == 's':
                return noun
            else:
                return noun + 's'

        assert category in (None, 'frame arrows', 'shape', 'link', 'inertia')
        if category is None or self._options['display '+plural(category)]:
            parent.append(child)

    def create_transform(self, pose, is_constant, name=None): #TODO: is_constant
        """Generate the node corresponding to the pose."""    # is useless
        attr = {"id":name, "name":name} if name else {}
        node = Element(QN("node"), attr)
        matrix = SubElement(node, QN("matrix"), {'sid':'matrix'})
        matrix.text = " ".join([str(round(val, 6)) for val in pose.flatten()])
        return node

    def create_line(self, start, end, color, name=None):
        vector = (array(end) - array(start))
        vector_norm = linalg.norm(vector)
        assert vector_norm > 0.
        n_vector = vector/vector_norm
        H = zaligned(n_vector)
        H[0:3, 3] = start
        node = self.create_transform(H, is_constant=True, name=name)
        scale = SubElement(node, QN('scale'))
        scale.text = "0. 0. {0}".format(vector_norm)
        node_link = SubElement(node, QN("node"))
        elem = SubElement(node_link, QN("instance_geometry"), \
                          {"url": self._shapes+"#line"})
        self._add_color(elem, color)
        _add_osg_description(node, "link")
        self._used_shapes.add('line')
        return node

    def create_inertia(self, inertia, color, name=None):
        """Generate a representation of inertia as an ellipsoid."""
        H = principalframe(inertia)
        M = transport(inertia, H)
        attr = {"name":name} if name else {}
        node = Element(QN("node"), attr)
        matrix = SubElement(node, QN("matrix"), {'sid':'matrix'})
        matrix.text = str(H.reshape(-1)).strip('[]')
        scale = SubElement(node, QN('scale'))
        f = self._options['inertia factor']
        scale.text = "{0} {1} {2}".format(M[0, 0]*f, M[1, 1]*f, M[2, 2]*f)
        elem = SubElement(node, QN("instance_geometry"), \
                          {"url": self._shapes+"#sphere_80"})
        self._add_color(elem, color)
        _add_osg_description(node, "inertia")
        self._used_shapes.add('sphere_80')
        return node

    def create_frame_arrows(self):
        return Element(QN("instance_node"), {"url": "#frame_arrows"})

    def create_box(self, half_extents, color, name=None):
        attr = {"name":name} if name else {}
        node = Element(QN("node"), attr)
        scale = SubElement(node, QN('scale'))
        scale.text = "{0} {1} {2}".format(*half_extents)
        elem = SubElement(node, QN("instance_geometry"), \
                          {"url": self._shapes+"#box"})
        self._add_color(elem, color)
        _add_osg_description(node, "shape")
        self._used_shapes.add('box')
        return node

    def create_plane(self, coeffs, color, name=None):
        H = zaligned(coeffs[0:3])
        H[0:3, 3] = coeffs[3] * coeffs[0:3]
        node = self.create_transform(H, is_constant=True, name=name)
        scale = SubElement(node, QN('scale'))
        scale.text = "{0} {1} 0.".format(*self._options["plane half extents"])
        node_shape = SubElement(node, QN("node"))
        elem = SubElement(node_shape, QN("instance_geometry"), \
                          {"url": self._shapes+"#plane"})
        self._add_color(elem, color)
        _add_osg_description(node, "shape")
        self._used_shapes.add('plane')
        return node

    def _create_ellipsoid(self, radii, color, resolution, name=None):
        assert resolution in ('20', '80', '320')
        attr = {"name":name} if name else {}
        node = Element(QN("node"), attr)
        scale = SubElement(node, QN('scale'))
        scale.text = "{0} {1} {2}".format(*radii)
        elem = SubElement(node, QN("instance_geometry"), \
                          {"url": self._shapes+"#sphere_"+resolution})
        self._add_color(elem, color)
        _add_osg_description(node, "shape")
        self._used_shapes.add('sphere_'+resolution)
        return node

    def create_ellipsoid(self, radii, color, name=None):
        return self._create_ellipsoid(radii, color, resolution='320', name=name)

    def create_point(self, color, name=None):
        radii = (self._options['point radius'],) * 3
        return self._create_ellipsoid(radii, color, resolution='20', name=name)

    def _create_cylinder(self, length, radius, color, resolution, name=None):
        assert resolution in ('8', '32')
        attr = {"name":name} if name else {}
        node = Element(QN("node"), attr)
        scale = SubElement(node, QN('scale'))
        scale.text = "{0} {0} {1}".format(radius, length)
        elem = SubElement(node, QN("instance_geometry"), \
                          {"url": self._shapes+"#cylinder_"+resolution})
        self._add_color(elem, color)
        _add_osg_description(node, "shape")
        self._used_shapes.add('cylinder_'+resolution)
        return node

    def create_cylinder(self, length, radius, color, name=None):
        return self._create_cylinder(length, radius, color, '32', name=name)

    def _color_id(self, color):
        return "color{0}".format(self._colors.index(color))

    def _add_color(self, parent, color):
        if not color in self._colors:
            self._colors.append(color)
        parent.append(XML("""
        <bind_material xmlns="{NS}">
            <technique_common>
                <instance_material symbol="material" target="#{0}" />
            </technique_common>
        </bind_material>
        """.format(self._color_id(color), NS=NS)))

    def _write_colors(self):
        lib_materials = require_SubElement(self._tree.getroot(), \
                                           'library_materials')
        lib_effects = require_SubElement(self._tree.getroot(), \
                                         'library_effects')
        for c in self._colors:
            color_id = self._color_id(c)
            lib_materials.append(XML("""
            <material id="{0}" xmlns="{NS}">
                <instance_effect url="#{0}_fx"/>
            </material>
            """.format(color_id, NS=NS)))

            lib_effects.append(XML("""
            <effect id="{0}_fx" xmlns="{NS}">
                <profile_COMMON>
                    <technique sid="blender">
                        <phong>
                            <diffuse>
                                <color>{1} {2} {3} 1</color>
                            </diffuse>
                        </phong>
                    </technique>
                </profile_COMMON>
            </effect>
            """.format(color_id, *c, NS=NS)))


    def finish(self):
        # write to file
        self._write_colors()
        src_root = ET.parse(SHAPES).getroot()
        to_root =  self._tree.getroot()
        if self._options["stand alone"]:
            for lib_name in ["library_materials",
                             "library_effects",
                             "library_nodes"]:
                src_lib = src_root.find(QN(lib_name))
                to_lib = require_SubElement(to_root, QN(lib_name))
                for elem in list(src_lib):
                    to_lib.append(elem)
            src_lib = src_root.find(QN("library_geometries"))
            to_lib = require_SubElement(to_root, QN("library_geometries"))
            for elem in self._used_shapes:
                to_lib.append(find_by_id(src_lib, elem))
        for lib_name in ["library_materials", "library_effects",
                         "library_nodes", "library_geometries"]:
            to_lib = require_SubElement(to_root, QN(lib_name))
            if len(list(to_lib))==0:
                to_root.remove(to_lib)
        write_collada_tree_in_file(self._filename, self._tree)



def depth_copy_node(src_root, to_root, node_url): #TODO: can be improved
    node_file, node_id = node_url.split("#")
    if node_file:
        depth_copy_node(ET.parse(node_file).getroot(), to_root, node_id)
    else:
        root_node = find_by_id(src_root, node_id)
        to_root_lib_node = require_SubElement(to_root, QN("library_nodes"))
        to_root_lib_node.append(root_node)
        subnode = root_node.findall(QN("instance_node"))
        for sn in subnode:
            sn_url = sn.get("url")
            depth_copy_node(src_root, to_root, sn_url)
            sn_url_id = sn_url.split("#")[1]
            sn.set("url", "#"+sn_url_id)
        subgeo = root_node.findall(QN("instance_geometry"))
        for sg in subgeo:
            sg_url = sg.get("url")
            sg_url_file, sg_url_id = sg_url.split("#")
            if sg_url_file:
                geom = find_by_id(ET.parse(sg_url_file).getroot(), sg_url_id)
            else:
                geom = find_by_id(src_root, sg_url_id)
            to_root_lib_geo = require_SubElement(to_root,
                                                 QN("library_geometries"),
                                                 position=1)
            to_root_lib_geo.append(geom)
            sg.set("url", "#"+sg_url_id)
        submat = root_node.getiterator(QN("instance_material"))
        for sm in submat:
            sm_url = sm.get("target")
            sm_url_file, sm_url_id = sm_url.split("#")
            if sm_url_file:
                material = find_by_id(ET.parse(sm_url_file).getroot(),
                                      sm_url_id)
            else:
                material = find_by_id(src_root, sm_url_id)
            to_root_lib_mat = require_SubElement(to_root,
                                                 QN("library_materials"),
                                                 position=1)
            if material.get("id") not in \
                           [e.get("id") for e in to_root_lib_mat.getchildren()]:
                to_root_lib_mat.append(material)
            sm.set("target", "#"+sm_url_id)
            subfx = material.getiterator(QN("instance_effect"))
            for sf in subfx:
                sf_url = sf.get("url")
                sf_url_file, sf_url_id = sf_url.split("#")
                if sf_url_file:
                    effect = find_by_id(ET.parse(sf_url_file).getroot(),
                                        sf_url_id)
                else:
                    effect = find_by_id(src_root, sf_url_id)
                to_root_lib_fx = require_SubElement(to_root,
                                                    QN("library_effects"),
                                                    position=1)
                if effect.get("id") not in \
                            [e.get("id") for e in to_root_lib_fx.getchildren()]:
                    to_root_lib_fx.append(effect)
                sf.set("url", "#"+sf_url_id)


def use_custom_shapes(dae_filename, mapping, stand_alone=False):
    tree = ET.parse(dae_filename)
    for node in tree.getiterator(QN("node")):
        node_id = node.get('id')
        if node_id in mapping:
            custom_shape = mapping[node_id]
            custom_node_file, custom_node_id = custom_shape.split("#")
            custom_node = find_by_id(ET.parse(custom_node_file).getroot(), custom_node_id)
            custom_node_tag = 'node' if 'node' in custom_node.tag else 'geometry'
            if stand_alone:
                if custom_node_tag is 'node':
                    depth_copy_node(ET.parse(custom_node_file).getroot(),
                                    tree.getroot(),
                                    "#"+custom_node_id)
                else:
                    geo_to_copy = find_by_id(ET.parse(custom_node_file).getroot(), custom_node_id)
                    to_lib_geo = require_SubElement(tree.getroot(),
                                                    QN("library_geometries"),
                                                    position=1)
                    to_lib_geo.append(geo_to_copy)
                custom_shape = "#"+custom_node_id
            shape_node = SubElement(node, QN("node"))
            SubElement(shape_node, QN("instance_"+custom_node_tag),
                       {"url": custom_shape})
            _add_osg_description(shape_node, "shape")

    write_collada_tree_in_file(dae_filename, tree)



def write_collada_scene(world, dae_filename, scale=1, options=None, flat=False, color_generator=None):
    """Write a visual description of the scene in a collada file.

    :param world: the world to convert
    :type world: :class:`arboris.core.World` instance
    :param dae_filename: path of the output collada scene file
    :type dae_filename: str
    :param scale: the scale of the world
    :type scale: float
    :param options: the options to set the world building
    :type options: dict
    :param flat: if True, each body is a child of the ground. Otherwise
                 the hierarchy is copied from arboris
    :type flat: bool

    """
    assert isinstance(world, World)
    if flat:
        name_all_elements(world.getbodies(), True)
    else:
        nonflatlist = [j.frames[1] for j in world.getjoints()]
        name_all_elements(nonflatlist, True)
    name_all_elements(world.itermovingsubframes(), True)
    world.update_geometric()
    drawer = arboris._visu.Drawer(ColladaDriver(dae_filename, scale, options), flat, color_generator)
    world.parse(drawer)
    drawer.finish()



def _write_col_anim(collada_animation, collada_scene, timeline, transforms):
    
    def create_anim_src_elem(parent_element, name, src, nameOfSrc, typeOfSrc, paramName, paramType, count, stride=None):
        idName = name +"-"+nameOfSrc
        src_elem = SubElement(parent_element, "source", {"id":idName})
        
        idName += "-array"
        typeArrayElem = SubElement(src_elem, typeOfSrc+"_array", {"id":idName, "count":str(count)})
        typeArrayElem.text = src
        
        techElem = SubElement(src_elem, "technique_common")
        accessElem = SubElement(techElem, "accessor", {"source":"#"+idName, "count":str(count)})
        if stride:
            accessElem.set("stride", str(stride))
        paramElem  = SubElement(accessElem, "param", {"name": paramName, "type":paramType})


    def create_anim_elem(anim_lib, name, timeline, val):
        idName=name+"-matrix"
        animElem = SubElement(anim_lib, "animation", {"id":idName})
        
        count = len(timeline)
        inputSrc  = " ".join(map(str, timeline))
        interSrc  = " ".join(["STEP"]*count)
        outputSrc = " ".join(map(str, val.flatten()))
        create_anim_src_elem(animElem, idName, inputSrc,  "input",         "float", "TIME", "float", count)
        create_anim_src_elem(animElem, idName, interSrc,  "interpolation", "Name", "INTERPOLATION", "name", count)
        create_anim_src_elem(animElem, idName, outputSrc, "output",        'float', "TRANSFORM", "float4x4", count, 16)

        samplerElem = SubElement(animElem, "sampler", {"id":idName+"-sampler"})
        for semantic in ["input", "output", "interpolation"]:
            SubElement(samplerElem, "input", {"semantic":semantic.upper(), "source":"#"+idName+"-"+semantic})
        channelElem = SubElement(animElem, "channel", {"source":"#"+idName+"-sampler", "target":name+"/"+"matrix"})


    tree = ET.parse(collada_scene)
    anim_lib = Element("library_animations")
    children = list(tree.getroot())
    tree.getroot().insert(children.index(tree.find(QN("scene"))), anim_lib)

    for name, val in transforms.items():
        create_anim_elem(anim_lib, name, timeline, val)

    write_collada_tree_in_file(collada_animation, tree)



def write_collada_animation(collada_animation, collada_scene, sim_file, 
                            hdf5_group="/", file_type=None):
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
            timeline = array(sim[hdf5_group]["timeline"])
            transforms = {}
            for k,v in sim[hdf5_group]["transforms"].items():
                transforms[k] = array(v)
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

    _write_col_anim(collada_animation, collada_scene, timeline, transforms)


def view(collada_file, hdf5_file=None, hdf5_group="/", daenimpath=None):
    """Display a collada file, generating the animation if necessary.

    Usage::

        view(collada_file)
        view(collada_scene_file, hdf5_file, [hdf5_group])

    If only the `collada_file` is given, it is displayed.
    If both `collada_file` and `hdf5_file` are given, they are combined into
    a collada animation file, which is displayed.

    This function is a Wrapper around the ``h5toanim`` and ``daenim`` external
    commands. Tey should both be installed for the function to work.

    """
    if daenimpath is None:
        if os.name == 'posix':
            daenimpath = 'daenim'
        elif os.name == 'nt':
            daenimpath = 'C:/Program Files/daenim/daenim.exe'
        else:
            print("May not work on this os. " + \
                  "h5toanim path should be specified manually")
            daenimpath = 'daenim'

    if hdf5_file is None:
        subprocess.check_call((daenimpath, collada_file))
    else:
        anim_file = tempfile.mkstemp(suffix='anim.dae', text=True)[1]
        write_collada_animation(anim_file, collada_file, hdf5_file, hdf5_group)
        subprocess.check_call((daenimpath, anim_file))
        os.remove(anim_file)


