# coding=utf-8


from numpy import array, linalg

from arboris.massmatrix import principalframe, transport
from arboris.homogeneousmatrix import zaligned

from arboris.core import World, name_all_elements
from arboris.visu.world_drawer import Drawer, ColorGenerator

import collada
from collada.common import E


import subprocess
import os
import tempfile
import warnings



def write_collada_scene(world, dae_filename, scale=1, options=None, flat=False, color_generator=None):
    """Write a visual description of the scene in a collada file.

    :param world: the world to convert
    :type world: :class:`arboris.core.World` instance
    :param dae_filename: path of the output collada scene file
    :type dae_filename: str
    :param scale: the scale of the world
    :type scale: float
    :param options: the options to set the world drawing
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

    if color_generator is None:
        color_generator = ColorGenerator( nb_colors=len(world.getbodies()) )

    world.update_geometric()

    drawer = Drawer(DaenimColladaDriver(dae_filename, scale, options), flat, color_generator)
    world.parse(drawer)
    drawer.finish()


from arboris.visu.dae_writer import write_collada_animation # save animation is done the same way



from arboris.visu.dae_writer import pydaenimColladaDriver
from arboris.visu.dae_writer import _save_all_descendant_geometryNode, _get_void_node

class DaenimColladaDriver(pydaenimColladaDriver):

    def _add_osg_description(self, node, description):
        """
        """
        extra = E.extra(
                    E.technique(
                        E.Descriptions( E.Description(description) ),
                    profile="OpenSceneGraph",
                    ),
                type="Node",
                )
        extra_node = collada.scene.ExtraNode(extra)
        node.children.append(extra_node)

    def create_ellipsoid(self, radii, color, name=None):
        node = pydaenimColladaDriver.create_ellipsoid(self, radii, color, name)
        self._add_osg_description(node, "shape")
        return node

    def create_box(self, half_extents, color, name=None):
        node = pydaenimColladaDriver.create_box(self, half_extents, color, name)
        self._add_osg_description(node, "shape")
        return node

    def create_cylinder(self, length, radius, color, name=None):
        node = pydaenimColladaDriver.create_cylinder(self, length, radius, color, name)
        self._add_osg_description(node, "shape")
        return node

    def create_plane(self, coeffs, color, name=None):
        node = pydaenimColladaDriver.create_plane(self, coeffs, color, name)
        self._add_osg_description(node, "shape")
        return node

    def create_frame_arrows(self):
        if self.dae.nodes.get("frame_arrows") is None:
            frame_arrows_node = self.shapes_dae.nodes.get("frame_arrows")
            self.dae.nodes.append(frame_arrows_node)

            s = self._options['frame arrows length']
            frame_arrows_node.transforms.append(collada.scene.ScaleTransform(s,s,s))
            _save_all_descendant_geometryNode(frame_arrows_node, self.dae)

        frame_arrows_node = self.dae.nodes.get("frame_arrows")

        node = collada.scene.NodeNode(frame_arrows_node)

        return node

    def create_line(self, start, end, color, name=""):
        if self.dae.geometries.get("line_geometry-mesh") is None:
            self.dae.geometries.append(self.shapes_dae.geometries.get("line_geometry-mesh"))
        line_geom = self.dae.geometries.get("line_geometry-mesh")

        node = _get_void_node()
        vector = (array(end) - array(start))
        vector_norm = linalg.norm(vector)
        assert vector_norm > 0.
        n_vector = vector/vector_norm
        H = zaligned(n_vector)
        H[0:3, 3] = start

        node = self._add_new_geometry(name, color, line_geom)
        node.transforms.append(collada.scene.ScaleTransform(vector_norm, vector_norm, vector_norm))
        node.transforms.append(collada.scene.MatrixTransform(H.flatten()))

        self._add_osg_description(node, "link")

        return node

##    def create_inertia(self, inertia, color, name=""):
##        """Generate a representation of inertia as an ellipsoid."""
##        pydaenimColladaDriver.create_inertia(self, inertia, color)
##        H_body_com  = principalframe(inertia)
##        Mcom        = transport(inertia, H_body_com)
##        mass        = Mcom[5,5]
##        m_o_inertia = Mcom[[0,1,2], [0,1,2]] * self._options['inertia factor']
##
##        node = _get_void_node()
##        node.transforms.append( collada.scene.MatrixTransform(H_body_com.flatten()) )
##
##        inertia_node = self.create_ellipsoid(m_o_inertia, color, name+".inertia")
##        node.children.append(inertia_node)
##
##        for c in inertia_node.children:
##            if isinstance(c, collada.scene.ExtraNode):
##                inertia_node.children.remove(c)
##
##        self._add_osg_description(inertia_node, "inertia")
##
##        return node




from arboris.observers import SocketCom
import subprocess, shlex
from time import sleep

class DaenimCom(SocketCom):
    """
    """
    def __init__(self, daefile=None, daenim_path=None, host="127.0.0.1", port=5000, timeout=3, \
                 options = "", flat=False, name=None):
        """
        """
        SocketCom.__init__(self, host, port, timeout, name)

        self.daefile = daefile

        if daenim_path is None:
            daenim_path = get_daenim_path()

        self.app_call = \
              [daenim_path, daefile, "-socket", self.host, str(self.port)] + \
               shlex.split(options)
        self.flat = flat
        self.world = None

    def init(self, world, timeline):
        """
        """
        if self.daefile is None:
            tmp_daefile = tempfile.mkstemp(suffix='scene.dae', text=True)[1]
            write_collada_scene(world, tmp_daefile, flat=self.flat)
            self.app_call[1] = tmp_daefile

        try:
            subprocess.Popen(self.app_call)
            SocketCom.init(self, world, timeline)
        except OSError:
            warnings.warn("Cannot find program "+self.app_call[0]+". Please check where the arboris viewer (daenim) is installed on your computer.")

        self.world = world
        sleep(.1)
        if self.flat:
            name_all_elements(self.world.getbodies(), True)
        else:
            nonflatlist = [j.frames[1] for j in self.world.getjoints()]
            name_all_elements(nonflatlist, True)
        name_all_elements(self.world.itermovingsubframes(), True)

    def update(self, dt):
        """
        """
        if self.flat:
            msg = b""
            for b in self.world.getbodies():
                H = b.pose
                msg += self.packing_transform(b.name, H)
        else:
            msg = ""
            for j in self.world.getjoints():
                H = j.pose
                msg += self.packing_transform(j.frames[1].name, H)

        for f in self.world.itermovingsubframes():
            H = f.pose if self.flat else f.bpose
            msg += self.packing_transform(f.name, H)

        try:
            self.conn.send(msg)
        except:
            print("DaenimCom not connected")


    def packing_transform(self, _name, _H):
        res = " ".join([_name] + map(str, _H[0:3, :].reshape(12)))+"\n"
        return res



def get_daenim_path():
    if os.name == 'posix':
        daenim_path = 'daenim'
    elif os.name == 'nt':
        daenim_path = 'C:/Program Files (x86)/Daenim/daenim.exe'
        if not os.path.exists(daenim_path):
            daenim_path = 'C:/Program Files/Daenim/daenim.exe'
    else:
        print("May not work on this os. " + \
                  "daenim path should be specified manually.")
        daenim_path=None
    return daenim_path



def view(scene, hdf5_file=None, hdf5_group="/", daenim_path=None):
    """Display a collada file, generating the animation if necessary.

    Usage::

        view(world)
        view(collada_scene_file)
        view(collada_scene_file, hdf5_file, [hdf5_group])

    If only the `scene` is given (World or collada file), it is displayed.
    If both `scene` and `hdf5_file` are given, they are combined into
    a collada animation file, which is displayed.

    This function is a Wrapper around the ``daenim`` external commands.
    It should be installed for the function to work.

    """
    if daenim_path is None:
        daenim_path = get_daenim_path()


    if isinstance(scene, World):
        scene_file = tempfile.mkstemp(suffix='anim.dae', text=True)[1]
        write_collada_scene(scene, scene_file)
        subprocess.check_call((daenim_path, scene_file))
        os.remove(scene_file)
    else:
        if hdf5_file is None:
            subprocess.check_call((daenim_path, scene))
        else:
            anim_file = tempfile.mkstemp(suffix='anim.dae', text=True)[1]
            write_collada_animation(anim_file, scene, hdf5_file, hdf5_group)
            subprocess.check_call((daenim_path, anim_file))
            os.remove(anim_file)


