
import arboris._visu
from arboris.massmatrix import principalframe, transport
from arboris.homogeneousmatrix import zaligned, pdot
import numpy as np

import visual


def get_vpython_default_options():
    return {
        "title": "Arboris-python simulation",
        "x": 100,
        "y": 100,
        "width": 640,
        "height": 480,
        "background": (.2,.2,.4),
        "center": (0,0,0),
        "forward": (-1,-1,-1),
        "range": 1.5,
        "autocenter":False,
        "autoscale": False,
    }


def set_frame_pose_from_homogeneousmatrix(f, H):
    f.pos  = H[0:3,3]
    f.axis = H[0:3,0]
    f.up   = H[0:3,1]


class VPythonDriver(arboris._visu.DrawerDriver):
    
    def __init__(self, scale=1., options=None):
        arboris._visu.DrawerDriver.__init__(self, scale=scale, options=get_vpython_default_options())
        if options is not None:
            self._options.update(options)

        self.scene = visual.display(**self._options)

        self.transform = {}
        self.inertia = []
        self.frame_arrows = []
        self.lines = []
        self.shapes = []

    def add_ground(self, up):
        self.scene.select()
        self.scene.up = up
        self.transform['ground'] = visual.frame()

    def init(self):
        self.scene.select()

    def finish(self):
        pass

    def create_transform(self, pose, is_constant, name=None):
        transform = visual.frame(pos=pose[0:3,3], axis=pose[0:3,0], up=pose[0:3,1])
        if name is not None:
            self.transform[name] = transform
        return transform

    def create_inertia(self, inertia, color, name=None):
        H = principalframe(inertia)
        M = transport(inertia, H)
        inert = visual.frame(pos=H[0:3,3], axis=H[0:3,0], up=H[0:3,1])
        f = 2*self._options['inertia factor']
        full_lengths = (M[0, 0]*f, M[1, 1]*f, M[2, 2]*f)
        inert_shape = visual.ellipsoid(frame=inert, pos=(0,0,0), size=full_lengths, color=color, visible=False)
        self.inertia.append(inert_shape)
        return inert

    def create_frame_arrows(self):
        frame_arrows = visual.frame()
        l = self._options['frame arrows length']
        visual.curve(frame=frame_arrows, pos=[(0,0,0), (l,0,0)], color=(1,0,0))
        visual.curve(frame=frame_arrows, pos=[(0,0,0), (0,l,0)], color=(0,1,0))
        visual.curve(frame=frame_arrows, pos=[(0,0,0), (0,0,l)], color=(0,0,1))
        
        self.frame_arrows.append(frame_arrows)
        return frame_arrows

    def create_line(self, start, end, color, name=None):
        line = visual.curve(pos=[start, end], color=color)
        self.lines.append(line)
        return line

    def create_ellipsoid(self, radii, color, name=None):
        full_lengths = [v*2. for v in radii]
        shape = visual.ellipsoid(pos=(0,0,0), size=full_lengths, color=color)
        self.shapes.append(shape)
        return shape

    def create_box(self, half_extents, color, name=None):
        full_extents = [v*2. for v in half_extents]
        shape = visual.box(pos=(0,0,0), size=full_extents, color=color)
        self.shapes.append(shape)
        return shape

    def create_plane(self, coeffs, color, name=None):
        x,y = self._options['plane half extents']
        H = zaligned(coeffs[0:3])
        H[0:3, 3] = coeffs[3] * coeffs[0:3]
        plane = visual.faces()
        for pt in [(-x,-y,0), (x,-y,0), (-x,y,0),(-x, y,0), (x,-y,0), ( x,y,0)]:
            pt0 = pdot(H, pt)
            plane.append(pos=pt0, normal=coeffs[0:3], color=color)
        self.shapes.append(plane)
        return plane

    def create_cylinder(self, length, radius, color, name=None):
        shape = visual.cylinder(pos=(0,0,-length/2.), axis=(0,0,length), radius=radius, color=color)
        self.shapes.append(shape)
        return shape

    def add_child(self, parent, child, category=None):
        child.frame = parent

    def update_transform(self, transform_name, pose):
        transform = self.transform[transform_name]
        set_frame_pose_from_homogeneousmatrix(transform, pose)

    def check_keyboard(self):
        unused_keys = []
        objList = []
        for i in np.arange(self.scene.kb.keys):
            k = self.scene.kb.getkey()
            if   k == "f": objList.extend(self.frame_arrows)
            elif k == "s": objList.extend(self.shapes)
            elif k == "i": objList.extend(self.inertia)
            elif k == "l": objList.extend(self.lines)
            else: unused_keys.append(k)
        for obj in objList:
            obj.visible = not obj.visible
        return unused_keys






################################################################################
#
# For animation from Collada file
#
################################################################################
import xml.etree.ElementTree as ET
from arboris.visu_collada import QN, find_by_id
import time

def get_collada_options(root):
    options = {}
    
    ## check up axis
    asset = root.find(QN("asset"))
    if asset is not None:
        upnode = asset.find(QN("up_axis"))
        if upnode is not None:
            if upnode.text == "X_UP":
                options["up"] = [1,0,0]
            elif upnode.text == "Y_UP":
                options["up"] = [0,1,0]
            elif upnode.text == "Z_UP":
                options["up"] = [0,0,1]
    else:
        options["up"] = [0,1,0]

    ##check frame_arrows length
    fa = find_by_id(root, "frame_arrows")
    options["frame arrows length"] = float(fa.find(QN("scale")).text.split()[0])

    return options


def get_collada_colors(root):
    colors = {}
    col_lib = root.find(QN("library_effects"))
    for c in col_lib.getchildren():
        col_name = c.get("id")[:-3]
        col = [v for v in c.iter(QN("color"))][0]
        colors[col_name] = [float(v) for v in col.text.split()[0:3]]
    return colors


def parse_collada(driver, colors, parent_collada_node, parent_frame, current_scale=(1,1,1)):
    """
    """
    def get_color(node):
        col_name = [v for v in node.iter(QN("instance_material"))][0].get("target")
        return colors[col_name[1:]]

    children_node = parent_collada_node.getchildren()
    for c in children_node:

        child_frame = None

        if c.tag == QN("scale"):
            current_scale = [float(v) for v in c.text.split()]

        elif c.tag == QN("node"):
            mat = c.find(QN("matrix"))
            if mat is not None:
                node_name = c.get("id")
                pose = np.array([float(v) for v in mat.text.split()]).reshape((4,4))
                child_frame = driver.create_transform(pose, False, node_name)
            else:
                child_frame = visual.frame()
            parse_collada(driver, colors, c, child_frame, current_scale)

        elif c.tag == QN("instance_node"):
            if c.get("url") == "#frame_arrows":
                child_frame = driver.create_frame_arrows()

        elif c.tag == QN("instance_geometry"):
            if c.get("url") == "#line":
                end = current_scale
                child_frame = driver.create_line((0,0,0), end, get_color(c))

            elif c.get("url") == "#box":
                half_extents = current_scale
                color = [v for v in c.iter(QN("instance_material"))][0]
                child_frame = driver.create_box(half_extents, get_color(c))

            elif c.get("url") in ["#cylinder_8", "#cylinder_32"]:
                radius,radius,length = current_scale
                color = [v for v in c.iter(QN("instance_material"))][0]
                child_frame = driver.create_cylinder(length, radius, get_color(c))

            elif c.get("url") in ["#sphere_80", "#sphere_320"]:
                full_lengths = np.array(current_scale)*2.
                desc = [v for v in parent_collada_node.iter(QN("Description"))]
                if len(desc) and desc[0].text == "inertia":
                    child_frame = visual.ellipsoid(pos=(0,0,0), size=full_lengths, color=(1,1,1))
                    driver.inertia.append(child_frame)
                else:
                    child_frame = visual.ellipsoid(pos=(0,0,0), size=full_lengths, color=get_color(c))
                    driver.shapes.append(child_frame)

            elif c.get("url") == "#plane":
                child_frame = driver.create_plane(np.array((0,0,1,0)), get_color(c))


        if child_frame is not None:
            driver.add_child(parent_frame, child_frame)



def get_animation_data(root):
    anim_lib = root.find(QN("library_animations"))
    if anim_lib is not None:
        anim_rec = {}
        timeline = []
        for c in anim_lib.getchildren():
            if c.tag == QN("animation"):
                name = c.get("id")
                assert name[-7:] == "-matrix"
                c_in  = find_by_id(c, name+"-input-array")
                c_out = find_by_id(c, name+"-output-array")
                count = int(c_in.get("count"))
                timeline            = np.array([float(v) for v in c_in.text.split()]) #we suppose that all animations have the same input vector (the timeline)
                anim_rec[name[:-7]] = np.array([float(v) for v in c_out.text.split()]).reshape((count, 4, 4))
        return timeline, anim_rec
    else:
        return None, None


def view_collada_file(animation_file, user_options=None):
    """
    """
    
    # Get all collada information
    tree = ET.parse(animation_file)
    root = tree.getroot()
    collada_options = get_collada_options(root)
    collada_colors  = get_collada_colors(root)
    ground_node = find_by_id(root, "ground")
    
    # Get collada animation (if any)
    timeline, animation_data = get_animation_data(root)

    # draw world from collada with VPython driver
    if user_options is not None:
        collada_options.update(user_options)
    drv = VPythonDriver(options = collada_options)
    drv.add_ground(collada_options["up"])
    parse_collada(drv, collada_colors, ground_node, drv.transform['ground'])
    
    
    # Show or animate collada file
    dt = 0.01
    if timeline is not None:
        max_idx = len(timeline)
        idx = 0
        pause = True
        while 1:
            other_keys = drv.check_keyboard()
            if ' ' in other_keys:
                pause = not pause

            for transform_name in animation_data:
                H = animation_data[transform_name][idx]
                drv.update_transform(transform_name, H)

            if not pause:
                idx = (idx + 1)%max_idx
                if idx:
                    dt = timeline[idx] - timeline[idx-1]
                print("time:", timeline[idx])

            time.sleep(dt)
    else:
        print("No animation found.")
        while 1:
            drv.check_keyboard()
            time.sleep(dt)


def view(world, scale=1., options=None, color_generator=None):
    """
    """
    driver = VPythonDriver(scale, options)
    drawer = arboris._visu.Drawer(driver, True, color_generator)
    world.update_geometric()
    world.parse(drawer)
    drawer.finish()
    while 1:
        driver.check_keyboard()
        time.sleep(.01)

