
from arboris.core import Observer, name_all_elements
import arboris._visu
from arboris.massmatrix import principalframe, transport

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
    }



class VPythonDriver(arboris._visu.DrawerDriver):
    
    def __init__(self, scale=1., options=None):
        arboris._visu.DrawerDriver.__init__(self, scale=scale, options=get_vpython_default_options())
        if options is not None:
            self._options.update(options)

        self.scene = visual.display(**self._options)
        self.scene.autocenter = False
        self.scene.autoscale  = False

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
        plane = visual.faces()
        #TODO append to add vertex : f.append(pos=(x,y,z), normal=(nx,ny,nz), color=(r,g,b))
        plane.append(pos=(-x, -y, 0), normal=(0,0,1), color=color)
        plane.append(pos=( x, -y, 0), normal=(0,0,1), color=color)
        plane.append(pos=(-x,  y, 0), normal=(0,0,1), color=color)
        plane.append(pos=(-x,  y, 0), normal=(0,0,1), color=color)
        plane.append(pos=( x, -y, 0), normal=(0,0,1), color=color)
        plane.append(pos=( x,  y, 0), normal=(0,0,1), color=color)
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
        transform.axis = pose[0:3,0]
        transform.up   = pose[0:3,1]
        transform.pos  = pose[0:3,3]




