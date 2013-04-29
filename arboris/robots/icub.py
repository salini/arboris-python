# coding = utf-8
# author Joseph Salini
# date 11th march 2010

from arboris.core import Body, SubFrame
from arboris.massmatrix import transport, box, cylinder, sphere
from arboris.shapes import Box, Cylinder, Sphere, Point
from arboris.homogeneousmatrix import transl, inv, rotx, roty, rotz
from numpy import pi, dot, array, cos, sin, eye, zeros
from arboris.joints import FreeJoint, RzJoint

def T_DH(d=0., t=0., r=0., a=0.):
    """
    d = d
    t = theta
    r = r
    a = alpha
    Homogeneous matrix with the DH parameters
    """
    return array([[cos(t), -sin(t)*cos(a), sin(t)*sin(a) , r*cos(t)],
                  [sin(t), cos(t)*cos(a) , -cos(t)*sin(a), r*sin(t)],
                  [0     , sin(a)        , cos(a)        , d       ],
                  [0     , 0             , 0             , 1.      ]])


def get_joint_limits(in_radian=True):
    """
    give information about joint limits
    give {'joint name': [lower_lim, upper_lim]}
    """
    lims = {'torso_pitch'     : (-22, 84),
            'torso_roll'      : (-39, 39),
            'torso_yaw'       : (-59, 59),
            'head_pitch'      : (-40, 30),
            'head_roll'       : (-70, 60),
            'head_yaw'        : (-55, 55),
            'l_shoulder_pitch': (-95, 5),
            'l_shoulder_roll' : (0, 160.8),
            'l_shoulder_yaw'  : (-37, 100),
            'l_elbow_pitch'   : (5.5, 106),
            'l_elbow_yaw'     : (-50, 50),
            'l_wrist_roll'    : (-65, 10),
            'l_wrist_pitch'   : (-25, 25),
            'r_shoulder_pitch': (-95, 5),
            'r_shoulder_roll' : (0, 160.8),
            'r_shoulder_yaw'  : (-37, 100),
            'r_elbow_pitch'   : (5.5, 106),
            'r_elbow_yaw'     : (-50, 50),
            'r_wrist_roll'    : (-65, 10),
            'r_wrist_pitch'   : (-25, 25),
            'l_hip_pitch'     : (-44, 132),
            'l_hip_roll'      : (-119, 17),
            'l_hip_yaw'       : (-79, 79),
            'l_knee'          : (-125, 23),
            'l_ankle_pitch'   : (-42, 21),
            'l_ankle_roll'    : (-24, 24),
            'r_hip_pitch'     : (-44, 132),
            'r_hip_roll'      : (-119, 17),
            'r_hip_yaw'       : (-79, 79),
            'r_knee'          : (-125, 23),
            'r_ankle_pitch'   : (-42, 21),
            'r_ankle_roll'    : (-24, 24),
            }
    if in_radian:
        for k, val in lims.items():
            lims[k] = (val[0]*pi/180, val[1]*pi/180)
    return lims

def get_torque_limits():
    """
    give information about torque limits (N.m)
    WARNING: these data has not been verified
    """
    return {'torso_pitch'     : 36,
            'torso_roll'      : 80,
            'torso_yaw'       : 80,
            'head_pitch'      : 20,
            'head_roll'       : 20,
            'head_yaw'        : 20,
            'l_shoulder_pitch': 84,
            'l_shoulder_roll' : 84,
            'l_shoulder_yaw'  : 34,
            'l_elbow_pitch'   : 20,
            'l_elbow_yaw'     : .45,
            'l_wrist_roll'    : .65,
            'l_wrist_pitch'   : .65,
            'r_shoulder_pitch': 84,
            'r_shoulder_roll' : 84,
            'r_shoulder_yaw'  : 34,
            'r_elbow_pitch'   : 20,
            'r_elbow_yaw'     : .45,
            'r_wrist_roll'    : .65,
            'r_wrist_pitch'   : .65,
            'l_hip_pitch'     : 84,
            'l_hip_roll'      : 84,
            'l_hip_yaw'       : 40,
            'l_knee'          : 30,
            'l_ankle_pitch'   : 24,
            'l_ankle_roll'    : 11,
            'r_hip_pitch'     : 84,
            'r_hip_roll'      : 84,
            'r_hip_yaw'       : 40,
            'r_knee'          : 30,
            'r_ankle_pitch'   : 24,
            'r_ankle_roll'    : 11,
            }

def get_joints_data():
    """ This dictionnary describes the joint of the robot
    Each joint has 3 arguments
    name  : [parent_body, H_parentbody_jointframe, child_body]

    parent_body: the name parent body (a string)

    H_parentbody_jointframe: give the transformation matrix from the
                             parent_body frame to the joint frame. The joint is
                             always a RzJoint, a rotation around z axis.

    chid_body: the child body frame (a string) which is linked with
    the joint to the parent body.
    """
    return {
    'torso_pitch': ['waist'     , dot(rotz(pi/2), roty(-pi/2)), 'lap_belt_1'],
    'torso_roll':  ['lap_belt_1', T_DH(0, 0, .032, pi/2)      , 'lap_belt_2'],
    'torso_yaw':   ['lap_belt_2', T_DH(0, -pi/2, 0, pi/2)     , 'chest'],

    'head_pitch': ['chest' , T_DH(-.1933, -pi/2, .00231, -pi/2), 'neck_1'],
    'head_roll':  ['neck_1', T_DH(0, pi/2, .033, pi/2)         , 'neck_2'],
    'head_yaw':   ['neck_2', T_DH(.001, -pi/2, 0, -pi/2)       , 'head'],

    'l_shoulder_pitch': ['chest'       , T_DH(-.1433, 105*pi/180, .0233647,-pi/2), 'l_shoulder_1'],
    'l_shoulder_roll':  ['l_shoulder_1', T_DH(.10774, pi/2, 0, -pi/2)            , 'l_shoulder_2'],
    'l_shoulder_yaw':   ['l_shoulder_2', T_DH(0, -pi/2, 0, pi/2)                 , 'l_arm'],
    'l_elbow_pitch':    ['l_arm'       , T_DH(.15228, 75*pi/180, 0, -pi/2)       , 'l_elbow_1'],
    'l_elbow_yaw':      ['l_elbow_1'   , T_DH(0, 0, -.015, pi/2)                 , 'l_forearm'],
    'l_wrist_roll':     ['l_forearm'   , T_DH(.1373, -pi/2, 0, pi/2)             , 'l_wrist_1'],
    'l_wrist_pitch':    ['l_wrist_1'   , T_DH(0, pi/2, 0, pi/2)                  , 'l_hand'],

    'r_shoulder_pitch': ['chest'       , T_DH(-.1433,-105*pi/180,-.0233647, pi/2), 'r_shoulder_1'],
    'r_shoulder_roll':  ['r_shoulder_1', T_DH(-.10774, -pi/2, 0, pi/2)           , 'r_shoulder_2'],
    'r_shoulder_yaw':   ['r_shoulder_2', T_DH(0, -pi/2, 0, -pi/2)                , 'r_arm'],
    'r_elbow_pitch':    ['r_arm'       , T_DH(-.15228, -105*pi/180, 0, -pi/2)    , 'r_elbow_1'],
    'r_elbow_yaw':      ['r_elbow_1'   , T_DH(0, 0, .015, pi/2)                  , 'r_forearm'],
    'r_wrist_roll':     ['r_forearm'   , T_DH(-.1373, -pi/2, 0, pi/2)            , 'r_wrist_1'],
    'r_wrist_pitch':    ['r_wrist_1'   , T_DH(0, pi/2, 0, pi/2)                  , 'r_hand'],

    'l_hip_pitch':   ['waist'    , dot(transl(0,-.0681, -.1199), rotx(-pi/2)), 'l_hip_1'],
    'l_hip_roll':    ['l_hip_1'  , T_DH(0, pi/2, 0, -pi/2)                   , 'l_hip_2'],
    'l_hip_yaw':     ['l_hip_2'  , T_DH(0, pi/2, 0, -pi/2)                   , 'l_thigh'],
    'l_knee':        ['l_thigh'  , T_DH(-.2236 , -pi/2, 0, pi/2)             , 'l_shank'],
    'l_ankle_pitch': ['l_shank'  , T_DH(0 , pi/2, -.213 , pi)                , 'l_ankle_1'],
    'l_ankle_roll':  ['l_ankle_1', T_DH(0, 0, 0, -pi/2)                      , 'l_foot'],

    'r_hip_pitch':   ['waist'    , dot(transl(0, .0681, -.1199), rotx(-pi/2)), 'r_hip_1'],
    'r_hip_roll':    ['r_hip_1'  , T_DH(0, pi/2, 0, pi/2)                    , 'r_hip_2'],
    'r_hip_yaw':     ['r_hip_2'  , T_DH(0, pi/2, 0, pi/2)                    , 'r_thigh'],
    'r_knee':        ['r_thigh'  , T_DH(.2236 , -pi/2, 0, -pi/2)             , 'r_shank'],
    'r_ankle_pitch': ['r_shank'  , T_DH(0 , pi/2, -.213 , pi)                , 'r_ankle_1'],
    'r_ankle_roll':  ['r_ankle_1', T_DH(0, 0, 0, pi/2)                       , 'r_foot'],
    }


def get_bodies_data():
    """ This dictionnary describes the bodies of the robot
    Each body is a list of shape
    name  : [ [(dims1)  | mass1  | H1],
              [(dims2)  | mass2  | H2] ]

    about dims (in meter):
    if dims has 1 element: it is a sphere; get (radius,)
    if dims has 2 elements: it is a cylinder with z-axis; get (length, radius)
    if dims has 3 elements: it is a box; get half-lengths along (x,y,z)

    masses are in kg

    about transformation matrix H:
    it represents the transformation from the body frame to the center of
    the shape.
    transl for translation along (x,y,z) (meter)
    rotx,roty, rotz for rotation around x,y,z axis
    dot for matrices multiplication
    """

    return {
    ### WARNING: pb between shape and inertia in the c++ file of the simulator
    #'waist'     : [[(0.002, 0.065, 0.027), 0.20297, dot(transl(0, 0, -0.065), rotx(pi/2))]],
    'waist'     : [[(.032, .0235, .057215), .20297, transl(.006, 0,-.1)]],
    'lap_belt_1': [[(.0315, .0635, .088)  , 3.623 , dot(transl(-.0383, -.0173, 0), rotz(-pi/4))]],
    'lap_belt_2': [[(.097, .031)          , .91179, transl(0, 0, .008)]],
    'chest':      [[(.0274, .04)          , .45165, transl(0, 0, -.0447)],
                   [(.036, .0545, .059)   , 1.8388, dot(transl(-.03, 0,-.1174), rotz( pi/12))],
                   [(.036, .0545, .059)   , 1.8388, dot(transl( .03, 0,-.1174), rotz(-pi/12))]],

    'neck_1': [[(.077, .015), .28252, eye(4)]],
    'neck_2': [[(.033, .015), .1    , eye(4)]],
    'head':   [[(.08,)      , .78   , transl(0, 0, .0825)]],

    'l_shoulder_1': [[(.011, .031)        , .48278 , transl(0, 0, .07224)]],
    'l_shoulder_2': [[(.059, .03)         , .20779 , eye(4)]],
    'l_arm':        [[(.156, .026)        , 1.1584 , transl(0, 0, .078)]],
    'l_elbow_1':    [[(.01,)              , .050798, eye(4)]],
    'l_forearm':    [[(.14, .02)          , .48774 , transl(0, 0, .07)]],
    'l_wrist_1':    [[(.04, .01)          , .05    , eye(4)]],
    'l_hand':       [[(.0345, .0325, .012), .19099 , transl(.0345, 0, 0)]],

    'r_shoulder_1': [[(.011, .031)        , .48278 , transl(0, 0, -.07224)]],
    'r_shoulder_2': [[(.059, .03)         , .20779 , eye(4)]],
    'r_arm':        [[(.156, .026)        , 1.1584 , transl(0, 0, -.078)]],
    'r_elbow_1':    [[(.01,)              , .050798, eye(4)]],
    'r_forearm':    [[(.14, .02)          , .48774 , transl(0, 0, -.07)]],
    'r_wrist_1':    [[(.04, .01)          , .05    , eye(4)]],
    'r_hand':       [[(.0345, .0325, .012), .19099 , transl(-.0345, 0, 0)]],

    'l_hip_1':   [[(.013, .038)      , .32708, transl(0, 0, .0375)]],
    'l_hip_2':   [[(.075, .031)      , 1.5304, eye(4)]],
    'l_thigh':   [[(.224, .034)      , 1.5304, transl(0, 0, -.112)],
                  [(.077,.0315)      , .79206, dot(transl(0, 0, -.224), roty(pi/2))]],
    'l_shank':   [[(.213, .0315)     , .95262, dot(transl(0, -.1065, 0), rotx(pi/2))]],
    'l_ankle_1': [[(.063, .0245)     , .14801, eye(4)]],
    'l_foot':    [[(.095, .027)      , .59285, transl(0,0,.0125)],
                  [(.002, .027, .065), .08185, transl(-.039, 0, .034)]],

    'r_hip_1':   [[(.013, .038)      , .32708, transl(0, 0, -.0375)]],
    'r_hip_2':   [[(.075, .031)      , 1.5304, eye(4)]],
    'r_thigh':   [[(.224, .034)      , 1.5304, transl(0, 0, .112)],
                  [(.077,.0315)      , .79206, dot(transl(0, 0, .224), roty(pi/2))]],
    'r_shank':   [[(.213, .0315)     , .95262, dot(transl(0, -.1065, 0), rotx(pi/2))]],
    'r_ankle_1': [[(.063, .0245)     , .14801, eye(4)]],
    'r_foot':    [[(.095, .027)      , .59285, transl(0,0,-.0125)],
                  [(.002, .027, .065), .08185, transl(-.039, 0, -.034)]],
    }


def get_bodies_shapes_data(shapes_from_inertia=True):
    """
    """
    bodies_shapes_data = {}

    for name, data in get_bodies_data().items():
        bodies_shapes_data[name] = [[d[0], d[2]] for d in data]

    bodies_shapes_data["head"].extend([
        [(.02,), transl(-.034, -.054, .0825)],  #l_eye
        [(.02,), transl( .034, -.054, .0825)],  #r_eye
    ])

    return bodies_shapes_data


def get_contact_data():
    """ somes extras: give some other important frames
    Each subframe is described as follows:
    subframe name : [body frame, dims, H_body_subframe]
    they can be used for contact point for example
    """
    return {
    'lh1'         : ['l_hand', (.012,), transl(.012,  .0205, 0.)],
    'lh2'         : ['l_hand', (.012,), transl(.012, -.0205, 0.)],
    'lh3'         : ['l_hand', (.012,), transl(.057,  .0205, 0.)],
    'lh4'         : ['l_hand', (.012,), transl(.057, -.0205, 0.)],
    'l_hand_tip'  : ['l_hand', (.012,), transl(.069, 0, 0)],
    'l_hand_palm' : ['l_hand', (.012,), transl(.0345, 0, 0)],

    'rh1'         : ['r_hand', (.012,), transl(-.012,  .0205, 0.)],
    'rh2'         : ['r_hand', (.012,), transl(-.012, -.0205, 0.)],
    'rh3'         : ['r_hand', (.012,), transl(-.057,  .0205, 0.)],
    'rh4'         : ['r_hand', (.012,), transl(-.057, -.0205, 0.)],
    'r_hand_tip'  : ['r_hand', (.012,), transl(-.069, 0, 0)],
    'r_hand_palm' : ['r_hand', (.012,), transl(-.0345, 0, 0)],

    'lf1'   : ['l_foot', (.002,), transl(-.039,-.027,-.031)],
    'lf2'   : ['l_foot', (.002,), transl(-.039, .027,-.031)],
    'lf3'   : ['l_foot', (.002,), transl(-.039, .027, .099)],
    'lf4'   : ['l_foot', (.002,), transl(-.039,-.027, .099)],
    'l_sole': ['l_foot', (), dot(transl(-.041, .0  , .034),
                                 dot(roty(-pi/2), rotx(-pi/2) ) )],

    'rf1'   : ['r_foot', (.002,), transl(-.039,-.027, .031)],
    'rf2'   : ['r_foot', (.002,), transl(-.039, .027, .031)],
    'rf3'   : ['r_foot', (.002,), transl(-.039, .027,-.099)],
    'rf4'   : ['r_foot', (.002,), transl(-.039,-.027,-.099)],
    'r_sole': ['r_foot', (), dot(transl(-.041, .0  ,-.034),
                                 dot(roty(pi/2), rotx(pi/2) ) )],
    }


def add(w, is_fixed=False, create_shapes=True, create_contacts=True):
    """
    construction of the icub robot for arboris-python:
    Kinematics data are from: http://eris.liralab.it/wiki/ICubForwardKinematics
    Inertia comes from the Icub.cpp used in the iCub_SIM program
    Some data are not well explained, or are badly defined
    """

    bodies_data = get_bodies_data()
    bodies_shapes_data = get_bodies_shapes_data()
    joints_data = get_joints_data()
    shapes_data = get_contact_data()

    ## bodies creation
    bodies = {}
    for name, data in bodies_data.items():
        bodies[name] = Body(name=name)
        mass = zeros((6, 6))
        for dims, m, H in data: # get dims, mass and transformation from data
            sf = SubFrame(bodies[name], H)
            if len(dims) == 3:      # check the type of shape: len =3: box
                M  = box(dims, m)
            elif len(dims) == 2:                            #  len =2:cylinder,
                M  = cylinder(dims[0], dims[1], m)
            elif len(dims) == 1:                            #  len =1:sphere,
                M  = sphere(dims[0], m)
            else:
                raise ValueError
            mass += transport(M, inv(H))    # add the mass of the shape to
        bodies[name].mass = mass            # the total mass

    ## check if iCub has its waist fixed on the structure (the ground)
    if is_fixed:
        bodies['waist'] = w.ground
    else:
        w.add_link(w.ground, FreeJoint(name='root'), bodies['waist'])

    ## joints creation
    for name, data in joints_data.items():
        parent, Hp_l, child = data
        w.add_link(SubFrame(bodies[parent], Hp_l),
                   RzJoint(name=name),
                   bodies[child])


    if create_shapes is True:
        ## body shapes creations
        for name, data in bodies_shapes_data.items():
            for dims, H in data: # get dims, mass and transformation from data
                sf = SubFrame(bodies[name], H)
                if len(dims) == 3:      # check the type of shape: len =3: box
                    sh = Box(sf, dims, name+".body_shape")
                elif len(dims) == 2:                            #  len =2:cylinder,
                    sh = Cylinder(sf, dims[0], dims[1], name+".body_shape")
                elif len(dims) == 1:                            #  len =1:sphere,
                    sh = Sphere(sf, dims[0], name+".body_shape")
                else:
                    raise ValueError
                w.register(sh)

    if create_contacts is True:
        ## contact shapes creation
        for name, data in shapes_data.items():
            parent, dims, Hpf = data
            sf = SubFrame(bodies[parent], Hpf, name=name)
            if len(dims) == 3:      # check the type of shape: len =3: box
                sh = Box(sf, dims, name=name)
            elif len(dims) == 2:                            #  len =2:cylinder,
                sh = Cylinder(sf, dims[0], dims[1], name=name)
            elif len(dims) == 1:                            #  len =1:sphere,
                sh = Sphere(sf, dims[0], name=name)
            else:
                sh = Point(sf, name=name)
            w.register(sh)

    w.init()

