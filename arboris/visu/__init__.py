#!/usr/bin/env python

"""
"""


from arboris.core import Observer

import pydaenim

from threading import Timer
import json

from arboris.core import World
from   arboris.visu.dae_writer import write_collada_scene
import tempfile
import os


class pydaenimCom(Observer):
    """
    """
    def __init__(self, colladafile, host="localhost", port=5000, timeout=5, flat=False, name=None):
        Observer.__init__(self, name)
        self.flat = flat
        
        self.websocket = pydaenim.pydaenimWebSocket(host, port, timeout)
        
        kwargs = {"host":self.websocket.host, "port":self.websocket.port}
        Timer(0.1, pydaenim.create_pydaenimViewer, (colladafile,), kwargs).start()
        
        self.websocket.listen()


    def init(self, world, timeline):
        """
        """
        self.world = world


    def update(self, dt):
        """
        """
        msg_list = ["update_transforms", self.world.current_time]
        frame_dict = {}

        if self.flat:
            for b in self.world.getbodies():
                frame_dict[b.name] = list(b.pose[0:3, :].reshape(12))
        else:
            for j in self.world.getjoints():
                frame_dict[j.frames[1].name] = list(j.pose[0:3, :].reshape(12))

        for f in self.world.itermovingsubframes():
            H = f.pose if self.flat else f.bpose
            frame_dict[f.name] =  list(H[0:3, :].reshape(12))

        msg_list.append(frame_dict)
        msg = json.dumps(msg_list, separators=(',',':'))

        try:
            self.websocket.send_message(msg)
        except:
            print("webDaenimCom not connected")


    def finish(self):
        pass



def view(world, browser=None, **kwargs):
    """
    """
    if isinstance(world, World):

        tempdir = tempfile.gettempdir()
        collada_path = tempdir+os.sep+"temp_scene.dae"

        write_collada_scene(world, collada_path)

    elif isinstance(world, basestring):
        collada_path = world

    pydaenim.create_pydaenimViewer(collada_path, browser, **kwargs)


