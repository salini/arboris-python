#!/usr/bin/env python

"""
This thest_arboris_coonection is a test to create an Arboris observer to
communicate with a html page. This page will be update to show the robot
moving in a three.js instance...
"""



import dae_writer


from arboris.core import Observer
import json

import wsdaenim
from threading import Timer


class wsDaenimCom(Observer):
    """
    """
    def __init__(self, colladafile, host="localhost", port=5000, timeout=5, flat=False, name=None):
        Observer.__init__(self, name)
        self.flat = flat
        
        self.websocket = wsdaenim.wsDaenimWebSocket(host, port, timeout)
        
        kwargs = {"host":self.websocket.host, "port":self.websocket.port}
        Timer(0.1, wsdaenim.create_wsdaenimPlayer, (colladafile,), kwargs).start()
        
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
            frame_dict[f.name] =  list(H.pose[0:3, :].reshape(12))

        msg_list.append(frame_dict)
        msg = json.dumps(msg_list, separators=(',',':'))

        try:
            self.websocket.send_message(msg)
        except:
            print("webDaenimCom not connected")


    def finish(self):
        pass
