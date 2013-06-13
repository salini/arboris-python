# coding=utf-8

"""Import all useful functions, classes and modules from arboris.

This is meant for interactive use.
"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

#pylint: disable-msg=W0611

from arboris.controllers import WeightController, \
                                ProportionalDerivativeController

from arboris.robots.human36 import add_human36
from arboris.robots.simpleshapes import add_sphere, add_box, add_cylinder, \
                                        add_groundplane
from arboris.robots.simplearm import add_simplearm
from arboris.robots.snake import add_snake

from arboris.visu.dae_writer import write_collada_scene, \
                                    write_collada_animation

from numpy import arange, dot, allclose, pi
