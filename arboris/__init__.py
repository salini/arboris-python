# coding=utf-8
"""Check which modules are availlable."""

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

def optional_modules():
    """A list of optional modules which are available."""
    mods = []
#    try:                             # this is an old module.
#        import arboris.qpcontroller  # it is not available anymore.
#        mods.append('qpcontroller')  # this snippet of code is kept as example
#    except ImportError:              # for further use of optional_modules
#        pass
    return mods

__all__ = ['adjointmatrix', 'collisions', 'constraints', 'controllers',
           'core', 'homogeneousmatrix', 'joints', 'massmatrix',
           'observers', 'rigidmotion', 'robots', 'shapes', 'twistvector',
           'visu_collada']
__all__.extend(optional_modules())
