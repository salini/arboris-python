Arboris & Tools

Installation on a Mac using macports
------------------------------------

(Written on 8 july 2011 : new versions of software might simplify the
 installation)

Note: This could take you some time, prepare a cup of coffee and clear
your evening.

Note: Versions specified are just informative. It worked with these
versions, but more recent ones should work as well. Especially, if a
variant is necessary it will be told explicitely.

Note: This file does not intend to be clean, optimal ! If you know how
to make it in a more proper way, please let us know.

Note: Lots of troubles can happen because of wrong permissions of files.
Check that you do not have bad a umask (especially for the root user).

Daenim
------

Note: These instructions show how to install daenim (and OpenSceneGraph)
in the /my/local directory.

* Installing minimal tools

Install the following packages: (sudo port install <package>)

cmake (current version: @2.8.4_0)
collada-dom (current version: @2.2)
hdf5-18 (current version: @1.8.7_1)

* Installing OpenSceneGraph > 2.9.9

At the time this file is written, macports does not offer any package
for OpenSceneGraph 2.9.9 or above. Therefore, you will have to install
it by hand.

svn checkout http://www.openscenegraph.org/svn/osg/OpenSceneGraph/tags/OpenSceneGraph-3.0.0 OpenSceneGraph
cd OpenSceneGraph
cmake -G Unix Makefiles -DCMAKE_INSTALL_PREFIX=/my/local
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_OSX_SYSROOT=/Developer/SDKs/MacOSX10.6.sdk
  -DCMAKE_OSX_ARCHITECTURES=x86_64 -DCMAKE_OSX_DEPLOYMENT_TARGET=10.6
  -DOSG_WINDOWING_SYSTEM=Cocoa -DOSG_DEFAULT_IMAGE_PLUGIN_FOR_OSX=imageio
make
make install

Note:
You will have possible warning about ITK not being set and Qt missing,
this is not important (it is only required for some examples and plugins
you won't need)

* Install Daenim

git clone git://github.com/salini/daenim.git

You will have to hack the file:
daenim/src/AnimtkViewerGUI.cpp and replace ICON_PATH by
"/my/local/share/daenim/Icons"

mkdir daenim-build
cd daenim-build
OSG_DIR=/my/local cmake -DCMAKE_INSTALL_PREFIX=/my/local ../daenim
make
make install

you need to add to your environment variable (~/.profile)
PATH=/my/local:$PATH
DYLIB_LIBRARY_PATH=/my/local/lib:$DYLIB_LIBRARY_PATH
OSG_LIBRARY_PATH=/my/local:$OSG_LIBRARY_PATH


Arboris-python
--------------

* Installing minimal tools

Install the following packages: (sudo port install <package>)
git-core (current version: @1.7.6_0)

* Installing python and required modules

IMPORTANT: 
Be sure to deactiavte ccache/distcc in macports (necessary for
py26-scipy to compile): 
- In /opt/local/etc/macports/macports.conf:
  set configureccache and configuredistcc to no
- If python and associated tools were previously installed with ccache
enabled, you will have to uninstall them all (which could implies lots
of packages).
- If you have no idea if it were or not installed while ccache being
enabled, then for safety and limi the risks, I would advise you to
uninstall them.

Install the following packages:
- python26 (current version: @2.6.7_0)
- py26-numpy (current version: @1.6.0_0+atlas+gcc44)
- py26-scipy (current version: @0.9.0_0+gcc44) 
- py26-matplotlib (current version: @1.0.1_3+tkinter) 
- py26-h5py (current version: @2.0.0_0)

* Install arboris-python:

git clone git://github.com/salini/arboris-python.git

cd arboris-python
sudo python setup.py install

Verify that everything is fine (daenim should be installed):
python examples/sim_minimal.py

ColladaTools
------------

Note: These instructions show how to install daenim (and OpenSceneGraph)
in the /my/local directory.

FIXME
git clone git://github.com/salini/ColladaTools.git
mkdir ColladaTools-build
cd ColladaTools-build
cmake ../ColladaTools
make
sudo cp bin/* /my/local/bin
rehash

Verify that everything is fine (daenim and arboris-python should be installed):
h5toanim --hdf5-file ../arboris-python/examples/current_anim.h5 --scene-file ../arboris-python/examples/current_scene.dae -o test.dae
daenim test.dae

LQPcontroller
-------------

* Installing minimal tools

Install the following packages: (sudo port install <package>)

py26-cvxopt (current version: @1.1.3_0+dsdp+fftw+glpk+gsl)

* Installing LQPctrl

git clone git://github.com/salini/LQPctrl.git
PYTHONLIBPATH=`python -c "import sys; print [ l for l in sys.path if l.endswith('site-packages') ][0]"` mv LQPctrl $PYTHONLIBPATH/LQPctrl

Note: better check that PYTHONLIBPATH is correct :)
