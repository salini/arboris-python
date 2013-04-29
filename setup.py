# coding: utf-8
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from distutils.core import setup, Command
import os.path
import subprocess

import os, sys, string, shutil, errno
from site import USER_BASE

def get_version():
    """
    Get the version from git or from the VERSION.txt file

    If we're in a git repository, uses the output of ``git describe`` as
    the version, and update the ``VERSION.txt`` file.
    Otherwise, read the version from the ``VERSION.txt`` file

    Much inspire from this post:
    http://dcreager.net/2010/02/10/setuptools-git-version-numbers/
    """

    def get_version_from_git():
        """Returns the version as defined by ``git describe``, or None."""
        try:
            p = subprocess.Popen(['git', 'describe'],
                      stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            p.stderr.close()
            line = p.stdout.readlines()[0].strip()
            assert line.startswith('v')
            return line[1:] #remove the leading 'v'
        except (OSError, IndexError):
            return None

    def get_version_from_file():
        """Returns the version as defined in the ``VERSION.txt`` file."""
        try:
            f = open('VERSION.txt', "r")
            version = f.readline().strip()
            f.close()
        except IOError:
            version = None
        return version

    def update_version_file(version):
        """Update, if necessary, the ``VERSION.txt`` file."""
        if version != get_version_from_file():
            f = open('VERSION.txt', "w")
            f.write(version+'\n')
            f.close()

    version = get_version_from_git()
    if version:
        update_version_file(version)
    else:
        version = get_version_from_file()
    return version


cmdclass = {}
try:
    # add a command for building the html doc
    from sphinx.setup_command import BuildDoc
    cmdclass['build_doc'] = BuildDoc
except ImportError:
    pass


def force_symlink(file1, file2):
    try:
        os.symlink(file1, file2)
    except OSError, e:
        if e.errno == errno.EEXIST:
            shutil.rmtree(file2)
            os.symlink(file1, file2)

class develop(Command):
    description = "Create symbolic link instead of installing files"
    user_options = [
            ('prefix=', None, "installation prefix"),
            ('uninstall', None, "uninstall development files")
            ]

    def initialize_options(self):
        self.prefix = None
        self.uninstall = 0

    def finalize_options(self):
        self.py_version = (string.split(sys.version))[0]
        if self.prefix is None:
            self.prefix = USER_BASE
        self.prefix = os.path.expanduser(self.prefix)

    def run(self):
        out_dir = os.path.join(self.prefix, "lib", "python"+self.py_version[0:3], "site-packages")
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)

        out_dir = os.path.join(out_dir, "arboris")
        src_dir = os.path.join(os.getcwd(), "arboris" )
        if self.uninstall == 1:
            if os.path.islink(out_dir):
                print "Removing symlink "+out_dir
                os.remove(out_dir)
            else:
                print "Not in dev mode, nothing to do"
        else:
            if os.path.islink(out_dir):
                print "Already in dev mode"
            else:
                print "Creating symlink "+src_dir+" -> "+out_dir
                force_symlink(src_dir, out_dir)

cmdclass['develop'] = develop



readme = open('README.md', 'r')
setup(name='arboris',
      version=get_version(),
      maintainer=unicode('Joseph SALINI & Sébastien BARTHÉLEMY', 'utf-8'),
      maintainer_email='salini@isir.upmc.fr',
      url='https://github.com/salini/arboris-python',
      description=
              'A rigid body dynamics and contacts simulator written in python.',
      long_description=unicode(readme.read(), 'utf-8'),
      license='LGPL',
      packages=['arboris',
                'arboris.robots'],
      package_data={'arboris': ['shapes.dae', 'scene.dae']},
      classifiers=[
              'Development Status :: 4 - Beta',
              'Environment :: Console',
              'Intended Audience :: Science/Research',
              'License :: OSI Approved :: GNU Library or Lesser General ' +\
                      'Public License (LGPL)',
              'Operating System :: OS Independent',
              'Programming Language :: Python',
              'Topic :: Scientific/Engineering :: Physics'],
        requires=['numpy', 'pycollada', 'pydaenim'],
      cmdclass=cmdclass)
readme.close()
