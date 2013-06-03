# -*- coding: utf-8 -*-
#
# arboris-python documentation build configuration file, created by
# sphinx-quickstart on Mon Mar 16 21:29:38 2009.
#
# This file is execfile()d with the current directory set to its containing dir.
#
# The contents of this file are pickled, so don't put values in the namespace
# that aren't pickleable (module imports are okay, they're removed automatically).
#
# All configuration values have a default value; values that are commented out
# serve to show the default value.

import sys, os

# If your extensions are in another directory, add it here. If the directory
# is relative to the documentation root, use os.path.abspath to make it
# absolute, like shown here.
sys.path.append(os.path.abspath('..'))
sys.path.append(os.path.abspath('../tests'))

# General configuration
# ---------------------

# Add any Sphinx extension module names here, as strings. They can be extensions
# coming with Sphinx (named 'sphinx.ext.*') or your custom ones.
extensions = ['sphinx.ext.autodoc',
              'sphinx.ext.doctest',
              'sphinx.ext.pngmath',
              'sphinx.ext.inheritance_diagram']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['.templates']

# The suffix of source filenames.
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# General substitutions.
project = 'arboris-python'
copyright = '2009, Sebastien BARTHELEMY'

# The default replacements for |version| and |release|, also used in various
# other places throughout the built documents.
#
# The short X.Y version.
version = '0.2'
# The full version, including alpha/beta/rc tags.
# get the release from arboris/__init__.py
f = open('../VERSION.txt', "r")
try:
    release = f.readlines()[0].strip()
except OSError:
    release = None
finally:
    f.close()
assert release.startswith(version)

# There are two options for replacing |today|: either, you set today to some
# non-false value, then it is used:
#today = ''
# Else, today_fmt is used as the format for a strftime call.
today_fmt = '%B %d, %Y'

# List of documents that shouldn't be included in the build.
#unused_docs = []

# List of directories, relative to source directories, that shouldn't be searched
# for source files.
#exclude_dirs = []

# The reST default role (used for this markup: `text`) to use for all documents.
#default_role = None
default_role = 'math'

# If true, '()' will be appended to :func: etc. cross-reference text.
#add_function_parentheses = True

# If true, the current module name will be prepended to all description
# unit titles (such as .. function::).
#add_module_names = True

# If true, sectionauthor and moduleauthor directives will be shown in the
# output. They are ignored by default.
#show_authors = False

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'


# Options for HTML output
# -----------------------

# The style sheet to use for HTML and HTML Help pages. A file of that name
# must exist either in Sphinx' static/ path, or in one of the custom paths
# given in html_static_path.
html_style = 'default.css'

# The name for this set of Sphinx documents.  If None, it defaults to
# "<project> v<release> documentation".
#html_title = None

# A shorter title for the navigation bar.  Default is the same as html_title.
#html_short_title = None

# The name of an image file (within the static path) to place at the top of
# the sidebar.
#html_logo = None

# The name of an image file (within the static path) to use as favicon of the
# docs.  This file should be a Windows icon file (.ico) being 16x16 or 32x32
# pixels large.
#html_favicon = None

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['.static']

# If not '', a 'Last updated on:' timestamp is inserted at every page bottom,
# using the given strftime format.
html_last_updated_fmt = '%b %d, %Y'

# If true, SmartyPants will be used to convert quotes and dashes to
# typographically correct entities.
#html_use_smartypants = True

# Custom sidebar templates, maps document names to template names.
#html_sidebars = {}

# Additional templates that should be rendered to pages, maps page names to
# template names.
#html_additional_pages = {}

# If false, no module index is generated.
#html_use_modindex = True

# If false, no index is generated.
#html_use_index = True

# If true, the index is split into individual pages for each letter.
#html_split_index = False

# If true, the reST sources are included in the HTML build as _sources/<name>.
#html_copy_source = True

# If true, an OpenSearch description file will be output, and all pages will
# contain a <link> tag referring to it.  The value of this option must be the
# base URL from which the finished HTML is served.
#html_use_opensearch = ''

# If nonempty, this is the file name suffix for HTML files (e.g. ".xhtml").
#html_file_suffix = ''

# Output file base name for HTML help builder.
htmlhelp_basename = 'arboris-pythondoc'


# Options for LaTeX output
# ------------------------

# The paper size ('letter' or 'a4').
#latex_paper_size = 'letter'
latex_paper_size = 'a4'

# The font size ('10pt', '11pt' or '12pt').
#latex_font_size = '10pt'

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title, author, document class [howto/manual]).
latex_documents = [
  ('index', 'arboris-python.tex', 'arboris-python Documentation',
   'Sebastien BARTHELEMY', 'manual'),
]

# The name of an image file (relative to this directory) to place at the top of
# the title page.
#latex_logo = None

# For "manual" documents, if this is true, then toplevel headings are parts,
# not chapters.
#latex_use_parts = False

# Additional stuff for the LaTeX preamble.


latex_preamble = r"""
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\newcommand{\tens}[1]           {#1}
\newcommand{\vect}[1]           {\mbox{\boldmath${#1}$}}
\newcommand{\argmin}[1]         {\underset{#1}{\operatorename{argmin}}}
\newcommand{\diag}              {\mathop{\mathrm{diag}}}
\newcommand{\norm}[1]           {\left\lVert #1 \right\rVert}
\newcommand{\tp}                {^{\mathsf{T}}}
\renewcommand{\skew}[1]         {\widehat{#1}}

\newcommand{\dt}                {\delta t}                  %dt

\newcommand{\ft}[2]             {_{#1,#2}}                  %from to: generaly to write: H_(i,j) or Ad_(i,j) (H from i to j)
\newcommand{\rt}[2]             {_{#1/#2}}                  %first relative to second
\newcommand{\icf}[2][{}]        {\vphantom{#2}^{#1}\!#2}    %in coordinate frame

\newcommand{\pt}[1][p]          {\vect{#1}}                 %point in space
\newcommand{\ve}[1][u]          {\vect{#1}}                 %vector in space

\newcommand{\force}[1][{}]      {\icf[#1]{\vect{f}}}        %one generalized force
\newcommand{\linforce}[1][{}]   {\force[#1]}                %translational force in wrench
\newcommand{\rotforce}[1][{}]   {\icf[#1]{\vect{\tau}}}     %rotational force in wrench
\newcommand{\wrench}[1][{}]     {\icf[#1]{\vect{W}}}        %wrench

\newcommand{\vel}[1][{}]        {\icf[#1]{\vect{v}}}        %one generalized velocity
\newcommand{\linvel}[1][{}]     {\vel[#1]}                  %translational velocity in twist
\newcommand{\rotvel}[1][{}]     {\icf[#1]{\vect{\omega}}}   %rotational velocity in twist
\newcommand{\twist}[1][{}]      {\icf[#1]{\vect{V}}}        %twist
\newcommand{\dtwist}[1][{}]     {\icf[#1]{\dot{\vect{V}}}}  %derivative of twist

\newcommand{\HM}                {\tens{H}}                  %homogeneous matrix
\newcommand{\Rot}               {\tens{R}}                  %rotation matrix
\newcommand{\Ad}                {\tens{Ad}}                 %adjoint matrix
\newcommand{\dAd}               {\dot{\Ad}}                 %derivative adjoint matrix

\newcommand{\J}[1][{}]          {\icf[#1]{\tens{J}}}        %jacobian matrix
\newcommand{\dJ}[1][{}]         {\icf[#1]{\dot{\tens{J}}}}  %derivative jacobian matrix

\newcommand{\Frame}[1][{}]      {\Psi_{#1}}                 %frame
\newcommand{\R}[1]              {\mathbb{R}^{#1}}           %real space
\newcommand{\Id}[1]             {\tens{I}_{#1}}             %identity
\newcommand{\In}[1][{}]         {\icf[#1]{\tens{\mathcal{I}}}}  %inertia

\newcommand{\torque}            {\vect{\tau}}               %torque
\newcommand{\q}                 {\vect{q}}                  %generalized coordinate q
\newcommand{\dq}                {\dot{\vect{q}}}            %generalized velocity dq
\newcommand{\ddq}               {\ddot{\vect{q}}}           %generalized acceleration ddq

\newcommand{\GPos}              {Q}
\newcommand{\GPosSet}           {\bar{\mathcal{Q}}}
\newcommand{\GVel}              {\nu}
\newcommand{\dGVel}             {\dot{\nu}}
\newcommand{\GAcc}              {\dot{\nu}}
\newcommand{\GForce}            {\gamma}

"""

# Documents to append as an appendix to all manuals.
#latex_appendices = []

# If false, no module index is generated.
#latex_use_modindex = True

pngmath_latex_preamble = latex_preamble

#Misc
#====
todo_include_todos = True


autoclass_content = 'both'


