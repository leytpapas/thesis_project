import sys

from setuptools import find_packages, setup
from Cython.Build import cythonize

ext_options = {"compiler_directives": {"profile": True}, "annotate": True, 'language_level': sys.version_info[0]}
print(ext_options)
setup(ext_modules=cythonize('run_cython.pyx', **ext_options))
