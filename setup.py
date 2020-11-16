"""Setup script

For details: https://packaging.python.org/en/latest/distributing.html
"""
import os
import setuptools
from setuptools.extension import Extension
from Cython.Compiler.Options import get_directive_defaults
directive_defaults = get_directive_defaults()
directive_defaults['linetrace'] = True
directive_defaults['binding'] = True
from Cython.Build import cythonize


here = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(here, 'readme.md'), encoding='utf-8') as fd:
    long_description = fd.read()

extensions = [
    Extension('springs.engine.cython', ['springs/engine/_cython/cython.pyx'],
              extra_compile_args=['-Wno-deprecated']), # avoid tp_print warnings
              # , define_macros=[('CYTHON_TRACE', '1')]),
    Extension('springs.engine.cpp', ['springs/engine/_cpp/cpp.pyx'], language='c++',
              extra_compile_args=['-std=c++17', '-Wno-deprecated'],
              extra_link_args=['-std=c++17'])
]


setuptools.setup(
    name='springs',
    version='0.5.0',

    description='Springs simulator',
    long_description=long_description,

    url='https://github.com/oist-cnru/pmstrnn',

    author='OIST CNRU',
    author_email='fabien.benureau@oist.jp',

    license='Open Science License',

    keywords='',

    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        'Development Status :: 4 - Beta',

        'Intended Audience :: Science/Research',

        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
    ],

    # where is our code
    packages=['springs'],
    ext_modules = cythonize(extensions),

    # required dependencies
    install_requires=['numpy', 'cython', 'reproducible', 'setproctitle', 'pyqt5'],
)
