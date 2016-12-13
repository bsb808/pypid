from setuptools import setup

import sys
import re

from setuptools.command.test import test as TestCommand

# pull the version from the file
version = re.search(
    '^__version__\s*=\s*"(.*)"',
    open('pypid/pypid.py').read(),
    re.M
    ).group(1)

# reuse the readme for the package description
with open("readme.rst", "rb") as f:
    long_descr = f.read().decode("utf-8")

# handle version based import requirements
install_requires = ['numpy']

name='pypid'
setup(
    name=name,
    version=version,
    packages=['pypid'],
    url='https://github.com/bsb808/pypid',
    license='TBD',
    author="Brian Bingham",
    long_description=long_descr,
    author_email='briansbingham@gmail.com',
    description='Python PID',
    tests_require=['pytest'],
    install_requires=install_requires,
)
