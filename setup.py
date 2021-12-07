# -*- coding: utf-8 -*-
"""
Created on Fri Mar 26 22:45:22 2021
@author: Matteo
"""

from setuptools import setup
from setuptools import find_packages




setup(name='rover4ws_kinematics',
      version='1.0.0',
      description='This package allows implementing a kinematic controller for a four ws rover',
      url='https://github.com/matteocaruso1993/rover4ws-kinematics',
      author='Matteo Caruso',
      author_email='matteo.caruso@phd.units.it',
      license="GPLv3",
      packages=find_packages(),
      package_data = {'rover4ws_kinematics': ['config/config.yaml']},
      install_requires=['numpy', 'matplotlib', 'scikit-spatial', 'shapely','yaml'],
      zip_safe=False,
      include_package_data=True,
      python_requires='>=3.6'
      )