# NOTE: This package uses ament_cmake as its build type.
# Data files, launch files, and executable scripts are installed via
# CMakeLists.txt.  setup.py is only needed so that
# ament_python_install_package() can install the Python package.
from setuptools import find_packages, setup

package_name = 'primitive_actions'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prabinrath',
    maintainer_email='prabinrath123@gmail.com',
    description='Primitive action scripts for robot manipulation tasks.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
