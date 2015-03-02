# DO NOT USE
# python setup.py install
# See http://docs.ros.org/hydro/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tiptri_detection'],
    package_dir={'': 'src'}
)

setup(**d)
