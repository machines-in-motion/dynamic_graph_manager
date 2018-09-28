from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['dynamic_graph_manager'],
    package_dir={'' : 'python'},
    scripts=['python/dynamic_graph_manager'],
)

setup(**setup_args)
