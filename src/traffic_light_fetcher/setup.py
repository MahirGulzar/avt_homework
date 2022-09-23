from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['traffic_light_fetcher'],
    package_dir={'': 'include/traffic_light_fetcher'},
)
setup(**setup_args)
