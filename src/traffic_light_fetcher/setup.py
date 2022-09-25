from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['traffic_light_fetcher'],
    package_dir={'': 'src'},
    install_requires=['tensorflow', 'tensorflow_hub', 'numpy']
)
setup(**setup_args)
