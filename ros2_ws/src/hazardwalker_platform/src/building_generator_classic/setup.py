from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup(
    **generate_distutils_setup(
        packages=["building_generator_classic"],
        package_dir={"": "."},
    )
)
