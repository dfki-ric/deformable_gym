from setuptools import setup


if __name__ == "__main__":
    with open("README.md", "r") as f:
        long_description = f.read()

    setup(name='deformable_gym',
          version="0.3.3",
          maintainer='Melvin Laux',
          maintainer_email='melvin.laux@uni-bremen.de',
          description='Gym environments for grasping deformable objects',
          long_description=long_description,
          long_description_content_type="text/markdown",
          license='BSD-3-Clause',
          packages=["deformable_gym"],
          install_requires=[
              "pybullet",
              "gymnasium",
              "numpy",
              "pytransform3d"
          ])
