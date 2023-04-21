from setuptools import setup


if __name__ == "__main__":
    with open("README.md", "r") as f:
        long_description = f.read()

    setup(name='deformable_gym',
          version="0.1",
          maintainer='Melvin Laux',
          maintainer_email='melvin.laux@dfki.de',
          description='APRIL gym environments for grasping deformable objects',
          long_description=long_description,
          long_description_content_type="text/markdown",
          license='Not public',
          packages=["deformable_gym"],
          install_requires=["pybullet",
                            "gym",
                            "numpy",
                            "pytransform3d"
                            ])
