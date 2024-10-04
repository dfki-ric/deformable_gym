from setuptools import setup

if __name__ == "__main__":
    with open("README.md", "r") as f:
        long_description = f.read()

    setup(
        name="deformable_gym",
        version="0.4.1",
        maintainer="Melvin Laux",
        maintainer_email="melvin.laux@uni-bremen.de",
        description="Gym environments for grasping deformable objects",
        long_description=long_description,
        long_description_content_type="text/markdown",
        license="BSD-3-Clause",
        packages=["deformable_gym"],
        install_requires=[
            "pybullet",
            "gymnasium>=1.0.0a2",
            "numpy>=1.23.5,<2.0.0",
            "pytransform3d",
        ],
        extras_require={
            "mujoco": [
                "mujoco==3.1.6",
            ],
            "dev": ["pytest", "pre-commit", "flake8"],
        },
    )
