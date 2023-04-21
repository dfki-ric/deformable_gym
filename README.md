# APRIL Deformable Grasping Gym

This repository contains a collection of RL gym environments built with PyBullet. In these environments, the agent 
needs to learn to grasp deformable object such as shoe insoles or pillows.



## Installation 

The Mia hand URDF is integrated as a git submodule. Please clone this repository using the `--recurse-submodules` option.

```bash
git clone https://github.com/dfki-ric/deformable_gym
```

After cloning the repository, it is recommended to install the library in editable mode.

```bash
pip install -e .
```

### Known Issues

If you use conda, you may experience the error below:

```
libGL error: MESA-LOADER: failed to open iris: /usr/lib/dri/iris_dri.so: cannot open shared object file: No such file or directory (search paths /usr/lib/x86_64-linux-gnu/dri:\$${ORIGIN}/dri:/usr/lib/dri, suffix _dri)
libGL error: failed to load driver: iris
libGL error: MESA-LOADER: failed to open swrast: /usr/lib/dri/swrast_dri.so: cannot open shared object file: No such file or directory (search paths /usr/lib/x86_64-linux-gnu/dri:\$${ORIGIN}/dri:/usr/lib/dri, suffix _dri)
libGL error: failed to load driver: swrast
```

In this case, install the following dependency via conda-forge:

```bash
conda install -c conda-forge libstdcxx-ng
```


## Documentation
The documentation can be found in the directory doc. To build the documentation, run e.g. (on linux):

```
cd doc
make html
```
The HTML documentation is now located at doc/build/html/index.html. You need the following packages to build the documentation:

```
pip install numpydoc sphinx sphinx-gallery sphinx-bootstrap-theme
```

## Contributing

If you wish to report bugs, please use the [issue tracker](https://git.hb.dfki.de/april/learning/deformable_gym/-/issues) 
at Gitlab. If you would like to contribute to DeformableGym, just open an issue or a 
[merge request](https://git.hb.dfki.de/april/learning/deformable_gym/-/merge_requests). The target branch for 
merge requests is the development branch. The development branch will be merged to master for new releases. If you have 
questions about the software, you should ask them in the discussion section.

The recommended workflow to add a new feature, add documentation, or fix a bug is the following:
- Push your changes to a branch (e.g. feature/x, doc/y, or fix/z) of your fork of the deformable_gym repository.
- Open a pull request to the latest development branch. There is usually an open merge request from the latest development branch to the main branch. 
- When the latest development branch is merged to the main branch, a new release will be made.

Note that there is a checklist for new features.

It is forbidden to directly push to the main branch. Each new version has its own development branch from which a pull request will be opened to the main branch. Only the maintainer of the software is allowed to merge a development branch to the main branch.

## Referencing

```
@MISC{Laux2023,
author = {Melvin Laux and Alexander Fabisch and Chandandeep Singh and Johannes Brust},
title = {DeformableGym, a Reinforcement Learning Benchmark for Grasping 3D Deformable Objects},
howpublished = {\url{TODO}},
year = {2023}
}
```

## Releases

### Semantic Versioning

Semantic versioning must be used, that is, the major version number will be
incremented when the API changes in a backwards incompatible way, the minor
version will be incremented when new functionality is added in a backwards
compatible manner, and the patch version is incremented for bugfixes,
documentation, etc.


## Funding

This library has been developed initially at the
[Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the 
[German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen together with the 
[Robotics Group](https://robotik.dfki-bremen.de/en/about-us/university-of-bremen-robotics-group.html) of the 
[University of Bremen](http://www.uni-bremen.de/en.html). At this phase, the work was supported through a grant from the European
Commission (870142).

<p float="left">
    <img src="doc/source/_static/DFKI_Logo.jpg" height="100px" />
    <img src="doc/source/_static/Uni_Logo.png" height="100px" />
</p>





