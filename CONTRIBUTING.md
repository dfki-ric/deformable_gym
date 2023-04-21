# Contributing

Everyone is welcome to contribute.

There are several ways to contribute to DeformableGym: you could

* send a bug report to the
  [bug tracker](https://git.hb.dfki.de/april/learning/deformable_gym)
* work on one of the reported issues
* write documentation
* add a new feature
* add tests
* add an example

## How to Contribute Code

This text is shamelessly copied from
[scikit-learn's](https://scikit-learn.org/stable/developers/contributing.html)
contribution guidelines.

The preferred way to contribute to DefromableGym is to fork the
[repository](https://git.hb.dfki.de/april/learning/deformable_gym) on GitLab,
then submit a "merge request" (MR):

1. Fork the [project repository](https://git.hb.dfki.de/april/learning/deformable_gym):
   click on the 'Fork' button near the top of the page. This creates a copy of
   the code under your account on the GitHub server.

2. Clone this copy to your local disk:

       $ git clone --recurse-submodules git@git.hb.dfki.de:april/learning/deformable_gym.git

3. Create a branch to hold your changes:

       $ git checkout -b my-feature

   and start making changes. Never work in the `main` branch!

4. Work on this copy, on your computer, using Git to do the version
   control. When you're done editing, do:

       $ git add modified_files
       $ git commit

   to record your changes in Git, then push them to GitHub with:

       $ git push -u origin my-feature

Finally, go to the web page of your fork of the deformable_gym repository,
and click 'Merge request' to send your changes to the maintainer for review.
Make sure that your target branch is 'development'.

In the above setup, your `origin` remote repository points to
YourLogin/deformable_gym.git. If you wish to fetch/merge from the main
repository instead of your forked one, you will need to add another remote
to use instead of `origin`. If we choose the name `upstream` for it, the
command will be:

    $ git remote add upstream https://git.hb.dfki.de/april/learning/deformable_gym

(If any of the above seems like magic to you, then look up the
[Git documentation](http://git-scm.com/documentation) on the web.)

## Requirements for New Features

Adding a new feature to deformable_gym requires a few other changes:

* New classes or functions that are part of the public interface must be
  documented. We use [NumPy's conventions for docstrings](https://github.com/numpy/numpy/blob/master/doc/HOWTO_DOCUMENT.rst.txt).
* Consider writing a simple example script.
* Tests: Unit tests for new features are mandatory. They should cover all
  branches. Exceptions are plotting functions, debug outputs, etc. These
  are usually hard to test and are not a fundamental part of the library.

## Merge Policy

Usually it is not possible to push directly to the development or main branch for
anyone. Only tiny changes, urgent bugfixes, and maintenance commits can be
pushed directly to the master branch by the maintainer without a review.
"Tiny" means backwards compatibility is mandatory and all tests must succeed.
No new feature must be added.

Developers have to submit merge requests. Those will be reviewed and merged by
a maintainer. New features must be documented and tested. Breaking changes must
be discussed and announced in advance with deprecation warnings.

## Versioning

Semantic versioning is used, that is, the major version number will be
incremented when the API changes in a backwards incompatible way, the
minor version will be incremented when new functionality is added in a
backwards compatible manner.