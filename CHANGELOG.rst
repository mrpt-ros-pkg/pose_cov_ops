^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pose_cov_ops
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix package.xml build_type for ros1/ros2
* Contributors: Jose Luis Blanco Claraco

0.3.5 (2022-06-10)
------------------
* Remove find_package() calls due to wrong mrpt-ros2bridge-config.cmake in older mrpt2 versions
* Contributors: Jose Luis Blanco-Claraco

0.3.4 (2022-05-31)
------------------
* Fix missing cmake xmllint at configure time.
* Contributors: Jose Luis Blanco-Claraco

0.3.3 (2022-05-30)
------------------
* Fix build for ros2
* Rename license as "LICENSE"
* Add contributing.md file
* Contributors: Jose Luis Blanco-Claraco

0.3.2 (2022-05-10)
------------------
* Unify ros1/ros2 build system in one single git branch.
* Port to mrpt2, avoid dependency on pkg mrpt_bridge
* Contributors: Jose Luis Blanco Claraco

0.3.1 (2022-05-04)
------------------
* Add License file
* remove wrong xml comment reference to mrpt1
* remove travis CI
* Update README.md
* Contributors: Jose Luis Blanco-Claraco

0.3.0 (2022-03-04)
------------------
* Update URLs to https
* Update build dep to mrpt2
* update to mrpt2
* add basic unit test
* Set message to status again instead of warning
* Updating CMakeLists.txt to pass build on Ubuntu 16.04LTS with CMake 3.5 with the latest version of MRPT
* Add ros build farm badgets and links
* Contributors: Inounx, Jose Luis Blanco-Claraco, Julian Lopez Velasquez

0.2.1 (2018-05-21)
------------------
* fix catkin_lint issues
* Contributors: Jose Luis Blanco Claraco

0.2.0 (2018-01-09)
------------------
* Made compatible with MRPT 1.x and 2.x
* Contributors: Jose Luis Blanco Claraco

0.1.7 (2017-12-09)
------------------
* relaxing need to find lib mrpt >=1.5
* Silent GCC warnings with Eigen hdrs
* Contributors: Jose Luis Blanco Claraco

0.1.6 (2017-12-04)
------------------
* Fix catkin dependencies
* Use C++14
* Fix build against MRPT 1.5 and 2.0
* Contributors: Javier G. Monroy, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

0.1.5 (2014-12-23)
------------------

0.1.4 (2014-12-23)
------------------
* Ported from old repository in mrpt_navigation

0.1.3 (2014-12-18)
------------------
* Fix many missing install files
* Contributors: Jose Luis Blanco

0.1.2 (2014-12-18)
------------------

0.1.1 (2014-12-17)
------------------
* First public binary release.

0.1.0 (2014-12-17)
------------------
* consistent version numbers
* Fixed: pose_cov_ops needs export section in manifest
