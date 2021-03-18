^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hfl_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2021-03-18)
------------------
* Merge pull request `#74 <https://github.com/continental/hfl_driver/issues/74>`_ from flynneva/fix/extrinsics_quaternion
* remove kinetic from CI due to EOL
* rotation fix, pointcloud size fix, telemetry parsing
* Merge pull request `#73 <https://github.com/continental/hfl_driver/issues/73>`_ from continental/ros1/main
* Merge pull request `#72 <https://github.com/continental/hfl_driver/issues/72>`_ from continental/ros1/devel
* bump ros actions
* Merge pull request `#71 <https://github.com/continental/hfl_driver/issues/71>`_ from continental/ros1/devel
* Merge pull request `#70 <https://github.com/continental/hfl_driver/issues/70>`_ from continental/ros1/devel
* fixed documentation
* Merge pull request `#69 <https://github.com/continental/hfl_driver/issues/69>`_ from continental/remove_legacy_code
* removed include tof
* removed msg gen
* removed legacy code & clean up cmakelist
* Merge pull request `#68 <https://github.com/continental/hfl_driver/issues/68>`_ from continental/ros1/devel
* fix some links in readme
* Merge pull request `#67 <https://github.com/continental/hfl_driver/issues/67>`_ from continental/ros1/devel
* Merge pull request `#66 <https://github.com/continental/hfl_driver/issues/66>`_ from continental/ros1/devel
* add link to continentals website
* Merge pull request `#65 <https://github.com/continental/hfl_driver/issues/65>`_ from continental/ros1/devel
* add note to readme
* Merge pull request `#64 <https://github.com/continental/hfl_driver/issues/64>`_ from continental/ros1/devel
* update readme
* Merge pull request `#63 <https://github.com/continental/hfl_driver/issues/63>`_ from continental/release-0.0.20
* Merge pull request `#62 <https://github.com/continental/hfl_driver/issues/62>`_ from continental/release-0.0.20
* Merge pull request `#59 <https://github.com/continental/hfl_driver/issues/59>`_ from continental/release-0.0.19
* Merge pull request `#55 <https://github.com/continental/hfl_driver/issues/55>`_ from continental/release-0.0.18
* Merge pull request `#51 <https://github.com/continental/hfl_driver/issues/51>`_ from continental/release-0.0.18
* Merge pull request `#48 <https://github.com/continental/hfl_driver/issues/48>`_ from continental/ros1/devel
* Contributors: Evan Flynn, flynneva

0.0.20 (2020-09-15)
-------------------
* fixed package.xml for rosdep install
* Merge pull request `#58 <https://github.com/continental/hfl_driver/issues/58>`_ from continental/release-0.0.19
* Contributors: Evan Flynn

0.0.19 (2020-09-11)
-------------------
* 0.0.19
* Update changelog
* Merge pull request `#54 <https://github.com/continental/hfl_driver/issues/54>`_ from continental/release-0.0.18
* removed hfl_utilties ros package.xml
* Merge pull request `#52 <https://github.com/continental/hfl_driver/issues/52>`_ from continental/release-0.0.18
* Contributors: Evan Flynn

0.0.18 (2020-09-11)
-------------------
* put in some dummy tests for now
* fixed one unit test
* hfl_utilities needs to be a SHARED lib
* missing image_geometry in package.xml
* fixed merge mistake
* Merge branch 'updates' into ros1/devel
* added comment referencing initMatrix function
* Merge pull request `#46 <https://github.com/continental/hfl_driver/issues/46>`_ from continental/main
* merge in public github changes
* - various updates to pointcloud output
* Merge pull request `#45 <https://github.com/continental/hfl_driver/issues/45>`_ from continental/release-0.0.17
* Contributors: Evan Flynn

0.0.17 (2020-08-28)
-------------------
* Merge pull request `#42 <https://github.com/continental/hfl_driver/issues/42>`_ from continental/ros1/main
* fixed build export depend typo
* add dynamic_reconfigure to build depend
* 0.0.17
* Update changelog
* Merge pull request `#42 <https://github.com/continental/hfl_driver/issues/42>`_ from continental/ros1/main
* Merge pull request `#39 <https://github.com/continental/hfl_driver/issues/39>`_ from continental/ros1/main
* Merge pull request `#34 <https://github.com/continental/hfl_driver/issues/34>`_ from continental/release-0.0.14
* Merge pull request `#31 <https://github.com/continental/hfl_driver/issues/31>`_ from continental/ros1/main
* Merge pull request `#29 <https://github.com/continental/hfl_driver/issues/29>`_ from continental/ros1/devel
* Merge pull request `#21 <https://github.com/continental/hfl_driver/issues/21>`_ from continental/ros1/devel
* Merge pull request `#15 <https://github.com/continental/hfl_driver/issues/15>`_ from continental/ros1/devel
* Merge pull request `#12 <https://github.com/continental/hfl_driver/issues/12>`_ from continental/ros1/main
* Contributors: Evan Flynn

0.0.16 (2020-08-28)
-------------------
* Merge pull request `#38 <https://github.com/continental/hfl_driver/issues/38>`_ from continental/ros1/main
* add tf to package.xml
* Update CMakeLists.txt
* add tf to find
* add tf2_geometry_msgs to find
* fixed tf build error
* clean up
* Contributors: Evan Flynn

0.0.15 (2020-08-28)
-------------------
* open rosrepo pr automatically
* Merge pull request `#35 <https://github.com/continental/hfl_driver/issues/35>`_ from continental/release-0.0.14
* 0.0.14
* Update changelog
* Merge pull request `#30 <https://github.com/continental/hfl_driver/issues/30>`_ from continental/ros1/main
* fixed release workflow typo
* Merge pull request `#28 <https://github.com/continental/hfl_driver/issues/28>`_ from continental/ros1/main
* Contributors: Evan Flynn, flynneva

0.0.14 (2020-08-28)
-------------------
* Merge pull request `#30 <https://github.com/continental/hfl_driver/issues/30>`_ from continental/ros1/main
* fixed release workflow typo
* Merge pull request `#28 <https://github.com/continental/hfl_driver/issues/28>`_ from continental/ros1/main
* Contributors: Evan Flynn

0.0.13 (2020-08-28)
-------------------
* fixed roslint error

0.0.12 (2020-08-24)
-------------------
* removed unnecessary install files
* fix udp_com linking error
* update suppoted platforms
* test depend roslint
* move roslint within conditional
* fixed rostest and arpa warning
* should be catkin_add_gtest
* removed arpa
* fixed for kinetic
* fixed cmake warnings and kinetic error
* added archive destination to install step
* switch back to action-ros-ci
* switch back to manual ros ci
* gh-pages should be html directory
* Update ros_ci.yml
* Merge pull request `#13 <https://github.com/continental/hfl_driver/issues/13>`_ from continental/ros1/main
* Contributors: Evan Flynn

0.0.11 (2020-08-04)
-------------------
* Merge pull request `#9 <https://github.com/continental/hfl_driver/issues/9>`_ from continental/main
* updated release workflow
* Contributors: Evan Flynn

0.0.10 (2020-08-04)
-------------------

