^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_babel_fish
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.6 (2024-12-30)
------------------
* Fixes service server segfaulting (`#10 <https://github.com/LOEWE-emergenCITY/ros_babel_fish/issues/10>`_) and adds a new test to cover this.
* Added convenience methods to get and set values of compound messages.
* Add ActionServer (`#9 <https://github.com/LOEWE-emergenCITY/ros_babel_fish/issues/9>`_)
  * More verbose error message when trying to assign a value to a CompoundMessage.
  * Moved BabelFishAction definition to separate header in preparation for BabelFishActionServer.
  * Added BabelFishActionServer with tests.
  * Suppress false positive in cppcheck.
  * Enable inline suppression in cppcheck.
* Updated test message and added goal rejected and cancel rejected tests for client.
* Refactored action client and added new tests.
* Added more method documentation.
* Added convenience methods to create empty action goals with BabelFish.
* Improved exceptions in type support loading.
* Updated export of cmake variables.
* Contributors: Stefan Fabian

0.9.4 (2024-09-03)
------------------

0.9.1 (2024-09-02)
------------------
* Updated dependencies.
* Added missing test depend and configured ament_cmake_clang_format.
* Formatting.
* Renamed package to ros_babel_fish as requested in `ros/rosdistro#41540 <https://github.com/ros/rosdistro/issues/41540>`_
* Contributors: Stefan Fabian
