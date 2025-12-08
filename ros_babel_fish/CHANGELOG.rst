^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_babel_fish
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.25.120 (2025-12-08)
---------------------
* Wrap exceptions when creating service or action client with invalid topic as BabelFishException.
* Fixed ament package not found error not being caught and wrapped as TypeSupportException / BabelFishException.
* Added a get method for CompoundArrayMessage to obtain element as shared_ptr.
* Added documentation and fixed tiny memory leak.
* Added convenience methods to message introspection wrapper.
* Fixed waiting indefinitely if topic is namespaced and added test case.
* Print warning when waiting for more than 3 seconds for topic.
* Contributors: Stefan Fabian

0.25.2 (2025-02-07)
-------------------
* Updated docs on template call array bounded / fixed length.
* Fixed FixedLengthArray assigns in CompoundMessages (`#11 <https://github.com/LOEWE-emergenCITY/ros_babel_fish/issues/11>`_).
* Use no declaration instead of static assert for compilers that evaluate it even when not used.
* Added a method to get the actual underlying message from a compound message.
  Usage:
  using geometry_msgs::msg::Point;
  Point point = compound["position"].as<CompoundMessage>().message<Point>();
* Contributors: Stefan Fabian

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
