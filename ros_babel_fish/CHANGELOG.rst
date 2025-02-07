^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_babel_fish
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.25.2 (2025-02-07)
-------------------
* Fixed FixedLengthArray assigns in CompoundMessages (`#11 <https://github.com/LOEWE-emergenCITY/ros_babel_fish/issues/11>`_).
* Use no declaration instead of static assert for compilers that evaluate it even when not used.
* Added a method to get the actual underlying message from a compound message.
  Usage:
  using geometry_msgs::msg::Point;
  Point point = compound["position"].as<CompoundMessage>().message<Point>();
* Replaced action_tutorials_interfaces by example_interfaces.
* Contributors: Stefan Fabian

0.10.3 (2024-12-03)
-------------------
* Fixes service server segfaulting (`#10 <https://github.com/LOEWE-emergenCITY/ros_babel_fish/issues/10>`_) and adds a new test to cover this.
* Added convenience methods to get and set values of compound messages.
* Updated export of cmake variables.
* Contributors: Stefan Fabian

0.10.0 (2024-10-25)
-------------------
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
* Contributors: Stefan Fabian

0.9.3 (2024-08-16)
------------------
* Updated dependencies.
  Removed test node. Examples serve the purpose.
* Contributors: Stefan Fabian

0.9.2 (2024-08-14)
------------------
* Initial release.
* Contributors: Stefan Fabian
