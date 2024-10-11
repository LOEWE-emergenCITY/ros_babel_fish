^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_babel_fish
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.5 (2024-10-11)
------------------
* Fixes to compile on Rolling. (`#8 <https://github.com/LOEWE-emergenCITY/ros_babel_fish/issues/8>`_)
  * Fixes to compile on Rolling.
  ROS 2 Rolling has made two changes that cause this package
  to not build in its current form:
  1. The TRACEPOINT macro has been renamed to TRACETOOLS_TRACEPOINT.
  2. The action_tutorials_interface package has been removed,
  since it was duplicating an action that was already available
  in example_interfaces.
  This commit fixes both of these issues.
  ---------
  Co-authored-by: Stefan Fabian <fabian@sim.tu-darmstadt.de>
* Contributors: Chris Lalancette

0.9.3 (2024-08-16)
------------------
* Updated dependencies.
  Removed test node. Examples serve the purpose.
* Contributors: Stefan Fabian

0.9.2 (2024-08-14)
------------------
* Initial release.
* Contributors: Stefan Fabian
