^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_babel_fish_test_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.25.11 (2025-11-11)
--------------------

3.25.2 (2025-02-07)
-------------------
* Refactored array size templating and improved compile time checks by moving checks to constexpr.
  Fixes (`#11 <https://github.com/LOEWE-emergenCITY/ros_babel_fish/issues/11>`_).
  This reduces the potential for errors and allows easier compile time checking.
  Many checks are also moved to constexpr and static assertions catching common errors at compile time instead of run time.
* Contributors: Stefan Fabian

0.10.2 (2024-12-03)
-------------------

0.10.1 (2024-10-25)
-------------------
* Updated test message and added goal rejected and cancel rejected tests for client.
* Contributors: Stefan Fabian

0.9.5 (2024-10-11)
------------------

0.9.3 (2024-08-16)
------------------
* Removed unneeded dependency.
* Contributors: Stefan Fabian

0.9.2 (2024-08-14)
------------------
* Initial release.
* Contributors: Stefan Fabian
