Change log
==========

1.0.0 (2018-05-16)
==================

* API changes:
  * Use different commands to connect
  * Configuration file is now JSON based, not XML
* Deprecated features:
  * None
* New features:
  * Dynamically create tools and emits events when new tools are discovered
  * Added new Qt widget with timestamp/valid and 3D visualization (from cisstParameterTypesQt)
  * Added logs for Qt widgets and ROS
  * Separated build of component, example and ROS node
  * ROS/catkin build compatible
  * Added ROS node with publishers configuration (list of tools), position of tools and tf2
  * Added "reference frame" in configuration file
  * Added README.md
  * Added path to locate configuration files and ROM files
  * Updated Python based example to handle new tools, python3 compatible
  * Tested on older Polaris as well as more recent Spectra, Vicra and Aurora
* Bug fixes:
  * None
