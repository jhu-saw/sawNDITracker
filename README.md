# sawNDITracker

This SAW component contains code for interfacing with many NDI tracking devices (Northern Digital Inc, https://www.ndigital.com/).  It compiles on Windows, Linux and likely MacOS.  It has been tested with:
  * Linux and Windows
  * NDI Polaris (old generation), Spectra and Vicra

The `ros` folder contains code for a ROS node that interfaces with the sawNDITracker component and publishes the 3D transformations of each tracked tool as well as a point cloud for all stray markers.  It also broadcasts transformations for `tf2`.  To build the ROS node, make sure you use `catkin build`.

If needed, one can also add OpenIGTLink support using sawOpenIGTLink (contact the sawNDITracker developers if you need help with this).

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Qt for user interface
 * ROS (optional)
 
# Running the examples
 
## Linux permissions
 
NDI trackers use a serial port to communicate.  When connecting your tracker to your computer, a pseudo device will be added to the `/dev` directory.   Usually something like `/dev/ttyS01`, `/dev/ttyUSB0` or `/dev/ttyACM0`.  Using the command `dmesg` can help identify which device is used.  Check the file permissions on said device, e.g.,
```sh
ls -al /dev/ttyUSB0 
crw-rw---- 1 root dialout 188, 0 Jan  3 09:32 /dev/ttyUSB0
```
On Ubuntu, the OS usually sets the ownership of `/dev/ttyUSB0` to `root` and the group to `dialout`.   To grant permissions to read and write to the device, use the command `sudo adduser <user_id> dialout` to add users to the `dialout` group.   Please note that the user has to logout/login for the new group membership to take effect.
 
## Main example
 
The main example provided is `sawNDITrackerQtExample`.  The command line options are:
```sh
sawNDITrackerQtExample:
-j <value>, --json-config <value> : json configuration file (optional)
-s <value>, --serial-port <value> : serial port (e.g. /dev/ttyUSB0, COM...) (optional)
-l, --log-serial : log all serial port read/writes in cisstLog.txt (optional)
```

The JSON configuration file is optional.  It can be skipped if all your tools are active (i.e., If you need a more human readable name for your active tools, you'll need a JSON configuration files.  If you use any passive tool (i.e., reflective markers for optical trackers), you will need a JSON configuration file.

Some examples of configuration files can be found in the `share` directory.  Here is an example for an active tool and a passive tool used on an older Polaris:
```json
{
    // serial port is optional, if already defined (e.g. command line
    // argument), this will be ignored.  The Connect method will try
    // to automatically find the serial port using a regular expression
    "serial-port": "/dev/ttyUSB0",

    // definition path is a list of directories used to find tool
    // definition files (in order defined in this file).  By default,
    // the search path include the current working directory as well
    // as the source directory with suffix "share/roms" at the tail.
    "definition-path": ["/a_directory", "/another_directory"],

    // list of tools to be tracked
    "tools": [
        {
            // active tool
            "name": "Base",
            "unique-id": "01-3288C807-8700223"
        }
	,
        {
            // passive tool, must be defined after Base since it uses Base as reference frame
            "name": "Pointer",
            "unique-id": "01-34801403-8700339",
            "definition": "8700339.rom", // this is a passive tool, the definition has to be provided
            "reference" : "Base"
        }
    ]
}
```

The tool name will be used to create the cisstMultiTask required interface (see https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts), Qt Widgets and ROS topics (see example below).

When starting the example, the GUI will just show the controller view:
![GUI controller view](doc/gui-on-start.png "GUI on start, controller view")

The first step is to connect to the device.   If the device is found, the tool widgets will appear.  The number of tools and names used is based on the content of the configuration file.
![GUI with tools](doc/gui-after-connect.png "GUI after connection, tools widgets should appear")

Then you can start tracking:
![GUI tracking](doc/gui-tracking.png "GUI tracking, when visible the timestamp should turn green")

If you unplug/replug an active tool, you might have to hit the `(Re)initialize` button and then turn tracking back on.

## Python

Since `cisstMultiTask` has a nice Python interface it is possible to use the `sawNDITracker` from Python.  Make sure you compiled `cisst` with the CMake option `CISST_HAS_SWIG_PYTHON` and you've sourced `cisstvars.sh` so you can find the `cisstMultiTaskPython` module.

```sh
cisst-saw/sawNDITracker/examples$ ./mainPython.py --port /dev/ttyUSB0 --json ../share/ndi-active-tools.json
```

## ROS

Please read the section above for the configuration file description.  The ROS node is `ndi_tracker` and can be found in the package `ndi_tracker_ros`:
```sh
roscd ndi_tracker_ros
rosrun ndi_tracker_ros ndi_tracker -j ../share/ndi-active-tools.json 
```

The ROS node has a few more command line options:
```sh
/home/adeguet1/catkin_ws/devel_debug/lib/ndi_tracker_ros/ndi_tracker:
 -j <value>, --json-config <value> : json configuration file (optional)
 -s <value>, --serial-port <value> : serial port (e.g. /dev/ttyUSB0, COM...) (optional)
 -l, --log-serial : log all serial port read/writes in cisstLog.txt (optional)
 -n <value>, --ros-namespace <value> : ROS namespace to prefix all topics, must have start and end "/" (default /ndi/) (optional)
 -p <value>, --ros-period <value> : period in seconds to read all components and publish (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the tracker's period (optional)
 -P <value>, --tf-ros-period <value> : period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the tracker's period (optional)
```

Once the node is started AND connected, the following ROS topics should appear:
```sh
/ndi/Base/position_cartesian_current
/ndi/Pointer/position_cartesian_current
/ndi/connect
/ndi/connected
/ndi/fiducials
```

The topic names for the tools are based on the names in the configuration file or the unique ID if there is no configuration file specified.

You can also visualize the tf2 output using:
```sh
rosrun tf2_tools view_frames.py
evince frames.pdf 
```

In our example, the `Base` is defined with respect to the `Camera` and the `Pointer` is defined with respect to the `Base`:
![tf2](doc/frames.png "tf2")

# Notes

## Units

By default NDI API reports distances in millimeters.   The `cisst` libraries used to implicitly rely on millimeters and grams but can now be configured to report distances in (preferred) SI units (i.e., meter, kg, N).   This setting can be changed in CMake while configuring `cisst` by setting the variable `CISST_USE_SI`.   The default is `CISST_USE_SI` is `0` (false) except when compiling with ROS catkin build tools (the default with ROS for `CISST_USE_SI` is `1`).   In your code, you can include `cisstConfig.h` and use the preprocessor variable `CISST_USE_SI` to handle both cases.   When running your program, you can also look at the `cisstLog.txt` file, it will contain a line indicating if `CISST_USE_SI` is true or false.
