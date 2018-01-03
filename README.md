# sawNDITracker

This SAW component contains code for interfacing with many NDI (Northern Digital Inc, https://www.ndigital.com/) trackers. 
It compiles on Windows, Linux and likely MacOS.  It has been tested with:
  * Linux and Windows
  * NDI Polaris (old generation), Spectra and Vicra

The `ros` folder contains code for a ROS node that interfaces with the sawNDITracker component
and publishes the 3D transformations of each tracked tool as well as a point cloud for all stray 
markers.  It also broadcasts transformations for `tf2`.  To build, make sure you use `catkin build`.

If needed, one can also add OpenIGTLink support using sawOpenIGTLink (contact the sawNDITracker developer 
if you need help with this).

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * Linux only (requires `libraw1394`)
 * cisst libraries: https://github.com/jhu-cisst/cisst
 
# Running the examples
 
## Linux permissions
 
NDI communication is basically a serial port.  When connecting your tracker to your computer, a "device" will be added
to the `/dev` directory.   Usually something like `/dev/ttyS01`, `/dev/ttyUSB0` or `/dev/ttyACM0`.  
Check the file permissions on said device, e.g.
```sh
ls -al /dev/ttyUSB0 
crw-rw---- 1 root dialout 188, 0 Jan  3 09:32 /dev/ttyUSB0
```
On Ubuntu, the OS sets the ownership of `/dev/ttyUSB0` to `root` and the group to `dialout`.   To grant permissions to 
read and write to the device, use the command `sudo adduser <user_id> dialout`.   Please note that the user has to 
logout/login for the new group membership to take effect.
 
## Without ROS
 
The main example provided is `sawNDITrackerQtExample`.  The command line options are:
```sh
sawNDITrackerQtExample:
-j <value>, --json-config <value> : json configuration file (optional)
-s <value>, --serial-port <value> : serial port (e.g. /dev/ttyUSB0, COM...) (optional)
-l, --log-serial : log all serial port read/writes in cisstLog.txt (optional)
```

The JSON configuration file is optional.  It can be skipped if all your tools are active (i.e. wired to the tool control
unit).  If you use any passive tool (i.e. reflective markers for optical trackers), you will need a JSON configuration 
file.

 
