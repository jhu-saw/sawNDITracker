#
# (C) Copyright 2009-2020 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

project (ndi_tracker_ros)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_crtk
              roscpp
              roslib
              )

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes
                              cisstParameterTypesQt
			      cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.0.9 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)

  # sawNDITracker has been compiled within cisst, we should find it automatically
  find_package (sawNDITracker REQUIRED)

  if (sawNDITracker_FOUND)

    # sawNDITracker configuration
    include_directories (${sawNDITracker_INCLUDE_DIR} ${ndi_tracker_ros_SOURCE_DIR})
    link_directories (${sawNDITracker_LIBRARY_DIR})

    add_library (ros_crtk_ndi_bridge
      mts_ros_crtk_ndi_bridge.cpp
      mts_ros_crtk_ndi_bridge.h)

    add_executable (ndi_tracker mainQt.cpp)

    # link against non cisst libraries and saw components
    target_link_libraries (ndi_tracker
      ros_crtk_ndi_bridge
      ${sawNDITracker_LIBRARIES}
      ${sawNDITrackerQt_LIBRARIES}
      ${catkin_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (ndi_tracker ${REQUIRED_CISST_LIBRARIES})

  endif (sawNDITracker_FOUND)

endif (cisst_FOUND)
