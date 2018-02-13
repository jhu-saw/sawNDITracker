#
# (C) Copyright 2009-2017 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.0.9 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawNDITrackerExamples_IS_CATKIN_BUILT)
  if (sawNDITrackerExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawNDITracker has been compiled within cisst, we should find it automatically
  find_package (sawNDITracker REQUIRED)

  if (sawNDITracker_FOUND)

    # sawNDITracker configuration
    include_directories (${sawNDITracker_INCLUDE_DIR})
    link_directories (${sawNDITracker_LIBRARY_DIR})

    add_executable (sawNDITrackerQtExample mainQt.cpp)
    set_property (TARGET sawNDITrackerQtExample PROPERTY FOLDER "sawNDITracker")

    # link against non cisst libraries and saw components
    target_link_libraries (sawNDITrackerQtExample
                           ${sawNDITracker_LIBRARIES}
                           ${sawNDITrackerQt_LIBRARIES}
                           ${QT_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawNDITrackerQtExample ${REQUIRED_CISST_LIBRARIES})

  endif (sawNDITracker_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
