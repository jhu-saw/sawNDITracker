#
# (C) Copyright 2009-2018 Johns Hopkins University (JHU), All Rights Reserved.
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
find_package (cisst 1.0.9 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawNDITrackerExamples_IS_CATKIN_BUILT)
  if (sawNDITrackerExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawNDITracker has been compiled within cisst, we should find it automatically
  find_package (sawNDITracker)
  find_package (sawOpenIGTLink)

  if (sawNDITracker_FOUND AND sawOpenIGTLink_FOUND)

    # sawNDITracker configuration
    include_directories (${sawNDITracker_INCLUDE_DIR}
                         ${sawOpenIGTLink_INCLUDE_DIR})
    link_directories (${sawNDITracker_LIBRARY_DIR}
                      ${sawOpenIGTLink_LIBRARY_DIR})

    add_executable (sawNDITrackerIGTLQtExample mainQt.cpp)
    set_property (TARGET sawNDITrackerIGTLQtExample PROPERTY FOLDER "sawNDITracker")

    # link against non cisst libraries and saw components
    target_link_libraries (sawNDITrackerIGTLQtExample
                           ${sawNDITracker_LIBRARIES}
                           ${sawNDITrackerQt_LIBRARIES}
                           ${sawOpenIGTLink_LIBRARIES}
                           ${QT_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawNDITrackerIGTLQtExample ${REQUIRED_CISST_LIBRARIES})

  else (sawNDITracker_FOUND AND sawOpenIGTLink_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawNDITracker and sawOpenIGTLink")
  endif (sawNDITracker_FOUND AND sawOpenIGTLink_FOUND)

else (cisst_FOUND)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND)
