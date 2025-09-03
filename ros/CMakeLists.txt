#
# (C) Copyright 2009-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.10)
project (ndi_tracker VERSION 1.1.0)

# first test for ROS1
set (ROS1_depends cisst_ros_crtk roscpp roslib)

find_package (catkin QUIET
  COMPONENTS
  ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  catkin_package (CATKIN_DEPENDS ${ROS1_depends})
  include_directories (${catkin_INCLUDE_DIRS})
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    set (ROS2_depends rclcpp
      std_msgs std_srvs
      geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs
      cisst_msgs crtk_msgs
      cisst_ros_bridge cisst_ros_crtk
      )
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonQt
  cisstVector
  cisstNumerical
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstParameterTypesQt
  cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  # sawNDITracker has been compiled within cisst, we should find it automatically
  find_package (sawNDITracker REQUIRED)

  if (sawNDITracker_FOUND)

    # sawNDITracker configuration
    include_directories (${sawNDITracker_INCLUDE_DIR} ${ndi_tracker_SOURCE_DIR})
    link_directories (${sawNDITracker_LIBRARY_DIR})

    add_executable (ndi_tracker
      mts_ros_crtk_ndi_bridge.cpp
      mts_ros_crtk_ndi_bridge.h
      ndi_tracker.cpp)

    # link against non cisst libraries and saw components
    target_link_libraries (ndi_tracker
      ${sawNDITracker_LIBRARIES}
      ${sawNDITrackerQt_LIBRARIES}
      ${catkin_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (ndi_tracker ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS ndi_tracker
      LIBRARY DESTINATION lib/${PROJECT_NAME}
      RUNTIME DESTINATION lib/${PROJECT_NAME}
      LIBRARY DESTINATION lib
      INCLUDES DESTINATION include)

    if (ROS_VERSION EQUAL 2)
      ament_target_dependencies (
        ndi_tracker
        ${ROS2_depends}
        )
      ament_package ()
    endif ()

  endif (sawNDITracker_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
