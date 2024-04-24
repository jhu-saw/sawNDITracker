#
# (C) Copyright 2011-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawNDITracker VERSION 1.1.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawNDITracker)
  set (sawNDITracker_INCLUDE_DIR
       "${sawNDITracker_SOURCE_DIR}/include"
       "${sawNDITracker_BINARY_DIR}/include")
  set (sawNDITracker_HEADER_DIR "${sawNDITracker_SOURCE_DIR}/include/sawNDITracker")
  set (sawNDITracker_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawNDITracker_LIBRARIES sawNDITracker)

  # Allow c++ code to find local header files
  include_directories (${sawNDITracker_INCLUDE_DIR} ${sawNDITracker_BINARY_DIR})

  # add Qt code
  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawNDITracker_LIBRARIES ${sawNDITracker_LIBRARIES} sawNDITrackerQt)
  endif (CISST_HAS_QT)

  # add all config files for this component
  cisst_add_config_files (sawNDITracker)

  add_library (sawNDITracker
               ${sawNDITracker_HEADER_DIR}/sawNDITrackerExport.h
               code/mtsNDISerial.cpp
               ${sawNDITracker_HEADER_DIR}/mtsNDISerial.h)
  set_target_properties (sawNDITracker PROPERTIES
                         VERSION ${sawNDITracker_VERSION}
                         FOLDER "sawNDITracker")
  cisst_target_link_libraries (sawNDITracker ${REQUIRED_CISST_LIBRARIES})

  configure_file ("${sawNDITracker_SOURCE_DIR}/code/sawNDITrackerConfig.h.in"
                  "${sawNDITracker_BINARY_DIR}/include/sawNDITracker/sawNDITrackerConfig.h")

  # Install target for headers and library
  install (DIRECTORY
           "${sawNDITracker_SOURCE_DIR}/include/sawNDITracker"
           "${sawNDITracker_BINARY_DIR}/include/sawNDITracker"
           DESTINATION include
           COMPONENT sawNDITracker-dev)

  install (TARGETS sawNDITracker COMPONENT sawNDITracker
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
