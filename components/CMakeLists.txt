#
# (C) Copyright 2011-2016 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawNDITracker)

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonXML
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawNDITracker_IS_CATKIN_BUILT)
  if (sawNDITracker_IS_CATKIN_BUILT)
    set (sawNDITracker_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawNDITracker/cmake")
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
    set (LIBRARY_OUTPUT_PATH    "${CATKIN_DEVEL_PREFIX}/lib")
  else ()
    set (sawNDITracker_CONFIG_FILE_DIR "${sawNDITracker_BINARY_DIR}")
  endif ()

  # create/configure file for find_package (sawNDITracker)
  set (sawNDITracker_INCLUDE_DIR
       "${sawNDITracker_SOURCE_DIR}/include"
       "${sawNDITracker_BINARY_DIR}/include")
  set (sawNDITracker_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawNDITracker_LIBRARIES sawNDITracker)
  set (sawNDITrackerQt_LIBRARIES sawNDITrackerQt)

  # Set the version number
  set (sawNDITracker_VERSION_MAJOR "1")
  set (sawNDITracker_VERSION_MINOR "0")
  set (sawNDITracker_VERSION_PATCH "0")
  set (sawNDITracker_VERSION "${sawNDITracker_VERSION_MAJOR}.${sawNDITracker_VERSION_MINOR}.${sawNDITracker_VERSION_PATCH}")

  # Generate sawNDITrackerRevision.h
  configure_file ("${sawNDITracker_SOURCE_DIR}/code/sawNDITrackerRevision.h.in"
                  "${sawNDITracker_BINARY_DIR}/include/sawNDITracker/sawNDITrackerRevision.h")

  include_directories (${sawNDITracker_INCLUDE_DIR} ${sawNDITracker_BINARY_DIR})
  set (sawNDITracker_HEADER_DIR "${sawNDITracker_SOURCE_DIR}/include/sawNDITracker")

  add_library (sawNDITracker
               ${sawNDITracker_HEADER_DIR}/sawNDITrackerExport.h
               code/mtsNDISerial.cpp
               ${sawNDITracker_HEADER_DIR}/mtsNDISerial.h)
  set_property (TARGET sawNDITracker PROPERTY FOLDER "sawNDITracker")
  cisst_target_link_libraries (sawNDITracker ${REQUIRED_CISST_LIBRARIES})

  # add Qt code
  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawNDITracker_LIBRARIES ${sawNDITracker_LIBRARIES} ${sawNDITrackerQt_LIBRARIES})
  endif (CISST_HAS_QT)

  set (sawNDITracker_CMAKE_CONFIG_FILE
       "${sawNDITracker_CONFIG_FILE_DIR}/sawNDITrackerConfig.cmake")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawNDITrackerConfig.cmake.in"
                  "${sawNDITracker_CMAKE_CONFIG_FILE}"
                  @ONLY)

  configure_file ("${sawNDITracker_SOURCE_DIR}/code/sawNDITrackerConfig.h.in"
                  "${sawNDITracker_BINARY_DIR}/include/sawNDITracker/sawNDITrackerConfig.h")

  # Install target for headers and library
  install (DIRECTORY
           "${sawNDITracker_SOURCE_DIR}/include/sawNDITracker"
           "${sawNDITracker_BINARY_DIR}/include/sawNDITracker"
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawNDITracker
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawNDITracker_BINARY_DIR}/sawNDITrackerConfig.cmake"
           DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
