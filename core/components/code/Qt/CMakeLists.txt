#
# (C) Copyright 2014-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt)

# cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # cisst
  include (${CISST_USE_FILE})

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  if (CISST_HAS_QT4)
    qt4_wrap_cpp (sawNDITracker_QT_WRAP_CPP
                  ${sawNDITracker_HEADER_DIR}/mtsNDISerialControllerQtWidget.h)
  else (CISST_HAS_QT4)
    set (CMAKE_AUTOMOC ON)
  endif (CISST_HAS_QT4)

  add_library (sawNDITrackerQt
               ${sawNDITracker_HEADER_DIR}/mtsNDISerialControllerQtWidget.h
               mtsNDISerialControllerQtWidget.cpp
               ${sawNDITracker_QT_WRAP_CPP})
  set_target_properties (sawNDITrackerQt PROPERTIES
                         VERSION ${sawNDITracker_VERSION}
                         FOLDER "sawNDITracker")
  cisst_target_link_libraries (sawNDITrackerQt ${REQUIRED_CISST_LIBRARIES})

  install (TARGETS sawNDITrackerQt COMPONENT sawNDITracker-Qt
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
