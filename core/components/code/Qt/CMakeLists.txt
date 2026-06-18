#
# (C) Copyright 2014-2026 Johns Hopkins University (JHU), All Rights Reserved.
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
find_package (cisst REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# for Qt generated header files
include_directories (${CMAKE_CURRENT_BINARY_DIR})
set (CMAKE_AUTOMOC ON)

add_library (sawNDITrackerQt
             ${sawNDITracker_HEADER_DIR}/mtsNDISerialControllerQtWidget.h
             mtsNDISerialControllerQtWidget.cpp)
set_target_properties (sawNDITrackerQt PROPERTIES
                       VERSION ${sawNDITracker_VERSION}
                       FOLDER "sawNDITracker")
target_link_libraries (sawNDITrackerQt ${cisst_LIBRARIES})

install (TARGETS sawNDITrackerQt COMPONENT sawNDITracker-Qt
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)
