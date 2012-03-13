#
# $Id$
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# Core library
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonXML
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # create/configure file for find_package (sawNDITracker)
  set (sawNDITracker_INCLUDE_DIR "${sawNDITracker_SOURCE_DIR}/include" "${sawNDITracker_BINARY_DIR}")
  set (sawNDITracker_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawNDITracker_LIBRARIES sawNDITracker)
  set (sawNDITrackerQt_LIBRARIES sawNDITrackerQt)

  include_directories (${sawNDITracker_INCLUDE_DIR} ${sawNDITracker_BINARY_DIR})
  set (sawNDITracker_HEADER_DIR "${sawNDITracker_SOURCE_DIR}/include/sawNDITracker")

  add_library (sawNDITracker
               ${sawNDITracker_HEADER_DIR}/sawNDITrackerExport.h
               mtsNDISerial.cpp
               ${sawNDITracker_HEADER_DIR}/mtsNDISerial.h)
  set_property (TARGET sawNDITracker PROPERTY FOLDER "sawNDITracker")
  cisst_target_link_libraries (sawNDITracker ${REQUIRED_CISST_LIBRARIES})

  # Qt widgets
  if (CISST_HAS_QT)

    file (MAKE_DIRECTORY "${sawNDITracker_BINARY_DIR}/sawNDITracker")

    # this needs to be moved to cisstMacros.cmake
    MACRO (CISST_QT4_WRAP_UI outfiles)
      QT4_EXTRACT_OPTIONS (ui_files ui_options ${ARGN})
      FOREACH (it ${ui_files})
        GET_FILENAME_COMPONENT (outfile ${it} NAME_WE)
        GET_FILENAME_COMPONENT (infile ${it} ABSOLUTE)
        # the only thing we modified ...
        SET (outfile ${sawNDITracker_BINARY_DIR}/sawNDITracker/${outfile}.h)
        ADD_CUSTOM_COMMAND (OUTPUT ${outfile}
                            COMMAND ${QT_UIC_EXECUTABLE}
                            ARGS ${ui_options} -o ${outfile} ${infile}
                            MAIN_DEPENDENCY ${infile})
        SET (${outfiles} ${${outfiles}} ${outfile})
      ENDFOREACH (it)
    ENDMACRO (CISST_QT4_WRAP_UI)

    # initial find_package doesn't have cisstQt
    cisst_set_directories (cisstQt)

    # for Qt generated header files
    include_directories (${CMAKE_CURRENT_BINARY_DIR})
    qt4_wrap_cpp (SAW_NDISERIAL_QT_COMPONENTS_GENERATED
                  ${sawNDITracker_HEADER_DIR}/mtsNDISerialControllerQtComponent.h
                  ${sawNDITracker_HEADER_DIR}/mtsNDISerialToolQtComponent.h)
    cisst_qt4_wrap_ui (SAW_NDISERIAL_QT_WIDGETS_GENERATED
                       mtsNDISerialControllerQtWidget.ui
                       mtsNDISerialToolQtWidget.ui)

    add_library (sawNDITrackerQt
                 ${SAW_NDISERIAL_QT_COMPONENTS_GENERATED}
                 ${SAW_NDISERIAL_QT_WIDGETS_GENERATED}
                 ${sawNDITracker_HEADER_DIR}/sawNDITrackerExportQt.h
                 mtsNDISerialControllerQtComponent.cpp
                 ${sawNDITracker_HEADER_DIR}/mtsNDISerialControllerQtComponent.h
                 mtsNDISerialToolQtComponent.cpp
                 ${sawNDITracker_HEADER_DIR}/mtsNDISerialToolQtComponent.h)
    set_property (TARGET sawNDITrackerQt PROPERTY FOLDER "sawNDITracker")
    target_link_libraries (sawNDITrackerQt ${QT_LIBRARIES})
    cisst_target_link_libraries (sawNDITrackerQt ${REQUIRED_CISST_LIBRARIES} cisstQt)
  endif (CISST_HAS_QT)

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawNDITrackerConfig.cmake.in"
                  "${sawNDITracker_BINARY_DIR}/sawNDITrackerConfig.cmake"
                   @ONLY@)

  # Install target for headers and library
  install (DIRECTORY
           "${sawNDITracker_SOURCE_DIR}/include/sawNDITracker"
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
