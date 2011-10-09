#
# $Id$
#
# (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
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

  add_library (sawNDITracker
               ../sawNDITrackerExport.h
               mtsNDISerial.cpp
               ../mtsNDISerial.h)

  cisst_target_link_libraries (sawNDITracker ${REQUIRED_CISST_LIBRARIES})

  # Qt widgets
  if (CISST_HAS_QT)

    # this needs to be moved to cisstMacros.cmake
    MACRO (CISST_QT4_WRAP_UI outfiles)
      QT4_EXTRACT_OPTIONS (ui_files ui_options ${ARGN})
      FOREACH (it ${ui_files})
        GET_FILENAME_COMPONENT (outfile ${it} NAME_WE)
        GET_FILENAME_COMPONENT (infile ${it} ABSOLUTE)
        # the only thing we modified ...
        SET (outfile ${sawComponents_BINARY_DIR}/sawNDITracker/${outfile}.h)
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
                  ../mtsNDISerialControllerQtComponent.h
                  ../mtsNDISerialToolQtComponent.h)
    cisst_qt4_wrap_ui (SAW_NDISERIAL_QT_WIDGETS_GENERATED
                       mtsNDISerialControllerQtWidget.ui
                       mtsNDISerialToolQtWidget.ui)

    add_library (sawNDITrackerQt
                 ${SAW_NDISERIAL_QT_COMPONENTS_GENERATED}
                 ${SAW_NDISERIAL_QT_WIDGETS_GENERATED}
                 ../sawNDITrackerExportQt.h
                 mtsNDISerialControllerQtComponent.cpp
                 ../mtsNDISerialControllerQtComponent.h
                 mtsNDISerialToolQtComponent.cpp
                 ../mtsNDISerialToolQtComponent.h)

    target_link_libraries (sawNDITrackerQt ${QT_LIBRARIES})
    cisst_target_link_libraries (sawNDITrackerQt ${REQUIRED_CISST_LIBRARIES} cisstQt)
  endif (CISST_HAS_QT)

  # create/configure file for find_package (sawNDITracker)
  set (sawNDITracker_INCLUDE_DIR "${sawComponents_SOURCE_DIR}" "${sawComponents_BINARY_DIR}")
  set (sawNDITracker_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawNDITracker_LIBRARIES sawNDITracker)
  set (sawNDITrackerQt_LIBRARIES sawNDITrackerQt)

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawNDITrackerConfig.cmake.in"
                  "${sawComponents_BINARY_DIR}/sawNDITrackerConfig.cmake"
                   @ONLY@)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
