/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-12-04

  (C) Copyright 2017-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*!
  \file
  \brief An example interface for NDI trackers with serial interface.
  \ingroup devicesTutorial
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidgetFactory.h>
#include <sawNDITracker/mtsNDISerial.h>
#include <sawNDITracker/mtsNDISerialControllerQtWidget.h>

#include <mts_ros_crtk_ndi_bridge.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsNDISerial*", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    ros::init(argc, argv, "dvrk", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;

    // parse options
    cmnCommandLineOptions options;
    std::string port;
    std::string configFile;
    double rosPeriod = 20.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &configFile);

    options.AddOptionOneValue("s", "serial-port",
                              "serial port (e.g. /dev/ttyUSB0, COM...)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &port);

    options.AddOptionNoValue("l", "log-serial",
                             "log all serial port read/writes in cisstLog.txt");

    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all components and publish (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the tracker's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);

    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the tracker's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);

    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    typedef std::list<std::string> managerConfigType;
    managerConfigType managerConfig;
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON file to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // log configuration
    if (options.IsSet("log-serial")) {
        std::cout << "Adding log for all serial port read/writes" << std::endl;
        cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
        cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
        cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
        cmnLogger::SetMaskClassMatching("mtsNDISerial", CMN_LOG_ALLOW_ALL);
        cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    }

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // create the components
    mtsNDISerial * tracker = new mtsNDISerial("NDI", 10.0 * cmn_ms);
    if (port != "") {
        tracker->SetSerialPort(port);
    }

    // configure the components
    std::string configPath = "";
    // if there's a config file passed as argument, try to locate it
    if (configFile != "") {
        if (cmnPath::Exists(configFile)) {
            configPath = configFile;
        } else {
            // search in current working directory and source tree
            cmnPath searchPath;
            searchPath.Add(cmnPath::GetWorkingDirectory());
            searchPath.Add(std::string(sawNDITracker_SOURCE_DIR) + "/../share", cmnPath::TAIL);
            configPath = searchPath.Find(configFile);
            // if still empty
            if (configPath.empty()) {
                std::cerr << "Failed to find configuration file \"" << configFile << "\"" << std::endl
                          << "Searched in: " << configPath << std::endl;
                return 1;
            }
        }
    }
    // configure
    tracker->Configure(configPath);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(tracker);

    // Qt widget
    mtsNDISerialControllerQtWidget * trackerWidget = new mtsNDISerialControllerQtWidget("NDI Widget");
    componentManager->AddComponent(trackerWidget);
    componentManager->Connect(trackerWidget->GetName(), "Controller",
                              tracker->GetName(), "Controller");

    tabWidget->addTab(trackerWidget, "Controller");

    // tool position widgets
    prmPositionCartesianGetQtWidgetFactory * positionQtWidgetFactory
        = new prmPositionCartesianGetQtWidgetFactory("positionQtWidgetFactory");
    positionQtWidgetFactory->AddFactorySource(tracker->GetName(), "Controller");
    componentManager->AddComponent(positionQtWidgetFactory);
    positionQtWidgetFactory->Connect();
    tabWidget->addTab(positionQtWidgetFactory, "Tools");

    // ROS CRTK bridge, using the derived class for NDi
    mts_ros_crtk_ndi_bridge * crtk_bridge
        = new mts_ros_crtk_ndi_bridge("ndi_serial_crtk_bridge", &rosNodeHandle);
    crtk_bridge->bridge(tracker->GetName(), "Controller",
                        "controller",
                        rosPeriod, tfPeriod);
    crtk_bridge->add_factory_source(tracker->GetName(), "Controller",
                                    rosPeriod, tfPeriod);
    componentManager->AddComponent(crtk_bridge);
    crtk_bridge->Connect();

    // custom user component
    const managerConfigType::iterator end = managerConfig.end();
    for (managerConfigType::iterator iter = managerConfig.begin();
         iter != end;
         ++iter) {
        if (!iter->empty()) {
            if (!cmnPath::Exists(*iter)) {
                CMN_LOG_INIT_ERROR << "File " << *iter
                                   << " not found!" << std::endl;
            } else {
                if (!componentManager->ConfigureJSON(*iter)) {
                    CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager" << std::endl;
                    return -1;
                }
            }
        }
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    ros::shutdown();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
