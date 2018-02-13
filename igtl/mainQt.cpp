/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2018-01-07

  (C) Copyright 2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>

#include <cisstMultiTask/mtsTaskManager.h>

#include <sawNDITracker/mtsNDISerial.h>
#include <sawNDITracker/mtsNDISerialControllerQtWidget.h>

#include <sawOpenIGTLink/mtsOpenIGTLinkBridge.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // parse options
    cmnCommandLineOptions options;
    std::string port;
    std::string configFile;
    unsigned int igtlPort;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &configFile);

    options.AddOptionOneValue("s", "serial-port",
                              "serial port (e.g. /dev/ttyUSB0, COM...)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &port);

    options.AddOptionNoValue("l", "log-serial",
                             "log all serial port read/writes in cisstLog.txt");

    options.AddOptionOneValue("i", "igtl-port",
                              "port used for OpenIGTLink server",
                              cmnCommandLineOptions::REQUIRED_OPTION, &igtlPort);

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

    // create the components
    mtsNDISerial * tracker = new mtsNDISerial("NDI", 50.0 * cmn_ms);
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

    // OpenIGTLink
    mtsOpenIGTLinkBridge * igtl = new mtsOpenIGTLinkBridge("NDI-igtl", 50.0 * cmn_ms);
    componentManager->AddComponent(igtl);

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // create a main window to hold QWidgets
    QMainWindow * mainWindow = new QMainWindow();
    mainWindow->setCentralWidget(trackerWidget);
    mainWindow->setWindowTitle("sawNDITracker");
    mainWindow->show();

    // run Qt user interface
    application.exec();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
