/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-11-28

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// cisst
#include "mtsNDISerialROS.h"

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsNDISerialROS, mtsTaskFromSignal, mtsTaskContinuousConstructorArg);

mtsNDISerialROS::mtsNDISerialROS(const std::string & componentName):
    mtsTaskFromSignal(componentName)
{
    Init();
}

mtsNDISerialROS::mtsNDISerialROS(const mtsTaskContinuousConstructorArg & argument):
    mtsTaskFromSignal(argument.Name)
{
    Init();
}
void mtsNDISerialROS::Init(void)
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        // ADV        interfaceRequired->AddFunction("Name", Tracker.Name);
        interfaceRequired->AddFunction("ToolNames", Tracker.ToolNames);
        interfaceRequired->AddEventHandlerWrite(&mtsNDISerialROS::ConnectedEventHandler,
                                                this, "Connected");
        interfaceRequired->AddEventHandlerVoid(&mtsNDISerialROS::UpdatedToolsEventHandler,
                                               this, "UpdatedTools");
    }
}

void mtsNDISerialROS::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsNDISerialROS::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
}

void mtsNDISerialROS::Run(void)
{
    ProcessQueuedEvents();
}

void mtsNDISerialROS::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsNDISerialROS::ConnectedEventHandler(const std::string & connected)
{
    if (connected != "") {
        UpdatedToolsEventHandler();
    }
}

void mtsNDISerialROS::UpdatedToolsEventHandler(void)
{
    std::string trackerName = "NDI";
        // ADV Tracker.Name(trackerName);

    std::vector<std::string> toolNames;
    Tracker.ToolNames(toolNames);

#if 0
    for (size_t i = 0; i < toolNames.size(); ++i) {
        std::string name = toolNames[i];
        Tool * tool;
        tool = Tools.GetItem(name);
        // if it's not found, it is new
        if (!tool) {
            tool = new Tool;
            // cisst interface
            tool->Interface = this->AddInterfaceRequired(name);
            tool->Interface->AddFunction("GetPositionCartesian", tool->GetPositionCartesian);
            mtsComponentManager * manager = mtsComponentManager::GetInstance();
            manager->Connect(trackerName, name,
                             this->GetName(), name);
            // Qt widget added to grid
            const int NB_COLS = 2;
            int position = static_cast<int>(Tools.size());
            int row = position / NB_COLS;
            int col = position % NB_COLS;
            std::cerr << name << " pos: " << position << " row " << row << " col " << col << std::endl;
            tool->Widget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
            QGTools->addWidget(new QLabel(name.c_str()), 2 * row, col);
            QGTools->addWidget(tool->Widget, 2 * row + 1, col);
            Tools.AddItem(name, tool);
        }
    }
#endif
}
