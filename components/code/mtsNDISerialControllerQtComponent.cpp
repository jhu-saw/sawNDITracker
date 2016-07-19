/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Ali Uneri
  Created on: 2009-10-29

  (C) Copyright 2009-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawNDITracker/mtsNDISerialControllerQtComponent.h>
#include <ui_mtsNDISerialControllerQtWidget.h>

CMN_IMPLEMENT_SERVICES(mtsNDISerialControllerQtComponent);


mtsNDISerialControllerQtComponent::mtsNDISerialControllerQtComponent(const std::string & taskName) :
    mtsComponent(taskName)
{
    ControllerWidget = new Ui::mtsNDISerialControllerQtWidget();
    ControllerWidget->setupUi(&CentralWidget);
    CentralWidget.setWindowTitle(QString::fromStdString(taskName));
    Timer = new QTimer(this);

    mtsInterfaceRequired * required = AddInterfaceRequired("Controller");
    if (required) {
        required->AddFunction("Beep", NDI.Beep);
        required->AddFunction("PortHandlesInitialize", NDI.Initialize);
        required->AddFunction("PortHandlesQuery", NDI.Query);
        required->AddFunction("PortHandlesEnable", NDI.Enable);
        required->AddFunction("CalibratePivot", NDI.CalibratePivot);
        required->AddFunction("ToggleTracking", NDI.Track);
        required->AddFunction("ReportStrayMarkers", NDI.ReportStrayMarkers);
    }

    required = AddInterfaceRequired("DataCollector");
    if (required) {
        required->AddFunction("StartCollection", Collector.Start);
        required->AddFunction("StopCollection", Collector.Stop);
    }

    // connect Qt signals to slots
    QObject::connect(ControllerWidget->ButtonBeep, SIGNAL(clicked()),
                     this, SLOT(NDIBeepQSlot()));
    QObject::connect(ControllerWidget->ButtonInitialize, SIGNAL(clicked()),
                     this, SLOT(NDIInitializeQSlot()));
    QObject::connect(ControllerWidget->ButtonCalibratePivot, SIGNAL(clicked()),
                     this, SLOT(NDICalibratePivotQSlot()));
    QObject::connect(ControllerWidget->ButtonTrack, SIGNAL(toggled(bool)),
                     this, SLOT(NDITrackQSlot(bool)));
    QObject::connect(ControllerWidget->ButtonReportStrayMarkers, SIGNAL(clicked()),
                     this, SLOT(NDIReportStrayMarkersQSlot()));
    QObject::connect(ControllerWidget->ButtonRecord, SIGNAL(toggled(bool)),
                     this, SLOT(RecordQSlot(bool)));
}


void mtsNDISerialControllerQtComponent::AddTool(QObject * toolQtComponent, QWidget * toolQtWidget)
{
    ControllerWidget->LayoutTools->addWidget(toolQtWidget);
    ControllerWidget->BoxTools->addItem(toolQtWidget->windowTitle());
    QObject::connect(this->Timer, SIGNAL(timeout()),
                     toolQtComponent, SLOT(UpdatePositionCartesian()));
}


void mtsNDISerialControllerQtComponent::NDIBeepQSlot(void)
{
    mtsInt numberOfBeeps = ControllerWidget->NumberOfBeeps->value();
    NDI.Beep(numberOfBeeps);
}


void mtsNDISerialControllerQtComponent::NDIInitializeQSlot(void)
{
    NDI.Initialize();
    NDI.Query();
    NDI.Enable();
}


void mtsNDISerialControllerQtComponent::NDICalibratePivotQSlot(void)
{
    mtsStdString toolName = ControllerWidget->BoxTools->currentText().toStdString();
    NDI.CalibratePivot(toolName);
}


void mtsNDISerialControllerQtComponent::NDITrackQSlot(bool toggled)
{
    NDI.Track(mtsBool(toggled));
    if (toggled) {
        Timer->start(20);
    } else {
        Timer->stop();
    }
}


void mtsNDISerialControllerQtComponent::NDIReportStrayMarkersQSlot(void)
{
    NDI.ReportStrayMarkers();
}


void mtsNDISerialControllerQtComponent::RecordQSlot(bool toggled)
{
    if (toggled) {
        Collector.Start();
    } else {
        Collector.Stop();
    }
}
