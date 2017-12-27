/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>
#include <QMessageBox>

// cisst
#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <sawNDITracker/mtsNDISerialControllerQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsNDISerialControllerQtWidget, mtsComponent, std::string);

mtsNDISerialControllerQtWidget::mtsNDISerialControllerQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    QMMessage = new mtsMessageQtWidget();

    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        QMMessage->SetInterfaceRequired(interfaceRequired);
        interfaceRequired->AddFunction("GetPeriodStatistics", Tracker.GetPeriodStatistics);
        interfaceRequired->AddFunction("Connect", Tracker.Connect);
        interfaceRequired->AddFunction("Disconnect", Tracker.Disconnect);
        interfaceRequired->AddFunction("InitializeAll", Tracker.InitializeAll);
        // ADV        interfaceRequired->AddFunction("Name", Tracker.Name);
        interfaceRequired->AddFunction("ToolNames", Tracker.ToolNames);
        interfaceRequired->AddFunction("ToggleTracking", Tracker.Track);
        interfaceRequired->AddFunction("Beep", Tracker.Beep);
        interfaceRequired->AddEventHandlerWrite(&mtsNDISerialControllerQtWidget::ConnectedEventHandler,
                                                this, "Connected");
        interfaceRequired->AddEventHandlerWrite(&mtsNDISerialControllerQtWidget::TrackingEventHandler,
                                                this, "Tracking");
        interfaceRequired->AddEventHandlerVoid(&mtsNDISerialControllerQtWidget::UpdatedToolsEventHandler,
                                               this, "UpdatedTools");
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsNDISerialControllerQtWidget::Configure(const std::string & CMN_UNUSED(filename))
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure" << std::endl;
}

void mtsNDISerialControllerQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsNDISerialControllerQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsNDISerialControllerQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsNDISerialControllerQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsNDISerialControllerQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;

    Tracker.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);

    const ToolMap::iterator end = Tools.end();
    ToolMap::iterator toolIter;
    for (toolIter = Tools.begin();
         toolIter != end;
         ++toolIter) {
        Tool * tool = toolIter->second;
        tool->GetPositionCartesian(tool->Position);
        tool->Widget->SetValue(tool->Position);
    }
}

void mtsNDISerialControllerQtWidget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout;

    QHBoxLayout * topLayout = new QHBoxLayout;
    mainLayout->addLayout(topLayout);

    // Side by side for 3D position, gripper...
    QVBoxLayout * controlLayout = new QVBoxLayout;
    topLayout->addLayout(controlLayout);

    QGridLayout * gridLayout = new QGridLayout;
    gridLayout->setContentsMargins(2, 2, 2, 2);
    gridLayout->setSpacing(1);
    int row = 0;

    // connect
    gridLayout->addWidget(new QLabel("Connect"), row, 0);
    QCBConnect = new QCheckBox;
    QCBConnect->setChecked(false);
    gridLayout->addWidget(QCBConnect, row, 1);
    connect(QCBConnect, SIGNAL(toggled(bool)),
            this, SLOT(SlotConnect(bool)));
    row++;

    // port name
    gridLayout->addWidget(new QLabel("Port"), row, 0);
    QLPortName = new QLabel("");
    gridLayout->addWidget(QLPortName, row, 1);
    row++;

    // track
    gridLayout->addWidget(new QLabel("Track"), row, 0);
    QCBTrack = new QCheckBox;
    QCBTrack->setChecked(false);
    gridLayout->addWidget(QCBTrack, row, 1);
    connect(QCBTrack, SIGNAL(toggled(bool)),
            this, SLOT(SlotTrack(bool)));
    row++;

    // (re)initialize all
    QPBInitializeAll = new QPushButton("(Re)initialize");
    gridLayout->addWidget(QPBInitializeAll, row, 0);
    connect(QPBInitializeAll, SIGNAL(clicked()),
            this, SLOT(SlotInitializeAll()));
    row++;

    // beep
    QPBBeepButton = new QPushButton("Beep");
    gridLayout->addWidget(QPBBeepButton, row, 0);
    QSBBeepCount = new QSpinBox;
    QSBBeepCount->setMaximum(9);
    QSBBeepCount->setMinimum(1);
    QSBBeepCount->setValue(3);
    gridLayout->addWidget(QSBBeepCount, row, 1);
    connect(QPBBeepButton, SIGNAL(clicked()),
            this, SLOT(SlotBeep()));

    controlLayout->addLayout(gridLayout);
    controlLayout->addStretch();

    // System
    QVBoxLayout * systemLayout = new QVBoxLayout();
    topLayout->addLayout(systemLayout);

    // Timing
    QMIntervalStatistics = new mtsIntervalStatisticsQtWidget();
    systemLayout->addWidget(QMIntervalStatistics);

    // Messages
    QMMessage->setupUi();
    systemLayout->addWidget(QMMessage);

    // Tools in a tab
    QGTools = new QGridLayout();
    mainLayout->addLayout(QGTools);

    setLayout(mainLayout);
    setWindowTitle("sawNDITracker");
    resize(sizeHint());

    // connect slots/signal for cisst events
    connect(this, SIGNAL(SignalConnectedEvent()),
            this, SLOT(SlotConnectedEvent()));
    connect(this, SIGNAL(SignalUpdatedToolsEvent()),
            this, SLOT(SlotUpdatedToolsEvent()));

    // by default assumes the system is not connected
    SetControlWidgetsEnabled(false);
}

void mtsNDISerialControllerQtWidget::SetControlWidgetsEnabled(const bool enabled)
{
    // set the connected flag
    QCBConnect->setChecked(enabled);
    // enable/disable other buttons
    QPBInitializeAll->setEnabled(enabled);
    QCBTrack->setEnabled(enabled);
    QPBBeepButton->setEnabled(enabled);
    QSBBeepCount->setEnabled(enabled);
}

void mtsNDISerialControllerQtWidget::SlotConnect(bool connect)
{
    if (connect) {
        std::string serialPortName; // empty means use default
        Tracker.Connect(serialPortName);
    } else {
        QCBTrack->setChecked(false);
        Tracker.Disconnect();
    }
}

void mtsNDISerialControllerQtWidget::SlotInitializeAll(void)
{
    Tracker.InitializeAll();
}

void mtsNDISerialControllerQtWidget::SlotTrack(bool track)
{
    Tracker.Track(track);
}

void mtsNDISerialControllerQtWidget::SlotBeep(void)
{
    Tracker.Beep(QSBBeepCount->value());
}

void mtsNDISerialControllerQtWidget::ConnectedEventHandler(const std::string & connected)
{
    mSerialPort = connected;
    emit SignalConnectedEvent();
}

void mtsNDISerialControllerQtWidget::SlotConnectedEvent(void)
{
    if (mSerialPort.empty()) {
        SetControlWidgetsEnabled(false);
        QLPortName->setText("no connected");
    } else {
        SetControlWidgetsEnabled(true);
        QLPortName->setText(mSerialPort.c_str());
        // some tools might have been added before connect so update now
        SlotUpdatedToolsEvent();
    }
}

void mtsNDISerialControllerQtWidget::TrackingEventHandler(const bool & tracking)
{
    const bool oldState = QCBTrack->blockSignals(true);
    QCBTrack->setChecked(tracking);
    QCBTrack->blockSignals(oldState);
}

void mtsNDISerialControllerQtWidget::UpdatedToolsEventHandler(void)
{
    emit SignalUpdatedToolsEvent();
}

void mtsNDISerialControllerQtWidget::SlotUpdatedToolsEvent(void)
{
    std::string trackerName = "NDI";
        // ADV Tracker.Name(trackerName);

    std::vector<std::string> toolNames;
    Tracker.ToolNames(toolNames);

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
            tool->Widget = new prmPositionCartesianGetQtWidget();
            // busy wait until this is connected to retrieve moving/reference frames
            while (!tool->GetPositionCartesian(tool->Position)) {
                osaSleep(100.0 * cmn_ms);
            }
            QGTools->addWidget(tool->Widget, row, col);
            Tools.AddItem(name, tool);
        }
    }
}
