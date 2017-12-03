#!/usr/bin/env python

import os
import time
import argparse

# command line arguments
parser = argparse.ArgumentParser(description='cisst/sawNDITracker')
parser.add_argument('--port', dest='port', required=True,
                    help='serial port (string with quotes)')

parser.add_argument('--json', dest='json', required=True, type=argparse.FileType('r'),
                    help='json configuration file')

cmd_line_args = parser.parse_args()

print('- port: ' + cmd_line_args.port)
print('- json: ' + cmd_line_args.json.name)

from cisstCommonPython import *
from cisstVectorPython import *
from cisstOSAbstractionPython import *
from cisstMultiTaskPython import *
from cisstParameterTypesPython import *

name = 'NDITracker'
period = 0.01

manager = mtsManagerLocal.GetInstance()
manager.CreateAllAndWait(5.0)
manager.StartAllAndWait(5.0)

proxy = mtsComponentWithManagement('{}Proxy'.format(name))
manager.AddComponent(proxy)
proxy.CreateAndWait(5.0)
time.sleep(0.5)

services = proxy.GetManagerComponentServices()

# create an instance of the tracker
print('--> loading dynamic library')
result = services.Load('sawNDITracker')
assert result, 'Failed to load {} using component services'.format('sawNDITracker')

print('--> create tracker component')
args = mtsTaskPeriodicConstructorArg(name, period, False, 256)
result = services.ComponentCreate('mtsNDISerial', args)
assert result, 'Failed to create {} of type {}'.format(name, 'mtsNDISerial')

# Configure the component
print('--> configure tracker component')
component = manager.GetComponent(name)
component.Configure(cmd_line_args.json.name)

# create the main interface to the tracker
print('--> create python interface for tracker')
controller = proxy.AddInterfaceRequiredAndConnect((name, 'Controller'))

component.CreateAndWait(5.0)
component.StartAndWait(5.0)

# initialize controller
print ('--> connect tracker to port')
controller.Connect(cmd_line_args.port)
time.sleep(2.0)

# see if the device is actually connected
print('--> make the tracker beep to make sure everything is ok')
controller.Beep(1)
time.sleep(1.0)
controller.Beep(2)

# create an interface for the tracked pointer body
print('--> assuming there is a rigid body named "Pointer", create an interface and track')
trackedBodyName = 'Pointer'
trackedBody = proxy.AddInterfaceRequiredAndConnect((name, trackedBodyName))

#enable tracking
controller.ToggleTracking(True)
controller.Beep(2)

print('--> controller tracking status: ' + str(controller.IsTracking()))

while True:
    pose = trackedBody.GetPositionCartesian()
    if pose.GetValid():  # if visible
        print(trackedBodyName + ': ' + str(pose.Position().Translation()))
    else:
        print(trackedBodyName + 'is not visible')
