
import os
import time
import threading
import numpy as np
import psutil
import distanceSensors
from queue import Empty
#import subprocess

from marvinglobal import marvinglobal as mg
from marvinglobal import marvinShares as marvinShares
from marvinglobal import cartClasses
from marvinglobal import distanceSensorClasses

import config
import arduinoReceive
import arduinoSend

import findObstacles
#import threadMonitorForwardMove
import serial
import cart
import simplejson as json

def cartInit(verbose:bool):
    """
    try to connect with cart control arduino (C0)
    :return:
    """
    arduinoConnection = "/dev/ttyACM"

    config.arduino = None
    config.arduinoConnEstablished = False

    # in case we have been connected previously try to use this port first
    firstPortToTry = {'port': 0}
    if os.path.isfile('lastCartArduinoConnection.json'):
        with open('lastCartArduinoConnection.json') as f:
            firstPortToTry = json.load(f)
            config.log("trying last used arduino connection first")
    checkPorts = [firstPortToTry['port'], 0, 1, 2, 3, 4, 5]

    for portNumber in checkPorts:
        usbPort = arduinoConnection + str(portNumber)
        arduino = None
        try:
            arduino = serial.Serial(usbPort)
            arduino.baudrate = 115200

        except Exception as e:
            config.log(f"could not connect with {usbPort}, try next")
            continue

        # arduino sends an initial message, check for correct Arduino
        if arduino.is_open:

            config.log(f"connected with {usbPort}, try to read first message")
            numBytesAvailable = 0
            timeout = time.time() + 5
            try:
                while numBytesAvailable == 0 and time.time() < timeout:
                    numBytesAvailable = arduino.inWaiting()
            except Exception as e:
                config.log(f"exception in arduinoReceive, readMessages: {e}")
                continue

            if numBytesAvailable == 0:
                config.log(f"could not receive initial message")
                continue
            else:
                # cartGlobal.log(f"inWaiting > 0")
                recvB = arduino.readline()
                try:
                    recv = recvB.decode()[:-2]  # without cr/lf
                except Exception as e:
                    config.log(f"problem with decoding cart msg '{recvB}' {e}")
                    continue

                if recv[0:3] == "C0 ":
                    config.arduino = arduino
                    config.arduinoConnEstablished = True
                    config.log(f"received {recv}, connected with cartControlArduino on usb port {usbPort}")

                    # save port number as last used port
                    with open('lastCartArduinoConnection.json', 'w') as json_file:
                        json.dump({'port': portNumber}, json_file)
                        config.log(f"saving current arduino port as last used one {arduinoConnection}{portNumber}")
                    break

                else:
                    config.log(f"received {recv}, not a cartControlArduino, try next")
                    continue

    if config.arduino is None:
        config.log(f"could not find a cartControlArduino, going down")
        endCartControl()
    """
        for servoTypeName, servoTypeData in servoTypeDefinitions.items():
            servoType = skeletonClasses.ServoType()   # inst of servoTypeData class
            servoType.updateValues(servoTypeData)
            config.servoTypeDictMaster.update({servoTypeName: servoType})

            # add servoTypes to shared data
            #updStmt = (mg.SharedDataItems.SERVO_TYPE, servoTypeName, dict(servoType.__dict__))
            msg = {'msgType': mg.SharedDataItems.SERVO_TYPE, 'sender': config.processName,
                   'info': {'type': servoTypeName, 'data': dict(servoType.__dict__)}}
            config.updateSharedDict(msg)
    """
    # create the distance sensor objects
    for i, item in enumerate(distanceSensorClasses.swipingIrSensorProperties):
        swipingIrSensor = distanceSensorClasses.SwipingIrSensor(**item)   # pass dict as parameters
        config.swipingIrSensorsMaster.update({i: swipingIrSensor})

        # populate the shared version of the dict
        msg = {'msgType': mg.SharedDataItems.SWIPING_IR_SENSORS, 'sender': config.processName,
               'info': {'irSensorId': i, 'data': config.swipingIrSensorsMaster[i].__dict__}}
        config.marvinShares.updateSharedData(msg)

    for i, item in enumerate(distanceSensorClasses.staticIrSensorProperties):
        id = i + mg.NUM_SWIPING_IR_DISTANCE_SENSORS
        staticIrSensor = distanceSensorClasses.StaticIrSensor(**item)
        config.staticIrSensorsMaster.update({id: staticIrSensor})

        # populate the shared version of the dict
        msg = {'msgType': mg.SharedDataItems.STATIC_IR_SENSORS, 'sender': config.processName,
               'info': {'irSensorId': id, 'data': config.staticIrSensorsMaster[id].__dict__}}
        config.marvinShares.updateSharedData(msg)

    for i, item in enumerate(distanceSensorClasses.usSensorProperties):
        id = mg.NUM_SWIPING_IR_DISTANCE_SENSORS + mg.NUM_STATIC_IR_DISTANCE_SENSORS + i
        usSensor = distanceSensorClasses.UsSensor(**item)     # convert dict to param list
        config.usSensorsMaster.update({id: usSensor})

        # populate the shared version of the dict
        msg = {'msgType': mg.SharedDataItems.ULTRASONIC_SENSORS, 'sender': config.processName,
               'info': {'usSensorId': id, 'data': config.usSensorsMaster[id].__dict__}}
        config.marvinShares.updateSharedData(msg)


    # start message receiving thread on verified port
    config.log(f"start thread for reading arduino messages")
    config.msgThread = threading.Thread(target=arduinoReceive.readMessages, args={})
    config.msgThread.start()

    arduinoSend.setVerbose(verbose)

    # send config values to cart
    config.log("initialize cart sensors ...")
    arduinoSend.sendConfigValues()

    # get current orientation of cart (bno055)
    arduinoSend.requestCartYaw()
    time.sleep(0.1)
    cart.loadCartLocation()

    findObstacles.initObstacleDetection()


def endCartControl():
    if config.arduinoConnEstablished:
        config.arduino.close()
    if config.marvinShares is not None:
        config.marvinShares.removeProcess(config.processName)
        config.stateMaster = mg.CartStatus.DOWN
        #updStmt: Tuple[int, cartClasses.State] = (mg.SharedDataItems.CART_STATE, config.stateMaster)
        updStmt = {'msgType': mg.SharedDataItems.CART_STATE, 'sender': config.processName, 'info': config.stateMaster}
        config.marvinShares.updateSharedData(updStmt)
    os._exit(2)


def getRemainingRotation():
    d = abs(config.locationMaster.yaw - config.locationMaster.targetYaw) % 360
    return 360 - d if d > 180 else d


def getLastBatteryCheckTime():
    return config._lastBatteryCheckTime


def setLastBatteryCheckTime(newTime):
    config._lastBatteryCheckTime = newTime


def getBatteryStatus():
    return config._batteryStatus


def updateBatteryStatus():

    config._batteryStatus = psutil.sensors_battery()
    setLastBatteryCheckTime(time.time())

    # inform navManager about low battery
    if config._batteryStatus.percent < 25 and not config._batteryStatus.power_plugged:
        msg = f"low battery: {config._batteryStatus.percent:.0f} percent"
        config.log(msg)




if __name__ == '__main__':

    #config.marvinShares = marvinShares.MarvinShares()  # shared data

    config.log(f"{config.processName},  trying to connect with marvinData")
    if not config.marvinShares.sharedDataConnect(config.processName):
        config.log(f"could not connect with marvinData")
        os._exit(3)

    # add own process to shared process list
    config.marvinShares.updateProcessDict(config.processName)

    config.log("cartControl started")

    cartInit(verbose=False)

    time.sleep(2)
    if not 'imageProcessing' in config.marvinShares.processDict.keys():
        config.log(f"cart needs a running imageProcessing, currently ignored")
        #endCartControl()


    # check for cart ready, abort after time limit
    readyTimeoutWait = time.time() + 7
    config.log(f'start waiting for cart ready, max 5 s')
    while (not config.cartReady) and (time.time() < readyTimeoutWait):
        #config.log(f"not ready yet {time.time()}, {readyTimeoutWait}")
        time.sleep(1)

    if not config.cartReady:
        config.log(f"cart did not start up in time, going down {config.cartReady=}, {time.time()<readyTimeoutWait=}")
        endCartControl()

    # empty request queue first
    while not config.marvinShares.cartRequestQueue.empty:
        config.marvinShares.cartRequestQueue.get()

    config.log(f"wait for new cart requests")

    #######################
    # TESTING send an initial request for a sensor test
    #######################
    #cmd = {'msgType': mg.CartCommands.TEST_IR_SENSOR, 'sensorId': 1}
    #config.share.cartRequestQueue.put(cmd)
    ###############################

    while True:

        request = None
        try:
            request = config.marvinShares.cartRequestQueue.get(timeout=1)
        except (TimeoutError, Empty):
            config.marvinShares.updateProcessDict(config.processName)
            arduinoSend.sendHeartbeat()
            #config.log(f"send heartbeat to marvinData and Arduino")
            continue
        except Exception as e:
            config.log(f"connection with sharedData lost {e=}, going down")
            endCartControl()

        config.log(f"new cart request: {request}")

        msgType = request['msgType']
        if 'sender' not in request:
            config.log(f"cart request from unknown sender, ignore")
            continue
        sender = request['sender']
        if msgType == mg.CartCommands.MOVE:
            #{'msgType': CartCommands.MOVE, 'direction': direction, 'speed': speed, 'distance': distance, 'protected': protected}
            cart.initiateMove(request['direction'], request['speed'], request['distance'], request['protected'])

        elif msgType == mg.CartCommands.ROTATE:
            #{'msgType': CartCommands.ROTATE, 'relAngle': , 'speed': speed, 'distance': distance, 'protected': protected})
            cart.initiateRotation(request['relAngle'], request['speed'])

        elif msgType == mg.CartCommands.SET_IR_SENSOR_REFERENCE_DISTANCES:
            #{'msgType': CartCommands.SET_IR_SENSOR_REFERENCE_DISTANCES, 'sensorId', 'distances'})
            # TODO: persist the reference distances
            sensorId = request['sensorId']
            config.log(f"set ir sensor reference distances {sensorId}, {request['distances']}")
            arduinoSend.setIrSensorReferenceDistances(request['sensorId'], request['distances'])

            # update shared data with new reference values
            if sensorId < mg.NUM_SWIPING_IR_DISTANCE_SENSORS:
                config.swipingIrSensorsMaster[sensorId]['referenceDistances'] = request['distances']
                updStmt = {'msgType': mg.SharedDataItems.SWIPING_IR_SENSORS, 'sender': config.processName,
                   'info': {'irSensorId': request['sensorId'], 'data': config.swipingIrSensorsMaster[sensorId]}}
                config.marvinShares.updateSharedData(updStmt)

            if sensorId >= mg.NUM_SWIPING_IR_DISTANCE_SENSORS:
                config.staticIrSensorsMaster[sensorId]['referenceDistances'] = request['distances']
                updStmt = {'msgType': mg.SharedDataItems.STATIC_IR_SENSORS, 'sender': config.processName,
                   'info': {'irSensorId': request['sensorId'], 'data': config.staticIrSensorsMaster[sensorId]}}
                config.marvinShares.updateSharedData(updStmt)


        elif msgType == mg.CartCommands.TEST_IR_SENSOR:
            #{'msgType': CartCommands.TEST_IR_SENSOR, 'sensorId'})
            config.log(f"test ir sensor {request['sensorId']}")
            distanceSensors.sensorInTest = request['sensorId']
            arduinoSend.testSensorCommand(request['sensorId'])


        elif msgType == mg.CartCommands.SET_VERBOSE:
            arduinoSend.setVerbose(request['newState'])


        elif msgType == mg.CartCommands.STOP_CART:
            arduinoSend.sendStopCommand(request['reason'])

        else:
            config.log(f"unknown cart request {request=}")



