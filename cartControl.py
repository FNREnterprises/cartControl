
import os
import time
import threading
import numpy as np
import psutil
import simplejson as json
from typing import Tuple
import logging
#from rpyc.utils.server import ThreadedServer
import socket
from multiprocessing.managers import SyncManager
from queue import Empty

import sys

#import cProfile, pstats, io
#from pstats import SortKey

import marvinglobal.marvinglobal as mg
import marvinglobal.marvinShares as marvinShares
import marvinglobal.cartClasses as cartCls

import config
import arduinoReceive
import arduinoSend

import findObstacles
import threadMonitorForwardMove
#import camImages
import cart


def cartInit(verbose:bool):
    """
    try to connect with arduino
    :return:
    """
    config.log("cart - start arduino message receiver")
    config.msgThread = threading.Thread(target=arduinoReceive.readMessages, args={})
    config.msgThread.start()

    config.log(f"cartInit")
    config.log("try to open serial connection to arduino on cart (COM5)")
    config.stateLocal.cartStatus = mg.CartStatus.CONNECTING
    cart.publishState()

    arduinoReceive.initSerial("COM5")


    # wait for confirmed message exchange with cart arduino
    config.log(f"cart - wait for first message from arduino ...")
    timeoutWait = time.time() + 10
    while not config.arduinoConnEstablished and time.time() < timeoutWait:
        time.sleep(0.1)

    if not config.arduinoConnEstablished:
        config.log(f"cart - could not get message response from Arduino, going down")
        endCartControl()


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
    if config.share is not None:
        config.share.removeProcess(config.processName)
        config.stateLocal = mg.CartStatus.DOWN
        updStmt: Tuple[int, cartCls.State] = (mg.SharedDataItem.CART_STATE, config.stateLocal)
        config.share.updateSharedData(updStmt)
    os._exit(1)


def getRemainingRotation():
    d = abs(config.locationLocal.yaw - config.locationLocal.targetYaw) % 360
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


    config.share = marvinShares.MarvinShares()
    if not config.share.sharedDataConnect(config.processName):
        config.log(f"could not connect with marvinData")
        os._exit(1)

    # add own process to shared process list
    config.share.updateProcessDict(config.processName)

    config.log("cartControl started")

    cartInit(verbose=True)

    time.sleep(2)
    if not 'imageProcessing' in config.share.processDict.keys():
        config.log(f"cart needs a running imageProcessing, currently ignored")
        #endCartControl()


    # start forward move monitoring thread
    config.log(f"start forward move monitoring thread")
    config.navThread = threading.Thread(target=threadMonitorForwardMove.monitorLoop, args={})
    config.navThread.setName("threadMonitorForwardMove")
    #config.navThread.start()


    # check for cart ready, abort after time limit
    readyTimeoutWait = time.time() + 5000
    config.log(f'start waiting for cart ready, {time.time()}, {readyTimeoutWait}')
    while (not config.cartReady) and (time.time() < readyTimeoutWait):
        #config.log(f"not ready yet {time.time()}, {readyTimeoutWait}")
        time.sleep(1)

    if not config.cartReady:
        config.log(f"cart did not start up in time, going down {config.cartReady=}, {time.time()<readyTimeoutWait=}")
        endCartControl()

    config.log(f"wait for cart requests")

    #######################
    # TESTING send an initial request for a sensor test
    #######################
    #cmd = {'cmd': mg.CartCommand.TEST_IR_SENSOR, 'sensorId': 1}
    #config.share.cartRequestQueue.put(cmd)
    ###############################

    while True:

        request = None
        try:
            request = config.share.cartRequestQueue.get(timeout=1)
        except (TimeoutError, Empty):
            config.share.updateProcessDict(config.processName)
            arduinoSend.sendHeartbeat()
            #config.log(f"send heartbeat to marvinData and Arduino")
            continue
        except Exception as e:
            config.log(f"connection with sharedData lost {e=}, going down")
            endCartControl()

        config.log(f"new request: {request}")

        cmd = request['cmd']
        if cmd == mg.CartCommand.MOVE:
            #{'cmd': CartCommand.MOVE, 'direction': direction, 'speed': speed, 'distance': distance, 'protected': protected}
            cart.initiateMove(request['direction'], request['speed'], request['distance'], request['protected'])

        elif cmd == mg.CartCommand.ROTATE:
            #{'cmd': CartCommand.ROTATE, 'relAngle': , 'speed': speed, 'distance': distance, 'protected': protected})
            cart.initiateRotation(request['relAngle'], request['speed'])

        elif cmd == mg.CartCommand.SET_IR_SENSOR_REFERENCE_DISTANCES:
            #{'cmd': CartCommand.SET_IR_SENSOR_REFERENCE_DISTANCES, 'sensorId', 'distances'})
            # TODO: persist the reference distances
            config.log(f"set ir sensor reference distances {request['sensorId'], request['distances']}")
            arduinoSend.setIrSensorReferenceDistances(request['sensorId'], request['distances'])

        elif cmd == mg.CartCommand.TEST_IR_SENSOR:
            #{'cmd': CartCommand.TEST_IR_SENSOR, 'sensorId'})
            config.log(f"test ir sensor {request['sensorId']}")
            config.sensorTestDataLocal.numMeasures = 0
            config.sensorTestDataLocal.sumMeasures = np.zeros((mg.NUM_SCAN_STEPS), dtype=np.int16)
            arduinoSend.testSensorCommand(request['sensorId'])


        elif cmd == mg.CartCommand.SET_VERBOSE:
            arduinoSend.setVerbose(request['newState'])

        else:
            config.log(f"unknown cart request {request=}")



