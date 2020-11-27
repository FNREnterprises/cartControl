
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
        sys.exit()

    arduinoSend.setVerbose(verbose)

    # send config values to cart
    config.log("initialize cart sensors ...")
    arduinoSend.sendConfigValues()

    # get current orientation of cart (bno055)
    arduinoSend.requestCartYaw()
    time.sleep(0.1)
    cart.loadCartLocation()

    findObstacles.initObstacleDetection()

'''
def updateCartLocation(magnitude, final=False):
    """
    based on distance update from the cart's odometer update the current cart position
    cart 0 degrees is to the right
    include xy drift by rotation
    :param orientation:
    :param magnitude: for rotation the angle in degrees, for moves the distance in mm
    :param moveDirection:
    :return:
    """
    if not final and magnitude == 0:
        return

    if config.movement.moveDirection not in [mg.MoveDirection.ROTATE_LEFT, mg.MoveDirection.ROTATE_RIGHT]:

        # for x/y change calculation we need degrees that start to the right (normal circle)
        # take cart orientation and cart move direction into account
        # cart orientation 0 is to the right
        trigDegrees = config.evalCartDegrees(config.location.yaw, config.movement.moveDirection)
        if trigDegrees is not None:
            config.location.x = config.movement.startX + int(magnitude * np.cos(np.radians(trigDegrees)))
            config.location.y = config.movement.startY + int(magnitude * np.sin(np.radians(trigDegrees)))
        else:
            config.log(f"unexpected missing trigDegrees in updateCartLocation, distance: {magnitude}, moveDirection: {config.movement.moveDirection}, final: {final}")

    #rpcSend.publishCartProgress(moveDirection, magnitude, final)

    if final or time.time() - config.location.lastLocationSaveTime > 0.2:
        config.share.updateSharedData(mg.SharedDataItem.LOCATION, config.location)
        saveCartLocation()
'''


def calculateCartTarget(orientation, distance, moveDirection: 'config.Direction'):

    # for x/y change calculation we need degrees that start to the right (normal circle)
    # take cart orientation and cart move direction into account
    # cart orientation 0 is straight up
    trigDegrees = config.evalCartDegrees(orientation, moveDirection)
    if trigDegrees is not None:
        cart.location.targetX = cart.movement.moveStartX + int(distance * np.cos(np.radians(trigDegrees)))
        cart.location.targetY = cart.movement.moveStartY + int(distance * np.sin(np.radians(trigDegrees)))
        config.log(f"move - from: {cart.location.x:.0f} / {cart.location.y:.0f}, orientation: {orientation}, moveDir: {moveDirection}, deg: {trigDegrees:.0f}, dist: {distance}, to: {cart.location.carttargetX:.0f} / {cart.location.carttargetY:.0f}")

    else:
        config.log(
            f"unexpected missing trigDegrees in calculateCarttarget, distance: {distance}, moveDirection: {moveDirection}")



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

    cartInit(verbose=False)

    time.sleep(2)
    if not 'imageProcessing' in config.share.processDict.keys():
        config.log(f"cart needs a running imageProcessing, going down")
        #os._exit(2)


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
        #config.navThread.join()
        #config.msgThread.join()
        os._exit(1)

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
        except TimeoutError:
            config.share.updateProcessDict(config.processName)
            continue
        except Empty:
            continue
        except Exception as e:
            config.log(f"connection with sharedData lost {e=}, going down")
            os._exit(2)

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



