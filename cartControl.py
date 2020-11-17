
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
    config.state.cartStatus = mg.CartStatus.CONNECTING
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
    loadCartLocation()

    findObstacles.initObstacleDetection()


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

    if config.movement.moveDirection not in [config.MoveDirection.ROTATE_LEFT, config.MoveDirection.ROTATE_RIGHT]:

        # for x/y change calculation we need degrees that start to the right (normal circle)
        # take cart orientation and cart move direction into account
        # cart orientation 0 is to the right
        trigDegrees = config.evalCartDegrees(cart.location.yaw, moveDirection)
        if trigDegrees is not None:
            cart.location.x = config._moveStartX + int(magnitude * np.cos(np.radians(trigDegrees)))
            cart.location.y = config._moveStartY + int(magnitude * np.sin(np.radians(trigDegrees)))
        else:
            config.log(f"unexpected missing trigDegrees in updateCartLocation, distance: {magnitude}, moveDirection: {moveDirection}, final: {final}")

    #rpcSend.publishCartProgress(moveDirection, magnitude, final)

    if final or time.time() - cart.lastLocationSaveTime > 0.2:
        config.share.updateSharedData(mg.SharedDataUpdate.LOCATION, cart.location)
        saveCartLocation()



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
    d = abs(cart.location.yaw - cart.location.targetYaw) % 360
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


def saveCartLocation():
    # Saving the objects:
    cartData = {'posX': int(cart.location.x), 'posY': int(cart.location.y), 'yaw': int(cart.location.yaw)}
    #if cart.location.x == 0 and cart.location.y == 0:
    #    config.log(f"CART LOCATION SAVED AS 0,0")
    filename = f"cartLocation.json"
    with open(filename, "w") as write_file:
        json.dump(cartData, write_file)
    cart.lastLocationSaveTime = time.time()
    #config.log(f"cart location saved")


def loadCartLocation():
    """
    we load the cart location from the file system
    as the carts imu is reset with each start of the cart the carts orientation might be 0
    set an orientationCorrection value to account for this
    :return:
    """
    # Getting back the last saved cart data
    config.log(f"load cart location")
    filename = f"cartLocation.json"
    cart.location = mg.Location()
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            cartData = json.load(read_file)

        config.location.x = cartData['posX']
        config.location.y = cartData['posY']
        config.platformImu.yawCorrection = (cartData['yaw'] - config.platformImu.yaw) % 360
        config.location.yaw = (cartData['yaw'] + config.platformImu.yawCorrection) % 360

    else:
        config.location.x = 0
        config.location.y = 0
        config.location.yaw = 0
        config.platformImu.yawCorrection = 0

    cart.publishPlatformImu()
    cart.publishLocation()

    config.log(f"imuYaw: {config.platformImu.yaw}," +
        f"cartYawCorrection: {config.platformImu.yawCorrection}," +
        f"roll: {config.platformImu.roll}," +
        f"pitch: {config.platformImu.pitch}, " +
        f"cartX: {config.location.x}, " +
        f"cartY: {config.location.y}, " +
        f"lastYaw: {config.location.yaw}")



if __name__ == '__main__':


    config.share = mg.MarvinShares()
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
    config.log(f'start waiting for cart ready')
    timeoutWait = time.time() + 5
    while (not config.cartReady) and (time.time() < timeoutWait):
        time.sleep(0.1)

    if not config.cartReady:
        config.log(f"cart did not start up in time, going down {config.cartReady=}, {time.time()<timeoutWait=}")
        #config.navThread.join()
        #config.msgThread.join()
        os._exit(1)

    config.log(f"wait for cart requests")

    #######################
    # send an initial request for testing
    #######################
    cmd = {'cmd':mg.CART_TEST_SENSOR, 'sensorId': 0}
    config.share.cartRequestQueue.put(cmd)
    ###############################

    while True:
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
        if cmd == mg.CART_MOVE:
            #{'cmd': CART_MOVE, 'direction': direction, 'speed': speed, 'distance': distance, 'protected': protected}
            arduinoSend.sendMoveCommand(request['direction'], request['speed'], request['distance'], request['protected'])

        elif cmd == mg.CART_ROTATE:
            #{'cmd': CART_ROTATE, 'relAngle': , 'speed': speed, 'distance': distance, 'protected': protected})
            arduinoSend.sendRotateCommand(request['relAngle'], request['speed'])

        elif cmd == mg.CART_SET_FLOOR_DISTANCES:
            #{'cmd': CART_SET_FLOOR_DISTANCES, 'sensorId', 'values'})
            arduinoSend.setFloorDistances(request['sensorId'])

        elif cmd == mg.CART_TEST_SENSOR:
            #{'cmd': CART_TEST_SENSOR, 'sensorId'})
            config.sensorTestData.numMeasures = 0
            config.sensorTestData.sumMeasures = np.zeros((mg.NUM_SCAN_STEPS), dtype=np.int16)
            arduinoSend.testSensorCommand(request['sensorId'])

        elif cmd == mg.CART_SET_VERBOSE:
            arduinoSend.setVerbose(request['newState'])

        else:
            config.log(f"unknown cart request {request=}")



