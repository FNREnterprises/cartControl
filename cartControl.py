
import os
import time
import threading
import numpy as np
import psutil
import simplejson as json
import logging
from rpyc.utils.server import ThreadedServer
import socket

from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys

import cProfile, pstats, io
from pstats import SortKey

import inmoovGlobal
import config
import threadWatchConnections
import arduinoReceive
import arduinoSend
import rpcReceive
import rpcSend
import gui
import findObstacles
import threadMonitorForwardMove
import camImages
import cartGui

standAloneMode = False  # False -> slave mode,  True -> standalone mode


def cartInit():
    """
    try to connect with arduino
    :return:
    """
    config.log(f"cartInit")
    config.log("try to open serial connection to arduino on cart (COM5)")
    arduinoReceive.initSerial("COM5")

    # start serial reader for arduino messages
    msgThread = threading.Thread(target=arduinoReceive.readMessages, args={})
    msgThread.start()

    config.log("cart - arduino message receiver thread started")

    #camImages.initCameras()

    # wait for cart startup finished
    config.log(f"cart - wait for first message from arduino ...")
    timeout = time.time() + 10
    while config.arduinoStatus == 0 or time.time() < timeout:
        time.sleep(0.1)

    if config.arduinoStatus == 0:
        config.log(f"cart - timeout in arduino connection, terminating")
        raise SystemExit()

    setMovementBlocked(False)

    arduinoSend.setVerbose(False)

    # start cart control gui and initialization
    #guiThread = threading.Thread(target=gui.startGui, args={})
    #guiThread.start()
    #config.log("cart gui started")

    # send config values to cart
    config.log("initialize cart sensors ...")
    arduinoSend.sendConfigValues()

    # get current orientation of cart (bno055)
    arduinoSend.requestCartOrientation()
    time.sleep(0.1)
    loadCartLocation()

    findObstacles.initObstacleDetection()

    # check for cart ready, abort after time limit
    timeoutReady = time.time() + 5
    while (not config.cartReady) and (time.time() < timeoutReady):
        time.sleep(0.1)

    if config.cartReady:
        rpcSend.publishCartInfo()
        rpcSend.publishServerReady()
    else:
        config.log(f"timeout in cart startup, going down")
        os._exit(0)



def updateCartLocation(magnitude, moveDirection, final=False):
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

    if moveDirection not in [config.MoveDirection.ROTATE_LEFT, config.MoveDirection.ROTATE_RIGHT]:

        # for x/y change calculation we need degrees that start to the right (normal circle)
        # take cart orientation and cart move direction into account
        # cart orientation 0 is to the right
        trigDegrees = config.evalCartDegrees(getCartYaw(), moveDirection)
        if trigDegrees is not None:
            config.cartLocationX = config._moveStartX + int(magnitude * np.cos(np.radians(trigDegrees)))
            config.cartLocationY = config._moveStartY + int(magnitude * np.sin(np.radians(trigDegrees)))
            config.cartStateChanged = True
        else:
            config.log(f"unexpected missing trigDegrees in updateCartLocation, distance: {magnitude}, moveDirection: {moveDirection}, final: {final}")

    rpcSend.publishCartProgress(moveDirection, magnitude, final)

    if final or time.time() - config.lastLocationSaveTime > 0.2:
        saveCartLocation()


def getCartYaw():
    config.cartOrientation =  (config.platformImuYaw + config.platformImuYawCorrection) % 360
    return config.cartOrientation


def calculateCartTargetLocation(orientation, distance, moveDirection: 'config.Direction'):

    # for x/y change calculation we need degrees that start to the right (normal circle)
    # take cart orientation and cart move direction into account
    # cart orientation 0 is straight up
    trigDegrees = config.evalCartDegrees(orientation, moveDirection)
    if trigDegrees is not None:
        config.cartTargetLocationX = config._moveStartX + int(distance * np.cos(np.radians(trigDegrees)))
        config.cartTargetLocationY = config._moveStartY + int(distance * np.sin(np.radians(trigDegrees)))
        config.cartStateChanged = True
        config.log(f"move - from: {config.cartLocationX:.0f} / {config.cartLocationY:.0f}, orientation: {orientation}, moveDir: {moveDirection}, deg: {trigDegrees:.0f}, dist: {distance}, to: {config.cartTargetLocationX:.0f} / {config.cartTargetLocationY:.0f}")

    else:
        config.log(
            f"unexpected missing trigDegrees in calculateCartTargetLocation, distance: {distance}, moveDirection: {moveDirection}")


def getCartLocation():
    return round(config.cartLocationX), round(config.cartLocationY)


def getMoveStart():
    return config._moveStartX, config._moveStartY


def getRemainingRotation():
    d = abs(getCartYaw() - config.cartTargetOrientation) % 360
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


def getVoltage12V():
    return config._mVolts12V


def setVoltage12V(value):
    config._mVolts12V = value




def saveCartLocation():
    # Saving the objects:
    cartData = {'posX': int(config.cartLocationX), 'posY': int(config.cartLocationY), 'orientation': getCartYaw()}
    #if config.cartLocationX == 0 and config.cartLocationY == 0:
    #    config.log(f"CART LOCATION SAVED AS 0,0")
    filename = f"cartLocation.json"
    with open(filename, "w") as write_file:
        json.dump(cartData, write_file)
    config.lastLocationSaveTime = time.time()
    #config.log(f"cart location saved")


def loadCartLocation():
    """
    we load the cart location from the file system
    as the carts imu is reset with each start of the cart the carts orientation might be 0
    set an orientationCorrection value to account for this
    :return:
    """
    # Getting back the last saved cart data
    filename = f"cartLocation.json"
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            cartData = json.load(read_file)

        config.cartLocationX = cartData['posX']
        config.cartLocationY = cartData['posY']
        lastYaw = cartData['orientation']
        config.platformImuYawCorrection = (lastYaw - config.platformImuYaw) % 360

    else:
        config.cartLocationX = 0
        config.cartLocationY = 0
        config._cartOrientation = 0
        lastYaw = 0
        config.platformImuYawCorrection = 0

    config.log(f"imuYaw: {config.platformImuYaw}, cartYawCorrection: {config.platformImuYawCorrection}, roll: {config.platformImuRoll}, pitch: {config.platformImuPitch}, cartX: {config.cartLocationX}, cartY: {config.cartLocationY}, lastYaw: {lastYaw}")


def setMovementBlocked(new):

    config._movementBlocked = new
    config._timeMovementBlocked = time.time()


def getMovementBlocked():
    return config._movementBlocked


def getMovementBlockedStartTime():
    return config._timeMovementBlocked


def setRequestedCartSpeed(speed):
    config._requestedCartSpeed = speed


def getRequestedCartSpeed():
    return config._requestedCartSpeed


def setCartMoving(newState, timeout=0):

    # log(f"setCartMoving {new}")
    config.cartMoving = newState
    config._cartMoveTimeout = timeout

    config.cartRotating = False


def isCartMoving():
    return config.cartMoving


def setCartRotating(new):

    # log(f"setCartRotating {new}")
    config.cartRotating = new
    config.cartMoving = False


def setCartMoveDistance(distanceMm):
    config._moveDistanceRequested = distanceMm
    config._moveStartX = config.cartLocationX
    config._moveStartY = config.cartLocationY
    config._moveStartTime = time.time()


if __name__ == '__main__':

    config.pcName = socket.gethostname()
    config.pcIP = socket.gethostbyname(config.pcName)

    ##########################################################
    # initialization
    # Logging, renaming old logs for reviewing ...
    baseName = "log/cartControl"
    oldName = f"{baseName}9.log"
    if os.path.isfile(oldName):
        os.remove(oldName)
    for i in reversed(range(9)):
        oldName = f"{baseName}{i}.log"
        newName = f"{baseName}{i+1}.log"
        if os.path.isfile(oldName):
            os.rename(oldName, newName)
    oldName = f"{baseName}.log"
    newName = f"{baseName}0.log"
    if os.path.isfile(oldName):
        try:
            os.rename(oldName, newName)
        except Exception as e:
            config.log(f"can not rename {oldName} to {newName}")

    logging.basicConfig(
        filename="log/cartControl.log",
        level=logging.INFO,
        format='%(message)s',
        filemode="w")

    config.log("cartControl started")

    if standAloneMode:

        pr = cProfile.Profile()

        # start arduino thread
        config.log(f"start cart initialization thread")
        startupThread = threading.Thread(target=cartInit, args={})
        startupThread.setName("cartInit")
        startupThread.start()

        time.sleep(2)
        if not camImages.initCameras():
            config.log(f"could not connect with all cams")
            exit()

        logWaitDone = False
        while config.arduinoStatus == 0:
            if not logWaitDone:
                config.log(f"waiting for first message from arduino")
                logWaitDone = True
            time.sleep(1)

        # erase EEPROM to force new calibration of distance sensors with next restart
        #arduinoSend.distanceSensorCalibration()

        #camImages.cams[inmoovGlobal.EYE_CAM].takeImage(show=False)
        #camImages.cams[inmoovGlobal.CART_CAM].takeImage(show=False)
        #camImages.cams[inmoovGlobal.HEAD_CAM].takeImage(show=False)
        #camImages.cams[inmoovGlobal.HEAD_CAM].takeImage(show=False)
        arduinoSend.requestCartOrientation()
        
        camImages.cams[inmoovGlobal.HEAD_CAM].startStream()

        rawPoints = camImages.cams[inmoovGlobal.HEAD_CAM].takeDepth()
        if rawPoints is None:
            config.log(f"could not acquire the depth points")

        else:
            camView = "ground"
            if camView == "ground":
                # with cam in ground view position
                #pr.enable()
                rotatedPoints = camImages.alignPointsWithGround(rawPoints, config.headImuPitch)
                distClosestObstacle, obstacleDirection = findObstacles.distGroundObstacles(rotatedPoints)
                #pr.disable()

            if camView == "ahead":
                # with cam in ahead watch position
                # handleRobot. inmoovGlobal.pitchWallWatchDegrees
                rotatedPoints = camImages.alignPointsWithGround(rawPoints, inmoovGlobal.pitchWallWatchDegrees)
                freeMoveDistance, obstacleDirection = findObstacles.lookForCartPathObstacle(rotatedPoints)

            if camView == "wall":
                config.log(f"obstacle in path: freeMoveDistance: {freeMoveDistance:.3f}, obstacleDirection: {obstacleDirection:.1f}")
                rotatedPoints = camImages.alignPointsWithGround(rawPoints, inmoovGlobal.pitchWallWatchDegrees)
                obstacleLine = findObstacles.findObstacleLine(rotatedPoints)

        camImages.cams[inmoovGlobal.HEAD_CAM].stopStream()

        '''
        s = io.StringIO()
        sortby = SortKey.CUMULATIVE
        ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        ps.print_stats()
        print(s.getvalue())
        '''
        exit()


    if not standAloneMode:

        # start arduino thread
        config.log(f"start cart initialization thread")
        startupThread = threading.Thread(target=cartInit, args={})
        startupThread.setName("cartInit")
        startupThread.start()

        time.sleep(2)
        if not camImages.initCameras():
            config.log(f"could not connect with all cams")
            exit()

        logWaitDone = False
        while config.arduinoStatus == 0:
            if not logWaitDone:
                config.log(f"waiting for first message from arduino")
                logWaitDone = True
            time.sleep(1)

        # start forward move monitoring thread
        config.log(f"start forward move monitoring thread")
        navThread = threading.Thread(target=threadMonitorForwardMove.monitorLoop, args={})
        navThread.setName("threadMonitorForwardMove")
        navThread.start()

        # start communication watchdog thread
        config.log(f"start rpyc communication watchdog")
        navThread = threading.Thread(target=threadWatchConnections.watchDog, args={})
        navThread.setName("threadWatchConnections")
        navThread.start()

        # start listener for rpc commands, does not return
        #config.log(f"start listening for rpyc commands on port {config.MY_RPC_PORT}")
        #myConfig = {"allow_all_attrs": True, "allow_pickle": True}
        #server = ThreadedServer(rpcReceive.rpcListener, port=config.MY_RPC_PORT, protocol_config=myConfig)
        #server.start()

        config.log(f"start listening for rpyc commands on port {config.MY_RPC_PORT}")
        server = ThreadedServer(rpcReceive.rpcListener(), port=config.MY_RPC_PORT,
                                protocol_config={'allow_public_attrs': True})
        rpycThread = threading.Thread(target=server.start)
        rpycThread.name = "rpycListener"
        rpycThread.start()

        cartGui.startGui()


