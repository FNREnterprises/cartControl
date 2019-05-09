
import os
import time
import threading
import numpy as np
import psutil
import simplejson as json
import logging
from rpyc.utils.server import ThreadedServer

import config
import watchDog
import arduinoReceive
import arduinoSend
import rpcReceive
import rpcSend
import gui


CART_PORT = 20003


startupThread = None


def cartInit():
    """
    try to connect with arduino
    :return:
    """

    config.log("try to open serial connection to arduino on cart (COM5)")
    arduinoReceive.initSerial("COM5")

    # start serial reader for arduino messages
    msgThread = threading.Thread(target=arduinoReceive.readMessages, args={})
    msgThread.start()

    config.log("cart - arduino message receiver thread startet")

    # wait for cart startup finished
    config.log(f"cart - wait for first message from arduino ...")
    timeout = time.time() + 10
    while config.arduinoStatus == 0 or time.time() < timeout:
        time.sleep(1)

    if config.arduinoStatus == 0:
        config.log(f"cart - timeout in arduino connection, terminating")
        raise SystemExit()

    setMovementBlocked(False)
    arduinoSend.setVerbose(False)

    # start cart control gui and initialization
    guiThread = threading.Thread(target=gui.startGui, args={})
    guiThread.start()
    config.log("cart gui started")

    # send config values to cart
    config.log("initialize cart sensors ...")
    arduinoSend.sendConfigValues()

    # get current orientation of cart (bno055)
    arduinoSend.requestCartOrientation()
    time.sleep(0.1)
    loadCartLocation()

    timeoutReady = time.time() + 5
    while time.time() < timeoutReady:
        time.sleep(1)

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
    if magnitude == 0:
        return

    if moveDirection not in [config.Direction.ROTATE_LEFT, config.Direction.ROTATE_RIGHT]:
        # for x/y change calculation we need degrees that start to the right (normal circle)
        # take cart orientation and cart move direction into account
        # cart orientation 0 is to the right
        trigDegrees = config.evalTrigDegrees(getCartYaw(), moveDirection)
        if trigDegrees is not None:
            config.cartLocationX = config._moveStartX + int(magnitude * np.cos(np.radians(trigDegrees)))
            config.cartLocationY = config._moveStartY + int(magnitude * np.sin(np.radians(trigDegrees)))
        else:
            config.log(f"unexpected missing trigDegrees in updateCartLocation, distance: {magnitude}, moveDirection: {moveDirection}, final: {final}")

    rpcSend.publishCartProgress(moveDirection, magnitude, final)

    if final or time.time() - config.lastLocationSaveTime > 0.2:
        saveCartLocation()


def getCartYaw():
    return (config.imuDegrees + config.imuDegreesCorrection) % 360


def calculateCartTargetLocation(orientation, distance, moveDirection: 'config.Direction'):

    # for x/y change calculation we need degrees that start to the right (normal circle)
    # take cart orientation and cart move direction into account
    # cart orientation 0 is straight up
    trigDegrees = config.evalTrigDegrees(orientation, moveDirection)
    if trigDegrees is not None:
        config._cartTargetLocationX = config._moveStartX + int(distance * np.cos(np.radians(trigDegrees)))
        config._cartTargetLocationY = config._moveStartY + int(distance * np.sin(np.radians(trigDegrees)))
        config.log(f"move - from: {config.cartLocationX:.0f} / {config.cartLocationY:.0f}, orientation: {orientation}, moveDir: {moveDirection}, deg: {trigDegrees:.0f}, dist: {distance}, to: {config._cartTargetLocationX:.0f} / {config._cartTargetLocationY:.0f}")

    else:
        config.log(
            f"unexpected missing trigDegrees in calculateCartTargetLocation, distance: {distance}, moveDirection: {moveDirection}")



def getCartLocation():
    return round(config.cartLocationX), round(config.cartLocationY)


def getMoveStart():
    return config._moveStartX, config._moveStartY


def getRemainingRotation():
    d = abs(getCartYaw() - config._targetOrientation) % 360
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


def updateDistances(Values):
    """
    cart arduino msg A1 sends measured distance values
    # !A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
    :param Values:
    :return:
    """
    sensorID = Values[0]
    numValues = Values[1]

    config.log(f"updateDistances, sensor: {config.getSensorName(sensorID)} {Values[2:-1]}", publish=False)

    for i in range(numValues):
        try:
            config.distanceList[sensorID][i] = float(Values[2 + i])
        except ValueError:
            config.distanceList[sensorID][i] = 0.0
        except IndexError:
            config.distanceList[sensorID][i] = 0.0
            config.log(f"something is wrong with !A1 message from cart")

    config.distanceSensors[sensorID]['timestamp'] = time.time()
    config.distanceSensors[sensorID]['newValuesShown'] = False


def setSensorDataShown(sensorID, new):
    config.distanceSensors[sensorID]['newValuesShown'] = new


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
        config.imuDegreesCorrection = (lastYaw - config.imuDegrees) % 360

    else:
        config.cartLocationX = 0
        config.cartLocationY = 0
        config._cartOrientation = 0
        config.imuDegreesCorrection = 0

    config.log(f"imuYaw: {config.imuDegrees}, cartYawCorrection: {config.imuDegreesCorrection}, roll: {config._imuRoll}, pitch: {config._imuPitch}, cartX: {config.cartLocationX}, cartY: {config.cartLocationY}, lastYaw: {lastYaw}")


def setMovementBlocked(new):

    config._movementBlocked = new
    config._timeMovementBlocked = time.time()


def getMovementBlocked():
    return config._movementBlocked, config._timeMovementBlocked


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


    if not config.standAloneMode:

        # start arduino thread
        config.log(f"start cart initialization thread")
        startupThread = threading.Thread(target=cartInit, args={})
        startupThread.setName("cartInit")
        startupThread.start()

        # TODO: currently always have kinect power on, could be improved
        time.sleep(2)
        while config.arduinoStatus == 0:
            config.log(f"something is blocking the connection with the arduino, another instance of cartControl active?")
            time.sleep(1)

        config.log(f"turn on kinect power")
        arduinoSend.powerKinect(True)

        # start communication watchdog thread
        config.log(f"start rpyc communication watchdog")
        navThread = threading.Thread(target=watchDog.watchDog, args={})
        navThread.setName("connectionWatchDog")
        navThread.start()

        # start listener for rpc commands, does not return
        config.log(f"start listening for rpyc commands on port {config.MY_RPC_PORT}")
        myConfig = {"allow_all_attrs": True, "allow_pickle": True}
        server = ThreadedServer(rpcReceive.rpcListener, port=config.MY_RPC_PORT, protocol_config=myConfig)
        server.start()

        # code here never executes


