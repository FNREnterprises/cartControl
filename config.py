
import time
import datetime
import sys
import os
import numpy as np
import psutil
import cv2
import json
from enum import Enum


# configuration values for cart arduino infrared distance limits
SHORT_RANGE_MIN = 6
SHORT_RANGE_MAX = 18
LONG_RANGE_MIN = 12
LONG_RANGE_MAX = 34
delayBetweenDistanceMeasurements = 1  # can probably be lower when battery fully loaded

import rpcSend

##################################################################
# cartControl can run as slave of navManager or in standalone mode
##################################################################
standAloneMode = False  # False -> slave mode,  True -> standalone mode

MY_IP = "192.168.0.17"
MY_RPC_PORT = 20001


# values for arduino to calculate distance mm/s from speed value
# (values depend on battery level too, see excel sheet for calc)
SPEED_FACTOR = 1.49
SPEED_OFFSET = 44.6
SPEED_FACTOR_SIDEWAY = 0.5
SPEED_FACTOR_DIAGONAL = 0.63

obstacleInfo = []

class Direction(Enum):
    STOP = 0
    FORWARD = 1
    FOR_DIAG_RIGHT = 2
    FOR_DIAG_LEFT = 3
    LEFT = 4
    RIGHT = 5
    BACKWARD = 6
    BACK_DIAG_RIGHT = 7
    BACK_DIAG_LEFT = 8
    ROTATE_LEFT = 9
    ROTATE_RIGHT = 10


# odometry is only active when cart is moving
_cartMoving = False
_cartMoveTimeout = 0
_cartRotating = False
_requestedCartSpeed = 0
_cartSpeedFromOdometry = 0

_movementBlocked = False
_timeMovementBlocked = None
_moveDistanceRequested = 0
_moveStartX = 0
_moveStartY = 0
_targetOrientation = 0
_moveStartTime = None
_lastCartCommand = ""
_orientationBeforeMove = 0
_moveDirection = 0

# Current cart position (center of cart) relative to map center, values in mm
_imuOrientation = 0
_cartOrientationCorrection = 0
_cartLocationX = 0
_cartLocationY = 0
_cartTargetLocationX = 0
_cartTargetLocationY = 0

_lastBatteryCheckTime = time.time()
_batteryStatus = None
arduinoStatus = 0
_mVolts12V = 0

# pixel-width in mm at 13 cm distance from ground
# PIX_PER_MM = 0.7

navManager = None

taskStarted = time.time()
_f = None


# def startlog():
#    logging.basicConfig(filename="cartControl.log", level=logging.INFO, format='%(asctime)s - %(name)s - %(message)s', filemode="w")




def log(msg):

    logtime = str(datetime.datetime.now())[11:]
    print(f"{logtime} - {msg}")

    rpcSend.publishLog("cartControl - " + msg)



def saveImg(img, frameNr):
    try:
        cv2.imwrite(f"C:/cartControl/floorImages/floor_{frameNr}.jpg", img)
    except:
        log(f"cartGlobal, saveImg exception {sys.exc_info()[0]}")


def setCartMoveDistance(distanceMm):
    global _moveDistanceRequested, _moveStartX, _moveStartY, _moveStartTime

    _moveDistanceRequested = distanceMm
    _moveStartX = _cartLocationX
    _moveStartY = _cartLocationY
    _moveStartTime = time.time()


def setCartMoving(newState, timeout=0):
    global _cartMoving, _cartMoveTimeout  # , _lastTrackedMoveDuration

    # log(f"setCartMoving {new}")
    _cartMoving = newState
    _cartMoveTimeout = timeout

    if isCartRotating():
        setCartRotating(False)


def isCartMoving():
    return _cartMoving


def setCartRotating(new):
    global _cartRotating

    # log(f"setCartRotating {new}")
    _cartRotating = new
    if isCartMoving():
        setCartMoving(False)


def setTargetOrientation(relAngle):  # CartRotating(new):

    global _targetOrientation

    _targetOrientation = (getCartOrientation() + relAngle) % 360
    log(f"setTargetOrientation -> currOrientation: {getCartOrientation()}, relAngle: {relAngle}, targetOrientation: {_targetOrientation}")
    locX, locY = getCartLocation()
    if navManager is not None:
        navManager.root.updateCartInfo(locX, locY, getCartOrientation())
    setCartRotating(True)


def isCartRotating():
    return _cartRotating


def setMovementBlocked(new):
    global _movementBlocked, _timeMovementBlocked

    _movementBlocked = new
    _timeMovementBlocked = time.time()


def getMovementBlocked():
    return _movementBlocked, _timeMovementBlocked


def setRequestedCartSpeed(speed):
    global _requestedCartSpeed

    _requestedCartSpeed = speed


def getRequestedCartSpeed():
    return _requestedCartSpeed


def evalTrigDegrees(orientation, moveDirection):
    """
    set moveDegrees based on moveDirection
    :param orientation:
    :param moveDirection:
    :return:
    """
    moveDegrees = None
    if moveDirection == Direction.FORWARD.value:
        moveDegrees = 180
    elif moveDirection == Direction.BACKWARD.value:
        moveDegrees = 0
    elif moveDirection == Direction.LEFT.value:
        moveDegrees = 90
    elif moveDirection == Direction.RIGHT.value:
        moveDegrees = - 90
    elif moveDirection == Direction.FOR_DIAG_LEFT.value:
        moveDegrees = 135
    elif moveDirection == Direction.FOR_DIAG_RIGHT.value:
        moveDegrees = - 135
    elif moveDirection == Direction.BACK_DIAG_LEFT.value:
        moveDegrees = 45
    elif moveDirection == Direction.BACK_DIAG_RIGHT.value:
        moveDegrees = -45
    if moveDegrees is None:
        return None
    else:
        return (orientation + moveDegrees + 90) % 360  # make 0 degrees pointing to the right in circle


def updateCartLocation(orientation, distance, moveDirection):
    """
    based on update messages from the cart update the current cart position
    :param orientation:
    :param distance:
    :param moveDirection:
    :return:
    """

    global _cartLocationX, _cartLocationY

    # for x/y change calculation we need degrees that start to the right (normal circle)
    # take cart orientation and cart move direction into account
    # cart orientation 0 is straight up
    trigDegrees = evalTrigDegrees(orientation, moveDirection)
    if trigDegrees is not None:
        _cartLocationX = _moveStartX + int(distance * np.cos(np.radians(trigDegrees)))
        _cartLocationY = _moveStartY + int(distance * np.sin(np.radians(trigDegrees)))
    #log(f"updateCartLocation, from X,Y {_moveStartX},{_moveStartY}, dist: {distance}, ori: {orientation}, dir: {moveDirection}, deg: {trigDegrees}, x/y: {_cartLocationX}/{_cartLocationY}")


def calculateCartTargetLocation(orientation, distance, moveDirection):

    global _cartTargetLocationX, _cartTargetLocationY

    # for x/y change calculation we need degrees that start to the right (normal circle)
    # take cart orientation and cart move direction into account
    # cart orientation 0 is straight up
    trigDegrees = evalTrigDegrees(orientation, moveDirection)
    if trigDegrees is not None:
        _cartTargetLocationX = _moveStartX + int(distance * np.cos(np.radians(trigDegrees)))
        _cartTargetLocationY = _moveStartY + int(distance * np.sin(np.radians(trigDegrees)))

    print(f"target x/y: {_cartTargetLocationX:2f} / {_cartTargetLocationY:2f}")

def getCartLocation():
    return _cartLocationX, _cartLocationY


def setImuOrientation(new):

    global _imuOrientation

    _imuOrientation = round(new)


def getCartOrientation():
    return (_imuOrientation + _cartOrientationCorrection) % 360


def getMoveStart():
    return _moveStartX, _moveStartY


def getRemainingRotation():
    d = abs(getCartOrientation() - _targetOrientation) % 360
    return 360 - d if d > 180 else d


def getLastBatteryCheckTime():
    return _lastBatteryCheckTime


def setLastBatteryCheckTime(newTime):
    global _lastBatteryCheckTime

    _lastBatteryCheckTime = newTime


def getBatteryStatus():
    return _batteryStatus


def updateBatteryStatus():
    global _batteryStatus

    _batteryStatus = psutil.sensors_battery()
    setLastBatteryCheckTime(time.time())

    # inform navManager about low battery
    if _batteryStatus.percent < 25:
        msg = f"low battery: {_batteryStatus.percent:.0f} percent"
        if standAloneMode or navManager is None:
            log(msg)
        else:
            navManager.root.lowBattery("cart - " + msg)


def getSensorName(sensorID):
    return ["", "VL nah", "VL fern", "VR nah", "VR fern", "left", "right", "HL nah", "HL fern", "HR nah", "HL fern"][
        sensorID + 1]


def getVoltage12V():
    return _mVolts12V


def setVoltage12V(value):
    global _mVolts12V

    _mVolts12V = value


def saveCartLocation():
    # Saving the objects:
    cartData = {'posX': int(_cartLocationX), 'posY': int(_cartLocationY), 'orientation': getCartOrientation()}
    filename = f"cartLocation.json"
    with open(filename, "w") as write_file:
        json.dump(cartData, write_file)


def loadCartLocation():
    """
    we load the cart location from the file system
    as the carts imu is reset with each start of the cart the carts orientation might be 0
    set an orientationCorrection value to account for this
    :return:
    """
    global _cartLocationX, _cartLocationY, _cartOrientationCorrection

    # Getting back the last saved cart data
    filename = f"cartLocation.json"
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            cartData = json.load(read_file)

        _cartLocationX = cartData['posX']
        _cartLocationY = cartData['posY']
        lastOrientation = cartData['orientation']
        _cartOrientationCorrection = (lastOrientation - _imuOrientation) % 360

    else:
        _cartLocationX = 0
        _cartLocationY = 0
        _cartOrientation = 0
        _cartOrientationCorrection = 0

    print(f"loadCartLocation, cartOrientationCorrection: {_cartOrientationCorrection}")


def setMoveDirection(direction):
    global _moveDirection

    _moveDirection = direction


